// Copyright (c) 2023 Black Coffee Robotics

/**
 * Source file for the PickAndPlace Class
 */

#include <ur5e_robotiq_85_mtc_pkg/pick_and_place.h>

PickAndPlace::PickAndPlace(const std::string& task_name,
    const ros::NodeHandle& nh, std::string object_name,
      geometry_msgs::Pose object_pose, geometry_msgs::Pose place_pose) :
        MTCLibrary(nh), task_name_(task_name), object_name_(object_name),
          object_pose_(object_pose), place_pose_(place_pose) {
  // Load Parameters
  loadParameters();
}

void PickAndPlace::loadParameters() {
  // load Parameters
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Loading Parameters for PickAndPlace" + bash_colours.at("reset"));

  ros::NodeHandle pnh("~");
  std::string param_ns = "pick_and_place/";
  std::size_t errors = 0;

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "object_center_offset", object_center_offset_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "object_center_offset_axis", object_center_offset_axis_);

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "approach_object_min_dist", approach_object_min_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "approach_object_max_dist", approach_object_max_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "approach_axis", approach_axis_);

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "lift_object_min_dist", lift_object_min_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "lift_object_max_dist", lift_object_max_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "lift_axis", lift_axis_);

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "drop_object_min_dist", drop_object_min_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "drop_object_max_dist", drop_object_max_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "drop_axis", drop_axis_);

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "retreat_object_min_dist", retreat_object_min_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "retreat_object_max_dist", retreat_object_max_dist_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "retreat_axis", retreat_axis_);

  rosparam_shortcuts::shutdownIfError("", errors);
}

bool PickAndPlace::checkTargetPose() {
  // Task 0 - check target pose
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Check Target Pose Stage" + bash_colours.at("reset"));

  // reset task
  resetTask(task_name_ + "_check_target_pose");

  sampling_planner_->setProperty(
    "goal_joint_tolerance", 1e-3);  // custom grasp requires greater tolerance

  // Applicability Stage
  current_state_ptr_ = nullptr;
  {
    applicability_filter_stage_->setPredicate([this](
      const moveit::task_constructor::SolutionBase& s, std::string& comment) {
      return expectAttached(s, comment, object_name_, false);
    });
    current_state_ptr_ = applicability_filter_stage_.get();
    task_->add(std::move(applicability_filter_stage_));
  }

  // Connect Stage - custom grasp
  {
    task_->add(std::move(connect_stage_for_custom_grasp_));
  }

  // Serial Container to Connect
  {
    // SubStages
    {
      std::vector<geometry_msgs::PoseStamped> CustomPoses;
      geometry_msgs::PoseStamped p;
      p.header.frame_id = planning_frame_;
      p.pose = place_pose_;
      // generate neutral orientation
      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      p.pose.orientation = tf2::toMsg(q);
      CustomPoses.push_back(p);

      // Custom grasp pose - Substage
      generate_custom_pose_stage_->setCustomPoses(CustomPoses);
      generate_custom_pose_stage_->setMonitoredStage(current_state_ptr_);

      // Add object_center_offset_ to grasp pose (ensure +ve values)
      grasp_frame_transform_.translation().y() -= object_center_offset_;

      // Compute IK - Substage
      auto compute_ik_custom_grasp =
        std::make_unique<moveit::task_constructor::stages::ComputeIK>(
          "grasp custom pose IK", std::move(generate_custom_pose_stage_));
      compute_ik_custom_grasp->setMaxIKSolutions(1);
      compute_ik_custom_grasp->setMinSolutionDistance(1.0);
      compute_ik_custom_grasp->setIKFrame(grasp_frame_transform_, hand_frame_);
      compute_ik_custom_grasp->properties().configureInitFrom(
          moveit::task_constructor::Stage::PARENT, { "eef", "group" });
      compute_ik_custom_grasp->properties().configureInitFrom(
          moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
      serial_container_->insert(std::move(compute_ik_custom_grasp));

      // remove object_pickup_offset_ from grasp pose
      grasp_frame_transform_.translation().y() += object_center_offset_;
    }
    task_->add(std::move(serial_container_));
  }
  return initTask(task_name_ + "_check_target_pose");
}



bool PickAndPlace::approachObject() {
  // Task 1 - approach object
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Approach Object Stage" + bash_colours.at("reset"));

  // reset task
  resetTask(task_name_ + "_approach_object");

  const std::string object = object_name_;

  // Applicability Stage
  moveit::task_constructor::Stage* current_state_ptr = nullptr;
  {
    auto current_state =
      std::make_unique<moveit::task_constructor::stages::CurrentState>(
        "current state");

    auto applicability_filter =
        std::make_unique<moveit::task_constructor::stages::PredicateFilter>(
          "applicability test", std::move(current_state));
    applicability_filter->setPredicate([object](
      const moveit::task_constructor::SolutionBase& s, std::string& comment) {
      if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
        comment = "object with id '" + object +
          "' is already attached and cannot be picked";
        return false;
      }
      return true;
    });
    current_state_ptr = applicability_filter.get();
    task_->add(std::move(applicability_filter));
  }


  // Open Hand Stage
  {
    task_->add(std::move(open_hand_stage_));
  }

  // Connect Stage
  {
    task_->add(std::move(connect_stage_for_grasp_));
  }

  // Serial Container
  {
    // Approach object - Substage
    {
      approach_object_stage_->setMinMaxDistance(
          approach_object_min_dist_, approach_object_max_dist_);

      // Set hand forward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.z = 1.0;
      approach_object_stage_->setDirection(vec);
      serial_container_->insert(std::move(approach_object_stage_));
    }

    // Generate grasp pose - Substage
    {
      generate_grasp_pose_stage_->setObject(object);
      generate_grasp_pose_stage_->setMonitoredStage(current_state_ptr);

      // Add object_pickup_offset_ to grasp pose (ensure +ve values)
      grasp_frame_transform_.translation().y() -= object_center_offset_;

      // Compute IK - Substage
      auto compute_ik =
        std::make_unique<moveit::task_constructor::stages::ComputeIK>(
          "grasp pose IK", std::move(generate_grasp_pose_stage_));
      compute_ik->setMaxIKSolutions(20);
      compute_ik->setMinSolutionDistance(1.0);
      compute_ik->setIKFrame(grasp_frame_transform_, hand_frame_);
      compute_ik->properties().configureInitFrom(
          moveit::task_constructor::Stage::PARENT, { "eef", "group" });
      compute_ik->properties().configureInitFrom(
          moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
      serial_container_->insert(std::move(compute_ik));

      // remove object_pickup_offset_ from grasp pose
      grasp_frame_transform_.translation().y() += object_center_offset_;
    }

    // allow collision - Substage
    {
      allow_hand_object_collision_stage_->allowCollisions(
        object_name_, task_->getRobotModel()->getJointModelGroup(
          hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), true);
      serial_container_->insert(std::move(allow_hand_object_collision_stage_));
    }
    task_->add(std::move(serial_container_));
  }

  return initTask(task_name_ + "_approach_object");
}

bool PickAndPlace::executePipeline() {
  // pick with an open grasp
  openGripperAction();

  // check stage sanity of target pose
  if (!checkTargetPose()) {
    return false;
  }

  if (!tryTask(false, true)) {
    ROS_ERROR_STREAM("Arm cannot reach target pose");
    return false;
  }

  // check stage sanity of approach object
  if (!approachObject()) {
    return false;
  }

  if (!tryTask()) {
    ROS_ERROR_STREAM("Arm cannot approach object");
    return false;
  }

  return true;
}
