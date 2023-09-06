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
    "orientation_constraint_name", orientation_constraint_name_);

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

  // Connect Stage - Custom Grasp
  {
    task_->add(std::move(connect_stage_for_custom_grasp_));
  }

  // Serial Container - Pick
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

      // Add object_center_offset
      modifyGripperTransform(
        object_center_offset_axis_, object_center_offset_);

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
      serial_container_for_pick_->insert(std::move(compute_ik_custom_grasp));

      // remove object_pickup_offset
      modifyGripperTransform(
        object_center_offset_axis_, -object_center_offset_);
    }
    task_->add(std::move(serial_container_for_pick_));
  }
  return initTask(task_name_ + "_check_target_pose");
}

bool PickAndPlace::approachObject() {
  // Task 1 - approach object
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Approach Object Stage" + bash_colours.at("reset"));

  // reset task
  resetTask(task_name_ + "_approach_object");

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

  // Open Hand Stage
  {
    task_->add(std::move(open_hand_stage_));
  }

  // Connect Stage - Normal Grasp
  {
    task_->add(std::move(connect_stage_for_grasp_));
  }

  // Serial Container - Pick
  {
    // Approach object - Substage
    {
      approach_object_stage_->setMinMaxDistance(
          approach_object_min_dist_, approach_object_max_dist_);

      // Set hand forward direction
      approach_object_stage_->setDirection(getVectorDirection(approach_axis_));
      serial_container_for_pick_->insert(std::move(approach_object_stage_));
    }

    // Generate grasp pose - Substage
    {
      generate_grasp_pose_stage_->setObject(object_name_);
      generate_grasp_pose_stage_->setMonitoredStage(current_state_ptr_);

      // Add object_pickup_offset_
      modifyGripperTransform(
        object_center_offset_axis_, object_center_offset_);

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
      serial_container_for_pick_->insert(std::move(compute_ik));

      // remove object_pickup_offset
      modifyGripperTransform(
        object_center_offset_axis_, -object_center_offset_);
    }
    task_->add(std::move(serial_container_for_pick_));
  }
  return initTask(task_name_ + "_approach_object");
}

bool PickAndPlace::liftAndPlaceObject() {
  // Task 2 - lift object
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Lift And Place Object Stage" + bash_colours.at("reset"));

  // reset task
  resetTask(task_name_ + "_lift_and_place_object");

  // Applicability Stage
  current_state_ptr_ = nullptr;
  {
    applicability_filter_stage_->setPredicate([this](
      const moveit::task_constructor::SolutionBase& s, std::string& comment) {
      return expectAttached(s, comment, object_name_, true);
    });
    current_state_ptr_ = applicability_filter_stage_.get();
    task_->add(std::move(applicability_filter_stage_));
  }

  // Allow Collision - Stage
  {
    allow_hand_object_collision_stage_->allowCollisions(
      object_name_, task_->getRobotModel()->getJointModelGroup(
        hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), true);
    task_->add(std::move(allow_hand_object_collision_stage_));
  }

  // Lift Object Stage
  {
    lift_object_stage_->setMinMaxDistance(
        lift_object_min_dist_, lift_object_max_dist_);

    // Set hand upward direction
    lift_object_stage_->setDirection(getVectorDirection(lift_axis_));
    task_->add(std::move(lift_object_stage_));
  }

  // Connect Stage - Place
  {
    moveit_msgs::Constraints upright_constraint;
    upright_constraint.name = orientation_constraint_name_;
    connect_stage_for_place_->setPathConstraints(upright_constraint);
    task_->add(std::move(connect_stage_for_place_));
  }

  // Serial Container - Place
  {
    // Drop Object - Substage
    {
      drop_object_stage_->setMinMaxDistance(
          drop_object_min_dist_, drop_object_max_dist_);

      // Set hand downward direction
      drop_object_stage_->setDirection(getVectorDirection(drop_axis_));
      serial_container_for_place_->insert(std::move(drop_object_stage_));
    }

    // Generate Place Pose - Substage
    {
      // set object name
      generate_place_pose_stage_->setObject(object_name_);

      // Set target pose
      geometry_msgs::PoseStamped p;
      p.header.frame_id = planning_frame_;
      p.pose = place_pose_;

      // netural orientation
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      p.pose.orientation = tf2::toMsg(q);
      generate_place_pose_stage_->setPose(p);
      generate_place_pose_stage_->setMonitoredStage(current_state_ptr_);

      // Add object_center_offset_ to grasp pose
      modifyGripperTransform(
        object_center_offset_axis_, object_center_offset_);

      // Compute IK - Substage
      auto wrapper =
        std::make_unique<moveit::task_constructor::stages::ComputeIK>(
          "place pose IK", std::move(generate_place_pose_stage_));
      wrapper->setMaxIKSolutions(5);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(
        moveit::task_constructor::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(
        moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
      serial_container_for_place_->insert(std::move(wrapper));

      // remove object_center_offset_ from grasp pose
      modifyGripperTransform(
        object_center_offset_axis_, -object_center_offset_);
    }

    // Open Hand - Substage
    {
      serial_container_for_place_->insert(std::move(open_hand_stage_));
    }

    // Forbid collision - Substage
    {
      forbid_hand_object_collision_stage_->allowCollisions(object_name_,
        *task_->getRobotModel()->getJointModelGroup(hand_group_name_), false);
      serial_container_for_place_->insert(
        std::move(forbid_hand_object_collision_stage_));
    }

    // Detach Object - Substage
    {
      detach_object_stage_->detachObject(object_name_, hand_frame_);
      serial_container_for_place_->insert(std::move(detach_object_stage_));
    }
    task_->add(std::move(serial_container_for_place_));
  }
  return initTask(task_name_ + "_lift_and_place_object");
}

bool PickAndPlace::retreatAndHome() {
  // Task 3 - retreat and home
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Retreat And Home Stage" + bash_colours.at("reset"));

  // reset task
  resetTask(task_name_ + "_retreat_and_home");

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

  // Retreat Object Stage
  {
    retreat_object_stage_->setMinMaxDistance(
        retreat_object_min_dist_, retreat_object_max_dist_);

    // Set hand backward direction
    retreat_object_stage_->setDirection(getVectorDirection(retreat_axis_));
    task_->add(std::move(retreat_object_stage_));
  }

  // Go to Home
  {
    task_->add(std::move(move_to_home_stage_));
  }

  return initTask(task_name_ + "_retreat_and_home");
}


bool PickAndPlace::executePipeline() {
  // pick with an open grasp
  openGripperAction();

  // check stage sanity of target pose
  if (!checkTargetPose()) {
    removeObject(object_name_);
    return false;
  }

  if (!tryTask(false, true)) {
    removeObject(object_name_);
    ROS_ERROR_STREAM("Arm cannot reach target pose");
    return false;
  }

  // check stage sanity of approach object
  if (!approachObject()) {
    removeObject(object_name_);
    return false;
  }

  if (!tryTask()) {
    removeObject(object_name_);
    ROS_ERROR_STREAM("Arm cannot approach object");
    return false;
  }

  // pick with a closed grasp
  closeGripperAction();
  // attach object to gripper
  move_group_->attachObject(object_name_, hand_frame_, gripper_touch_links_);

  // check stage sanity of lift object
  if (!liftAndPlaceObject()) {
    removeObject(object_name_);
    move_group_->detachObject(object_name_);
    return false;
  }

  if (!tryTask()) {
    removeObject(object_name_);
    move_group_->detachObject(object_name_);
    ROS_ERROR_STREAM("Arm cannot lift and place the object");
    return false;
  }

  // wait for object to settle
  ros::Duration(1.0).sleep();

  // check stage sanity of retreat and home
  if (!retreatAndHome()) {
    removeObject(object_name_);
    return false;
  }

  if (!tryTask()) {
    removeObject(object_name_);
    ROS_ERROR_STREAM("Arm cannot retreat and home");
    return false;
  }

  return true;
}
