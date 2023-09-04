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

  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "object_center_offset", object_center_offset_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "approach_object_min_dist", approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "approach_object_max_dist", approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "lift_object_min_dist", lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "lift_object_max_dist", lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "drop_object_min_dist", drop_object_min_dist_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "drop_object_max_dist", drop_object_max_dist_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "retreat_object_min_dist", retreat_object_min_dist_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "retreat_object_max_dist", retreat_object_max_dist_);

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

  // Open Hand Stage
  {
    auto stage =
      std::make_unique<moveit::task_constructor::stages::MoveTo>(
        "open hand", sampling_planner_);
    stage->setGroup(hand_group_name_);
    stage->setGoal(hand_open_pose_);
    task_->add(std::move(stage));
  }

  // Allow collision for gripper and object
  {
    auto stage =
      std::make_unique<
        moveit::task_constructor::stages::ModifyPlanningScene>(
        "allow collision (hand,object)");
    stage->allowCollisions(object_name_,
      *task_->getRobotModel()->getJointModelGroup(hand_group_name_), true);
    task_->add(std::move(stage));
  }

  // Move to Pick Connect Stage - non cartesian
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(
      "connect to pick",
        moveit::task_constructor::stages::Connect::GroupPlannerVector{
          { arm_group_name_, sampling_planner_ } });
    stage->setTimeout(60.0);
    stage->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT);
    task_->add(std::move(stage));
  }

  // Serial Container to Connect
  {
    auto grasp =
      std::make_unique<moveit::task_constructor::SerialContainer>(
        "pick object");
    task_->properties().exposeTo(grasp->properties(),
      { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT,
        { "eef", "hand", "group", "ik_frame" });

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

      auto stage =
        std::make_unique<moveit::task_constructor::stages::GenerateCustomPose>(
          "generate custom grasp pose");
      stage->setCustomPoses(CustomPoses);
      stage->properties().configureInitFrom(
          moveit::task_constructor::Stage::PARENT);
      stage->setMonitoredStage(current_state_ptr_);

      // Add object_center_offset_ to grasp pose (ensure +ve values)
      grasp_frame_transform_.translation().y() -= object_center_offset_;

      // Compute IK - substage
      auto wrapper =
        std::make_unique<moveit::task_constructor::stages::ComputeIK>(
          "grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(20);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(
          moveit::task_constructor::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(
          moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));

      // remove object_center_offset_ from grasp pose
      grasp_frame_transform_.translation().y() += object_center_offset_;
    }
    task_->add(std::move(grasp));
  }
  return initTask(task_name_ + "_check_target_pose");
}



bool PickAndPlace::approachObject() {
  // Task 1 - approach object
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

  return initTask(task_name_ + "_approach_object");
}

bool PickAndPlace::executePipeline() {
  // pick with an open grasp
  openGripperAction();
  ros::Duration(1.0).sleep();

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
