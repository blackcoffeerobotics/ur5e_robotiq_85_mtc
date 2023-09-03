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

bool PickAndPlace::approachObject() {
  // Task 1 - approach object
  // reset task
  resetTask(task_name_ + "_approach_object");

  // Applicability Stage
  current_state_ptr_ = nullptr;
  {
    applicability_filter_->setPredicate([this](
      const moveit::task_constructor::SolutionBase& s, std::string& comment) {
      return expectAttached(s, comment, object_name_, false);
    });
    current_state_ptr_ = applicability_filter_.get();
    task_->add(std::move(applicability_filter_));
  }

  return initTask(task_name_ + "_approach_object");
}

bool PickAndPlace::executePipeline() {
  // pick with an open grasp
  openGripperAction();
  ros::Duration(1.0).sleep();

  // approach object
  if (!approachObject()) {
    return false;
  }

  if (!tryTask()) {
    ROS_ERROR_STREAM("Arm cannot approach object");
    return false;
  }

  return true;
}
