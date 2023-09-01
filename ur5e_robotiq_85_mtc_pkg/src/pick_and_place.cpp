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
}

