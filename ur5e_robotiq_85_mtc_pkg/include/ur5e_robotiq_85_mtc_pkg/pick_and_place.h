// Copyright (c) 2023 Black Coffee Robotics

/**
 * Header file for the Pick and Place Class
 */

#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

// Import from base class
#include <ur5e_robotiq_85_mtc_pkg/mtc.h>
#include <string>


class PickAndPlace : public MTCLibrary {
 private:
  ros::NodeHandle nh_;
  std::string task_name_;
  std::string object_name_;
  geometry_msgs::Pose object_pose_;
  geometry_msgs::Pose place_pose_;

 public:
  PickAndPlace(const std::string& task_name,
    const ros::NodeHandle& nh, std::string object_name,
      geometry_msgs::Pose object_pose, geometry_msgs::Pose place_pose);



};

#endif  // PICK_AND_PLACE_H_
