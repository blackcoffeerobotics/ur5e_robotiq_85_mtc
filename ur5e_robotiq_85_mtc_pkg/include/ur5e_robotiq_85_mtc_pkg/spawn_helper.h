// Copyright (c) 2023 Black Coffee Robotics

/**
 * Header file for the Spawn Helper Class
 */

#ifndef SPAWN_HELPER_H_
#define SPAWN_HELPER_H_

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>

class SpawnHelper {
 public:
  ros::NodeHandle nh_;

  explicit SpawnHelper(const ros::NodeHandle& nh);

  geometry_msgs::Pose getBaseFramePose(std::string link_name);

  geometry_msgs::Pose getObjectPose(std::string base_frame_name,
    std::string object_name, std::vector<double> origin_compensation);
};

#endif  // SPAWN_HELPER_H_
