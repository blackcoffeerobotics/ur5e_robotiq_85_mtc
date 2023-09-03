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

  // Task specific parameters
  double object_center_offset_;
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;
  double drop_object_min_dist_;
  double drop_object_max_dist_;
  double retreat_object_min_dist_;
  double retreat_object_max_dist_;

 public:
  PickAndPlace(const std::string& task_name,
    const ros::NodeHandle& nh, std::string object_name,
      geometry_msgs::Pose object_pose, geometry_msgs::Pose place_pose);

  void loadParameters() override;

  bool approachObject();
  bool executePipeline() override;
};

#endif  // PICK_AND_PLACE_H_
