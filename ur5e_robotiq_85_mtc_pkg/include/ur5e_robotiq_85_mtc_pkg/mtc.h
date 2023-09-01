// Copyright (c) 2023 Black Coffee Robotics

/**
 * Header file for the MTC Library Class
 */

#ifndef MTC_H_
#define MTC_H_

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/generate_custom_pose.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/LinkStates.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <object_recognition_msgs/ObjectType.h>

#include <vector>
#include <string>
#include <tuple>
#include <map>

const std::map<
  std::string, std::string> bash_colours = {
    {"black", "\033[0;30m"},
    {"red", "\033[0;31m"},
    {"green", "\033[0;32m"},
    {"yellow", "\033[0;33m"},
    {"blue", "\033[0;34m"},
    {"purple", "\033[0;35m"},
    {"cyan", "\033[0;36m"},
    {"white", "\033[0;37m"},
    {"reset", "\033[0m"}
  };

class MTCLibrary {
 public:
  ros::NodeHandle nh_;
  ros::Publisher gripper_pub_, planning_scene_pub_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;

  std::string task_name_;
  std::string planning_frame_, hand_frame_;

  std::vector<std::string> touch_links_;
  geometry_msgs::Pose object_pose_;

  std::string object_name_;
  std::vector<double> object_dimensions_;
  float default_transform_[6];

  moveit::task_constructor::TaskPtr task_;

  // MTC Parameters
  int max_solutions_;
  double scaling_factor_;
  double planning_timeout_;
  std::string planning_group_;

  std::string arm_group_name_;
  std::string hand_group_name_;
  std::string eef_name_;

  std::string hand_open_pose_;
  std::string hand_close_pose_;
  std::string arm_home_pose_;

  Eigen::Isometry3d grasp_frame_transform_;

  std::string fixed_constraint_path_;
  std::string free_constraint_path_;

  std::string gripper_cmd_topic_;
  std::vector<std::string> gripper_touch_links_;


  // Functions
  explicit MTCLibrary(const ros::NodeHandle& nh);

  void set_gripper_transform(float transform[6]);

  bool task_plan(bool oneshot);
  bool task_execute();
  bool try_task(bool execute = true, bool oneshot = false);
  bool execute_pipeline();

  void openGripperAction();
  void closeGripperAction();
  void setConstraints(std::string constraint_path);

  void loadParameters();

  std::string getPlanningFrame();

  void spawnObject(
    std::string object_name, geometry_msgs::Pose object_pose,
      std::vector<double> object_dimensions, std::string reference_frame);
  void removeObject(std::string object_name);
};

#endif  // MTC_H_