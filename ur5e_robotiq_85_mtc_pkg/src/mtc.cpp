// Copyright (c) 2023 Black Coffee Robotics

/**
 * Source file for the MTC Library class
 */

#include <ur5e_robotiq_85_mtc_pkg/mtc.h>


MTCLibrary::MTCLibrary(const ros::NodeHandle& nh) : nh_(nh) {
  loadParameters();

  // initialize move_group
  move_group_ =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      planning_group_);

  // initialize planning scene publisher
  planning_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(
    "planning_scene", 1);

  // initialize gripper publisher
  gripper_pub_ = nh_.advertise<control_msgs::GripperCommandActionGoal>(
    gripper_cmd_topic_, 1);

  // initialize frames
  planning_frame_ = move_group_->getPlanningFrame();
  hand_frame_ = move_group_->getEndEffectorLink();
}

void MTCLibrary::loadParameters() {
  // load Parameters
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Loading Parameters" + bash_colours.at("reset"));

  ros::NodeHandle pnh("~");
  std::string param_ns = "mtc/";
  std::size_t errors = 0;

  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "max_solutions", max_solutions_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "scaling_factor", scaling_factor_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "planning_timeout", planning_timeout_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "planning_group", planning_group_);

  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "arm_group_name", arm_group_name_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "hand_group_name", hand_group_name_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "hand_open_pose", hand_open_pose_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "hand_close_pose", hand_close_pose_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "arm_home_pose", arm_home_pose_);

  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "grasp_frame_transform", grasp_frame_transform_);

  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "fixed_constraint_path", fixed_constraint_path_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "free_constraint_path", free_constraint_path_);

  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "gripper_cmd_topic", gripper_cmd_topic_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "gripper_touch_links", gripper_touch_links_);

  rosparam_shortcuts::shutdownIfError("", errors);
}

void MTCLibrary::setConstraints(std::string constraint_path) {
  // set constraints
  ros::param::set("move_group/constraint_approximations_path",
    constraint_path);
}

std::string MTCLibrary::getPlanningFrame() {
  // get planning frame
  return planning_frame_;
}

void MTCLibrary::spawnObject(
  std::string object_name, geometry_msgs::Pose object_pose,
    std::vector<double> object_dimensions, std::string reference_frame) {
  // spawn objects in the planning scene
  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = reference_frame;
  object.pose.orientation.w = 1.0;
  object.primitives.resize(1);

  int dim_size = object_dimensions.size();

  if (dim_size == 3) {
    object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  } else if (dim_size == 2) {
    object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  } else if (dim_size == 1) {
    object.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  } else {
    ROS_ERROR_STREAM("Object type not supported");
  }
  object.primitives[0].dimensions = object_dimensions;
  object.primitive_poses.push_back(object_pose);
  object.operation = object.ADD;

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(object);
  planning_scene.is_diff = true;
  planning_scene_pub_.publish(planning_scene);

  ros::Duration(1.0).sleep();
}

void MTCLibrary::removeObject(std::string object_name) {
  // remove objects from the planning scene
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Removing Scene Objects" + bash_colours.at("reset"));
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.operation = object.REMOVE;
  planning_scene.world.collision_objects.push_back(object);
  planning_scene_pub_.publish(planning_scene);
}

void MTCLibrary::set_gripper_transform(float transform[6]) {
  grasp_frame_transform_.setIdentity();
  grasp_frame_transform_.translation() =
    Eigen::Vector3d(transform[0], transform[1], transform[2]);
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(transform[3], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(transform[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(transform[5], Eigen::Vector3d::UnitZ());
  grasp_frame_transform_.linear() = q.matrix();
}

bool MTCLibrary::task_plan(bool oneshot) {
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Start searching for task solutions" + bash_colours.at("reset"));

  if (oneshot) {
    return static_cast<bool>(task_->plan(1));
  } else {
    return static_cast<bool>(task_->plan(max_solutions_));
  }
}

bool MTCLibrary::task_execute() {
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Executing solution trajectory" + bash_colours.at("reset"));
  moveit_msgs::MoveItErrorCodes execute_result;

  execute_result = task_->execute(*task_->solutions().front());
  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR_STREAM("Task execution failed and returned: "
      << execute_result.val);

    if (execute_result.val == moveit_msgs::MoveItErrorCodes::PREEMPTED) {
      ROS_ERROR_STREAM("Task execution preempted due to obstacle");
    }

    return false;
  }
  return true;
}

bool MTCLibrary::try_task(bool execute, bool oneshot) {
  time_t start = time(NULL);

  while (time(NULL) - start < planning_timeout_) {
    if (task_plan(oneshot)) {
      ROS_INFO_STREAM(
          bash_colours.at("green") + "Planning succeded"+ bash_colours.at("reset"));

      if (execute) {
        if (!task_execute()) {
          ROS_ERROR("Execution failed");
          return false;
        }
        ROS_INFO_STREAM(bash_colours.at("green") +
          "Execution complete" + bash_colours.at("reset"));
        return true;
      } else {
        ROS_INFO_STREAM(bash_colours.at("green") +
          "Execution disabled" + bash_colours.at("reset"));
        return true;
      }
    } else {
      ROS_ERROR("Planning failed");
    }
  }
  return false;
}
