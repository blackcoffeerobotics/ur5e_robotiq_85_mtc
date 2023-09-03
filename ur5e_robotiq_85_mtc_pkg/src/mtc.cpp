// Copyright (c) 2023 Black Coffee Robotics

/**
 * Source file for the MTC Library class
 */

#include <ur5e_robotiq_85_mtc_pkg/mtc.h>


MTCLibrary::MTCLibrary(const ros::NodeHandle& nh) : nh_(nh) {
  loadParameters();
  initializePlannersAndStages();

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
    "Loading Parameters for MTCLibrary" + bash_colours.at("reset"));

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
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "gripper_open_position", gripper_open_position_);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "gripper_closed_position", gripper_closed_position_);

  rosparam_shortcuts::shutdownIfError("", errors);
}

void MTCLibrary::initializePlannersAndStages() {
  // initialize planners
  sampling_planner_ =
    std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>();
  sampling_planner_->setProperty(
    "max_velocity_scaling_factor", scaling_factor_);
  sampling_planner_->setProperty(
    "max_acceleration_scaling_factor", scaling_factor_);
  sampling_planner_->setProperty(
    "goal_joint_tolerance", 1e-5);

  cartesian_planner_ =
    std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(scaling_factor_);
  cartesian_planner_->setMaxAccelerationScalingFactor(scaling_factor_);
  cartesian_planner_->setStepSize(.01);

  current_state_ =
    std::make_unique<moveit::task_constructor::stages::CurrentState>(
      "current state");

  applicability_filter_ =
    std::make_unique<moveit::task_constructor::stages::PredicateFilter>(
      "applicability test", std::move(current_state_));
}

void MTCLibrary::resetTask(std::string task_name) {
  // reset the task variable
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  task_->stages()->setName(task_name);
  task_->loadRobotModel();

  // Set task properties
  task_->setProperty("group", arm_group_name_);
  task_->setProperty("eef", eef_name_);
  task_->setProperty("hand", hand_group_name_);
  task_->setProperty("hand_grasping_frame", hand_frame_);
  task_->setProperty("ik_frame", hand_frame_);
}

bool MTCLibrary::initTask(std::string task_name) {
  // initialize the task after appending the stages
  // Check for failure
  try {
    task_->init();
  } catch (moveit::task_constructor::InitStageException& e) {
    ROS_ERROR_STREAM("Initialization failed for " << task_name << ": "
      << e);
    return false;
  }
  return true;
}

void MTCLibrary::setConstraints(std::string constraint_type) {
  // set constraints
  if (constraint_type == "fixed") {
    ros::param::set("move_group/constraint_approximations_path",
      fixed_constraint_path_);
  } else if (constraint_type == "free") {
    ros::param::set("move_group/constraint_approximations_path",
      free_constraint_path_);
  } else {
    ROS_ERROR_STREAM("Constraint type not supported");
  }
}

std::string MTCLibrary::getPlanningFrame() {
  // get planning frame
  return planning_frame_;
}

void MTCLibrary::openGripperAction() {
  // open gripper
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Opening Gripper" + bash_colours.at("reset"));
  control_msgs::GripperCommandActionGoal gripper_msg;
  gripper_msg.goal.command.position = gripper_open_position_;
  gripper_msg.goal.command.max_effort = 100;
  gripper_pub_.publish(gripper_msg);
  ros::Duration(1.0).sleep();
}

void MTCLibrary::closeGripperAction() {
  // close gripper
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Closing Gripper" + bash_colours.at("reset"));
  control_msgs::GripperCommandActionGoal gripper_msg;
  gripper_msg.goal.command.position = gripper_closed_position_;
  gripper_msg.goal.command.max_effort = 100;
  gripper_pub_.publish(gripper_msg);
  ros::Duration(1.0).sleep();
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

void MTCLibrary::setGripperTransform(float transform[6]) {
  grasp_frame_transform_.setIdentity();
  grasp_frame_transform_.translation() =
    Eigen::Vector3d(transform[0], transform[1], transform[2]);
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(transform[3], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(transform[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(transform[5], Eigen::Vector3d::UnitZ());
  grasp_frame_transform_.linear() = q.matrix();
}

bool MTCLibrary::expectAttached(
  const moveit::task_constructor::SolutionBase& s,
    std::string& comment, std::string object, bool state) {
  if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
    comment = object + " is attached to the gripper";
    return state;
  }
  comment = object + " is not attached to the gripper";
  return !state;
}

bool MTCLibrary::taskPlan(bool oneshot) {
  ROS_INFO_STREAM(bash_colours.at("green") +
    "Start searching for task solutions" + bash_colours.at("reset"));

  if (oneshot) {
    return static_cast<bool>(task_->plan(1));
  } else {
    return static_cast<bool>(task_->plan(max_solutions_));
  }
}

bool MTCLibrary::taskExecute() {
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

bool MTCLibrary::tryTask(bool execute, bool oneshot) {
  time_t start = time(NULL);

  while (time(NULL) - start < planning_timeout_) {
    if (taskPlan(oneshot)) {
      ROS_INFO_STREAM(
          bash_colours.at("green") +
            "Planning succeded"+ bash_colours.at("reset"));

      if (execute) {
        if (!taskExecute()) {
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

bool MTCLibrary::executePipeline() {
  return false;
}
