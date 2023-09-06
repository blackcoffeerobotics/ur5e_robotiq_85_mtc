// Copyright (c) 2023 Black Coffee Robotics

/**
 * Source file for the MTC Library class
 */

#include <ur5e_robotiq_85_mtc_pkg/mtc.h>


MTCLibrary::MTCLibrary(const ros::NodeHandle& nh) : nh_(nh) {
  loadParameters();

  // set package path
  package_path_ = ros::package::getPath("ur5e_robotiq_85_mtc_pkg") + "/";

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

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "max_solutions", max_solutions_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "scaling_factor", scaling_factor_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "planning_timeout", planning_timeout_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "planning_group", planning_group_);

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "arm_group_name", arm_group_name_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "hand_group_name", hand_group_name_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "hand_open_pose", hand_open_pose_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "hand_close_pose", hand_close_pose_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "arm_home_pose", arm_home_pose_);

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "grasp_frame_transform", grasp_frame_transform_);

  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "gripper_cmd_topic", gripper_cmd_topic_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "gripper_touch_links", gripper_touch_links_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "gripper_open_position", gripper_open_position_);
  errors += !rosparam_shortcuts::get("", pnh, param_ns +
    "gripper_closed_position", gripper_closed_position_);

  rosparam_shortcuts::shutdownIfError("", errors);
}

void MTCLibrary::initializePlannersAndStages() {
  // Planners
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

  // Stages
  current_state_stage_ =
    std::make_unique<moveit::task_constructor::stages::CurrentState>(
      "current state stage");

  applicability_filter_stage_ =
    std::make_unique<moveit::task_constructor::stages::PredicateFilter>(
      "applicability filter stage", std::move(current_state_stage_));

  // MoveTo Stages

  open_hand_stage_ =
    std::make_unique<moveit::task_constructor::stages::MoveTo>(
      "open hand stage", sampling_planner_);
  open_hand_stage_->setGroup(hand_group_name_);
  open_hand_stage_->setGoal(hand_open_pose_);

  move_to_home_stage_ =
    std::make_unique<moveit::task_constructor::stages::MoveTo>(
      "move to home stage", sampling_planner_);

  move_to_home_stage_->setGroup(arm_group_name_);
  move_to_home_stage_->setGoal(arm_home_pose_);

  // Modify Planning Scene Stages

  allow_hand_object_collision_stage_ =
    std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
      "allow hand object collision stage");
  forbid_hand_object_collision_stage_ =
    std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
      "forbid hand object collision stage");
  detach_object_stage_ =
    std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
      "detach object stage");

  // Connect Stages

  connect_stage_for_custom_grasp_ =
    std::make_unique<moveit::task_constructor::stages::Connect>(
      "connect to custom grasp",
        moveit::task_constructor::stages::Connect::GroupPlannerVector{
          { arm_group_name_, sampling_planner_ },
            { hand_group_name_, sampling_planner_ } });
  connect_stage_for_custom_grasp_->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT);

  connect_stage_for_grasp_ =
    std::make_unique<moveit::task_constructor::stages::Connect>(
      "connect to grasp",
        moveit::task_constructor::stages::Connect::GroupPlannerVector{
          { arm_group_name_, sampling_planner_ } });
  connect_stage_for_grasp_->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT);

  connect_stage_for_place_ =
    std::make_unique<moveit::task_constructor::stages::Connect>(
      "connect to place",
        moveit::task_constructor::stages::Connect::GroupPlannerVector{
          { arm_group_name_, sampling_planner_ } });
  connect_stage_for_place_->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT);

  // Generate Pose Stages

  generate_custom_pose_stage_ =
    std::make_unique<moveit::task_constructor::stages::GenerateCustomPose>(
      "generate custom grasp pose");
  generate_custom_pose_stage_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT);

  generate_grasp_pose_stage_ =
    std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>(
      "generate grasp pose");
  generate_grasp_pose_stage_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT);
  generate_grasp_pose_stage_->properties().set("marker_ns", "grasp_pose");
  generate_grasp_pose_stage_->setPreGraspPose(hand_open_pose_);
  generate_grasp_pose_stage_->setAngleDelta(M_PI / 12);

  generate_place_pose_stage_ =
    std::make_unique<moveit::task_constructor::stages::GeneratePlacePose>(
      "generate place pose");
  generate_place_pose_stage_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT, { "ik_frame" });
  generate_place_pose_stage_->properties().set("marker_ns", "place_pose");

  // MoveRelative Stages

  approach_object_stage_ =
    std::make_unique<moveit::task_constructor::stages::MoveRelative>(
      "approach object", cartesian_planner_);
  approach_object_stage_->properties().set("marker_ns", "approach_object");
  approach_object_stage_->properties().set("link", hand_frame_);
  approach_object_stage_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT, { "group" });

  retreat_object_stage_ =
    std::make_unique<moveit::task_constructor::stages::MoveRelative>(
      "retreat object", cartesian_planner_);
  retreat_object_stage_->properties().set("marker_ns", "retreat_object");
  retreat_object_stage_->properties().set("link", hand_frame_);
  retreat_object_stage_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT, { "group" });

  lift_object_stage_ =
    std::make_unique<moveit::task_constructor::stages::MoveRelative>(
      "lift object", cartesian_planner_);
  lift_object_stage_->properties().set("marker_ns", "lift_object");
  lift_object_stage_->properties().set("link", hand_frame_);
  lift_object_stage_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT, { "group" });

  drop_object_stage_ =
    std::make_unique<moveit::task_constructor::stages::MoveRelative>(
      "drop object", cartesian_planner_);
  drop_object_stage_->properties().set("marker_ns", "drop_object");
  drop_object_stage_->properties().set("link", hand_frame_);
  drop_object_stage_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT, { "group" });

  // Containers
  serial_container_for_pick_ =
      std::make_unique<moveit::task_constructor::SerialContainer>(
        "serial container for picking");
  task_->properties().exposeTo(serial_container_for_pick_->properties(),
    { "eef", "hand", "group", "ik_frame" });
  serial_container_for_pick_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT,
      { "eef", "hand", "group", "ik_frame" });

  serial_container_for_place_ =
      std::make_unique<moveit::task_constructor::SerialContainer>(
        "serial container for placing");
  task_->properties().exposeTo(serial_container_for_place_->properties(),
    { "eef", "hand", "group" });
  serial_container_for_place_->properties().configureInitFrom(
    moveit::task_constructor::Stage::PARENT,
      { "eef", "hand", "group" });
}


void MTCLibrary::modifyGripperTransform(
  std::string axis_string, double offset) {
  // get direction and axis from string
  char sign = axis_string[0];
  char axis = axis_string[1];

  if (sign == '-') {
    offset *= -1;
  }

  if (axis == 'x') {
    grasp_frame_transform_.translation().x() += offset;
  } else if (axis == 'y') {
    grasp_frame_transform_.translation().y() += offset;
  } else if (axis == 'z') {
    grasp_frame_transform_.translation().z() += offset;
  } else {
    ROS_ERROR_STREAM("Invalid axis_string");
  }
}

geometry_msgs::Vector3Stamped MTCLibrary::getVectorDirection(
  std::string axis_string) {
  // modify direction based on input string
  char sign = axis_string[0];
  char axis = axis_string[1];

  // define default direction
  float direction = 1.0;

  if (sign == '-') {
    direction *= -1;
  }

  // define vector
  geometry_msgs::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;

  if (axis == 'x') {
    vec.vector.x = direction;
  } else if (axis == 'y') {
    vec.vector.y = direction;
  } else if (axis == 'z') {
    vec.vector.z = direction;
  } else {
    ROS_ERROR_STREAM("Invalid axis_string");
  }
  return vec;
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

  // initialize planners and stages
  initializePlannersAndStages();
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

  ros::Duration(0.1).sleep();
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
