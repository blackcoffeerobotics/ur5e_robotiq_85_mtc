// Copyright (c) 2023 Black Coffee Robotics

/**
 * Source file for the Spawn Helper class
 */

#include <ur5e_robotiq_85_mtc_pkg/spawn_helper.h>


SpawnHelper::SpawnHelper(const ros::NodeHandle& nh) : nh_(nh) {
}


geometry_msgs::Pose SpawnHelper::getBaseFramePose(std::string link_name) {
  // listen to the /gazebo/link_states topic
  gazebo_msgs::LinkStatesConstPtr link_states =
    ros::topic::waitForMessage<gazebo_msgs::LinkStates>(
      "/gazebo/link_states", nh_);

  // find the index of the base frame
  auto it = std::find(link_states->name.begin(), link_states->name.end(),
    link_name);

  // get the pose of the base frame
  return link_states->pose[std::distance(link_states->name.begin(), it)];
}

geometry_msgs::Pose SpawnHelper::getObjectPose(
    std::string base_link_state, std::string object_name,
      std::vector<double> origin_compensation) {
  // listen to the /gazebo/link_states topic
  gazebo_msgs::LinkStatesConstPtr link_states =
    ros::topic::waitForMessage<gazebo_msgs::LinkStates>(
      "/gazebo/link_states", nh_);

  geometry_msgs::Pose object_pose;
  geometry_msgs::Pose base_frame_pose = getBaseFramePose(base_link_state);

  auto it = std::find(
    link_states->name.begin(), link_states->name.end(), object_name);

  object_pose.position.x = link_states->pose[std::distance(
    link_states->name.begin(), it)].position.x;
  object_pose.position.y = link_states->pose[std::distance(
    link_states->name.begin(), it)].position.y;
  object_pose.position.z = link_states->pose[std::distance(
    link_states->name.begin(), it)].position.z;
  object_pose.orientation.x = link_states->pose[std::distance(
    link_states->name.begin(), it)].orientation.x;
  object_pose.orientation.y = link_states->pose[std::distance(
    link_states->name.begin(), it)].orientation.y;
  object_pose.orientation.z = link_states->pose[std::distance(
    link_states->name.begin(), it)].orientation.z;
  object_pose.orientation.w = link_states->pose[std::distance(
    link_states->name.begin(), it)].orientation.w;

  // compensate for the origin of the model
  object_pose.position.x += origin_compensation[0];
  object_pose.position.y += origin_compensation[1];
  object_pose.position.z += origin_compensation[2];

  // add rpy values from origin compensation
  // check size of origin_compensation
  if (origin_compensation.size() == 6) {
    tf2::Quaternion quat_compensation;
    quat_compensation.setRPY(origin_compensation[3], origin_compensation[4],
      origin_compensation[5]);
    object_pose.orientation = tf2::toMsg(quat_compensation);
  }
  // shift origin to base frame pose
  object_pose.position.x -= base_frame_pose.position.x;
  object_pose.position.y -= base_frame_pose.position.y;
  object_pose.position.z -= base_frame_pose.position.z;

  // shift origin quaternion to base frame pose
  tf2::Quaternion model_quat, base_quat;
  tf2::fromMsg(object_pose.orientation, model_quat);
  tf2::fromMsg(base_frame_pose.orientation, base_quat);
  model_quat = base_quat.inverse() * model_quat;
  object_pose.orientation = tf2::toMsg(model_quat);

  return object_pose;
}
