// Copyright (c) 2023 Black Coffee Robotics

/**
 * Program to test the Pick and Place function of the UR5e arm
   using MoveIt! Task Constructor.
 */

#include <ros/ros.h>
#include <ur5e_robotiq_85_mtc_pkg/spawn_helper.h>
#include <ur5e_robotiq_85_mtc_pkg/mtc.h>
#include <ur5e_robotiq_85_mtc_pkg/pick_and_place.h>



std::map<std::string,
  std::tuple<std::string, std::vector<double>, std::vector<double>>>
  populatePlanningSceneObjects(XmlRpc::XmlRpcValue planning_scene_objects) {
  // define map
  std::map<std::string,
    std::tuple<std::string, std::vector<double>, std::vector<double>>>
      planning_scene_objects_map;

  for (int i = 0; i < planning_scene_objects.size(); i++) {
    // assert for struct type
    ROS_ASSERT(planning_scene_objects[i].getType() ==
      XmlRpc::XmlRpcValue::TypeStruct);

    // get object name
    // assert for string type
    ROS_ASSERT(planning_scene_objects[i]["name"].getType() ==
      XmlRpc::XmlRpcValue::TypeString);
    std::string object_name = planning_scene_objects[i]["name"];

    // get object link state
    // assert for string type
    ROS_ASSERT(planning_scene_objects[i]["link_state"].getType() ==
      XmlRpc::XmlRpcValue::TypeString);
    std::string object_link_state = planning_scene_objects[i]["link_state"];

    // get object dimensions
    // assert for array type
    ROS_ASSERT(planning_scene_objects[i]["dimensions"].getType() ==
      XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> object_dimensions;
    XmlRpc::XmlRpcValue objec_dimensions_xml =
      planning_scene_objects[i]["dimensions"];
    for (int j = 0; j < objec_dimensions_xml.size(); j++) {
      object_dimensions.push_back(objec_dimensions_xml[j]);
    }

    // get origin compensation
    // assert for array type
    ROS_ASSERT(planning_scene_objects[i]["origin_compensation"].getType() ==
      XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> origin_compensation;
    XmlRpc::XmlRpcValue origin_compensation_xml =
      planning_scene_objects[i]["origin_compensation"];
    for (int k = 0; k < origin_compensation_xml.size(); k++) {
      origin_compensation.push_back(origin_compensation_xml[k]);
    }
    // save object to map
    planning_scene_objects_map[object_name] =
      std::make_tuple(
        object_link_state, object_dimensions, origin_compensation);
  }

  return planning_scene_objects_map;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string base_link_state;
  std::string object_name;
  std::vector<double> place_offset;
  std::map<std::string,
    std::tuple<std::string, std::vector<double>, std::vector<double>>>
      planning_scene_objects;

  // ...... Loading parameters from the parameter server ........

  ROS_INFO_STREAM(bash_colours.at("green") +
    "Loading mtc parameters" + bash_colours.at("reset"));

  ros::NodeHandle pnh("~");
  std::string param_ns = "pick_and_place/";
  std::size_t errors = 0;

  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "base_link_state", base_link_state);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "object_name", object_name);
  errors += !rosparam_shortcuts::get(
    "", pnh, param_ns + "place_offset", place_offset);
  rosparam_shortcuts::shutdownIfError("", errors);


  XmlRpc::XmlRpcValue planning_scene_objects_xml;
  pnh.getParam(param_ns + "planning_scene_objects", planning_scene_objects_xml);
  ROS_ASSERT(planning_scene_objects_xml.getType() ==
    XmlRpc::XmlRpcValue::TypeArray);

  planning_scene_objects =
    populatePlanningSceneObjects(planning_scene_objects_xml);

  // .............. end of loading parameters ............

  geometry_msgs::Pose object_pose;
  geometry_msgs::Pose place_pose;
  std::string planning_frame;
  bool success;

  // initialize the spawnhelper class
  SpawnHelper sh(nh);

  // get object pose
  object_pose = sh.getObjectPose(
    base_link_state, std::get<0>(planning_scene_objects.at(object_name)),
      std::get<2>(planning_scene_objects.at(object_name)));

  // define place pose
  place_pose = object_pose;
  place_pose.position.x += place_offset[0];
  place_pose.position.y += place_offset[1];
  place_pose.position.z += place_offset[2];

  // define first task for holding the coke can
  PickAndPlace pp("pick_and_place " + object_name, nh,
    object_name, object_pose, place_pose);

  // get planning frame
  planning_frame = pp.getPlanningFrame();

  // spawn all objects
  for (int i = 0; i < planning_scene_objects.size(); i++) {
    // get object name
    std::string obj_name = planning_scene_objects_xml[i]["name"];

    // get object dimensions
    std::vector<double> obj_dimensions =
      std::get<1>(planning_scene_objects.at(obj_name));

    // get object pose
    geometry_msgs::Pose obj_pose = sh.getObjectPose(
      base_link_state, std::get<0>(planning_scene_objects.at(obj_name)),
        std::get<2>(planning_scene_objects.at(obj_name)));

    // spawn object
    pp.spawnObject(obj_name, obj_pose, obj_dimensions, planning_frame);
  }

  ROS_INFO_STREAM(bash_colours.at("green") +
    "Starting Pick and Place" + bash_colours.at("reset"));

  // execute pipeline
  success = pp.executePipeline();

  if (success) {
    ROS_INFO_STREAM(bash_colours.at("green") +
      "Pick and Place Successful" + bash_colours.at("reset"));
  } else {
    ROS_INFO_STREAM(bash_colours.at("red") +
      "Pick and Place Failed" + bash_colours.at("reset"));
  }

  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}

