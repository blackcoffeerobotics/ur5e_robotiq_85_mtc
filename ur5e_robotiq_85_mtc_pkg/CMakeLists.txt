cmake_minimum_required(VERSION 3.0.2)
project(ur5e_robotiq_85_mtc_pkg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rosparam_shortcuts
  ur5e_robotiq_85_moveit_config
  ur_description
  ur_gazebo
  robotiq_2f_85
  roboticsgroup_upatras_gazebo_plugins
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mtc_lib pick_and_place_lib
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

add_library(spawn_helper_lib src/spawn_helper.cpp)
add_library(mtc_lib src/mtc.cpp)

add_library(pick_and_place_lib src/pick_and_place.cpp)
target_link_libraries(pick_and_place_lib mtc_lib ${catkin_LIBRARIES})


add_executable(pick_and_place_node
  src/exec_pick_and_place.cpp
)

target_link_libraries(pick_and_place_node
  ${catkin_LIBRARIES}
  spawn_helper_lib
  mtc_lib
  pick_and_place_lib
)

