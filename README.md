# ur5e_robotiq_85_mtc

## Overview

This package contains a custom [MoveIt! configuration package](ur5e_robotiq_85_moveit_config) for the UR5e arm equipped with a Robotiq 85 gripper.
<br/>It also contains a [MoveIt! Task Constructor (MTC) package](ur5e_robotiq_85_mtc_pkg) that can be used to generate plans for the robot to perform a pick and place task. All revelant parameters for MTC are stored in the [config folder](ur5e_robotiq_85_mtc_pkg/config/mtc/).

## Tutorial

* Install all the dependencies for the `rosinstall` tool:

	```bash
  sudo apt-get install \
    python3-vcstools \
    python3-rosinstall
	```

* Install all the external dependencies (These include MoveIt Task Constructor, and our fork of MoveIt! for orientation constraints):

	```bash
  cd third_party/
  rosinstall . /opt/ros/$ROS_DISTRO ur5e_robotiq_85_mtc.rosinstall
  ./modify_mtc.sh
	```

* Install all the dependencies by running the following command in the root of your workspace:

	```bash
  rosdep install --from-paths src --ignore-src -r -y
	```

* Build the workspace and source the setup file:

	```bash
  catkin build
  source devel/setup.bash
	```

* Launch Gazebo along with support for MoveIt! and RViz:

	```bash
  roslaunch ur5e_robotiq_85_mtc_pkg setup.launch start_rviz:=true
	```

* Finally, run the pick and place pipeline:

	```bash
  roslaunch ur5e_robotiq_85_mtc_pkg pick_and_place.launch
	```

	https://github.com/blackcoffeerobotics/ur5e_robotiq_85_mtc/assets/45683974/5c3c0a33-740e-40fa-b4b7-5060289c6a3f



## Our Contributions to the MoveIt! Fork

### 1. Backporting the Joint Flip BugFix from MoveIt!2:

Following the amazing work of [Jeroen De Maeyer](https://github.com/JeroenDM) during his GSoC 2020 project for [MoveIt! Constrained Planning](https://gist.github.com/JeroenDM/426e3a7e083049295bbcb660c9a98e63), most of the crucial fixes had been ported over to MoveIt2 instead of the former.<br/>

As a result, orientation constraints in MoveIt! do not work by default because of the specified [bug](https://github.com/ros-planning/moveit/pull/2273).<br/>

We ported the relevant features of his [PR to MoveIt!2](https://github.com/ros-planning/moveit2/pull/347) into MoveIt! in the [following PR](https://github.com/ros-planning/moveit/pull/3415) to the repository.<br/>

Therefore, to ensure that the constraints specified in the Connect Stage carry out the motion plan to the SerialContainer, use the [main](https://github.com/leander-dsouza/moveit/tree/master) branch of our custom fork of MoveIt!.

### 2. Caching Constraints now takes Parameterization into account

If your environment remains static between runs, it might be a good idea to save aspects of the planning scene that do not vary. This helps us in creating a database of pre-computed constrained states. As a result, the planning time during the constrained motion is greatly reduced upon execution.<br/>

The format for saving constraints mainly involves setting the frame of reference, target orientation, tolerance, and [parameterization](https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/OrientationConstraint.html). The parameterization field allows us to specify the target orientation using Euler angles or rotation vectors.<br/>

The default way you stored constraints earlier did not consider this field while generating the database. This is paramount as specifying constraints using Euler angles is not ideal in certain orientations, in which the end effector is stuck in a Gimbal lock. This is easily overcome by using rotation vectors instead of Euler angles in specifying target orientation.<br/>

Therefore, before you run your setup, store your constraints in your database using rotation vector representation instead and use the [improve-constraint-manifold](https://github.com/leander-dsouza/moveit/tree/improve-constraint-manifold) branch of our custom fork of MoveIt!.

###### 💾 EOF
