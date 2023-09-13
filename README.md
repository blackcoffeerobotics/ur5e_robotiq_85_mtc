# ur5e_robotiq_85_mtc

## Overview

This package contains a custom [MoveIt! configuration package](ur5e_robotiq_85_moveit_config) for the UR5e arm equipped with a Robotiq 85 gripper.
<br/>It also contains a [MoveIt! Task Constructor (MTC) package](ur5e_robotiq_85_mtc_pkg) that can be used to generate plans for the robot to perform a pick and place task. All revelant parameters for MTC are stored in the [config folder](ur5e_robotiq_85_mtc_pkg/config/mtc/).

## Tutorial

* Clone our fork of MoveIt! for orientation constraints:

	```bash
	git clone git@github.com:leander-dsouza/moveit.git
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

	<p align="center">
		<img src="/uploads/e7a40c1bee9e0a8fbec942f6066b1209/pick_and_place_pipeline.mp4" width="900" height="500">
	</p>

