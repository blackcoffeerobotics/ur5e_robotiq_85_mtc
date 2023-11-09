#!/bin/bash

# Move Scripts
cp -r scripts/generate_custom_pose.h moveit_task_constructor/core/include/moveit/task_constructor/stages
cp -r scripts/generate_custom_pose.cpp moveit_task_constructor/core/src/stages
cp -r scripts/CMakeLists.txt moveit_task_constructor/core/src/stages/CMakeLists.txt
sed -i 's/1e-4/1e-2/g' moveit_task_constructor/core/src/stages/connect.cpp
