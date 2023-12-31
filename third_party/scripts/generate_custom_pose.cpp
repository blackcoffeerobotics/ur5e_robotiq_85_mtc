/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke, Michael Goerner */

#include <moveit/task_constructor/stages/generate_custom_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#include <iterator>

namespace moveit {
namespace task_constructor {
namespace stages {

GenerateCustomPose::GenerateCustomPose(
  const std::string& name) : MonitoringGenerator(name) {
  auto& p = properties();
  p.declare<std::vector<geometry_msgs::PoseStamped>>(
    "poses_array", "target poses array to pass on in spawned states");
}

void GenerateCustomPose::reset() {
  upstream_solutions_.clear();
  MonitoringGenerator::reset();
}

void GenerateCustomPose::onNewSolution(const SolutionBase& s) {
  /* It's safe to store a pointer to this solution,
    as the generating stage stores it
 */
  upstream_solutions_.push(&s);
}

bool GenerateCustomPose::canCompute() const {
  return !upstream_solutions_.empty();
}

void GenerateCustomPose::compute() {
  if (upstream_solutions_.empty())
    return;

  planning_scene::PlanningScenePtr scene =
    upstream_solutions_.pop()->end()->scene()->diff();

  std::vector<geometry_msgs::PoseStamped> customPoses =
    properties().get<std::vector<geometry_msgs::PoseStamped>>("poses_array");

  int i = 0;

  while (i < customPoses.size()) {
    if (customPoses[i].header.frame_id.empty()) {
        customPoses[i].header.frame_id = scene->getPlanningFrame();

    } else if (!scene->knowsFrameTransform(customPoses[i].header.frame_id)) {
        ROS_WARN_NAMED("GeneratePose", "Unknown frame: '%s'",
          customPoses[i].header.frame_id.c_str());
        return;
    }

    InterfaceState state(scene);
    state.properties().set("target_pose", customPoses[i]);

    SubTrajectory trajectory;
    trajectory.setCost(0.0);

    rviz_marker_tools::appendFrame(
      trajectory.markers(), customPoses[i], 0.1, "pose frame");

    spawn(std::move(state), std::move(trajectory));
    i++;
}
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
