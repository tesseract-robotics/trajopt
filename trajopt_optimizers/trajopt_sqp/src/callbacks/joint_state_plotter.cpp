/**
 * @file joint_state_plotter.cpp
 * @brief A callback to plot a set of JointPosition variables
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <trajopt_sqp/callbacks/joint_state_plotter.h>

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>

#include <tesseract_common/joint_state.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_state_solver/state_solver.h>

using namespace trajopt_sqp;

JointStatePlottingCallback::JointStatePlottingCallback(std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                                       std::unique_ptr<tesseract_scene_graph::StateSolver> state_solver)
  : plotter_(std::move(plotter)), state_solver_(std::move(state_solver))
{
}

void JointStatePlottingCallback::plot(const QPProblem& /*problem*/)
{
  const tesseract_common::JointTrajectory trajectory = trajopt_ifopt::toJointTrajectory(joint_positions_);

  if (plotter_)
    plotter_->plotTrajectory(trajectory, *state_solver_);
}

void JointStatePlottingCallback::addVariableSet(
    const std::shared_ptr<const trajopt_ifopt::JointPosition>& joint_position)
{
  joint_positions_.push_back(joint_position);
}

void JointStatePlottingCallback::addVariableSet(
    const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition> >& joint_positions)
{
  for (const auto& cnt : joint_positions)
    joint_positions_.push_back(cnt);
}
bool JointStatePlottingCallback::execute(const QPProblem& problem, const SQPResults& /*sqp_results*/)
{
  plot(problem);
  return true;
}
