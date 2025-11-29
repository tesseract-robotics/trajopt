/**
 * @file joint_state_plotter.h
 * @brief A callback to plot a set of JointPosition variables
 *
 * @author Matthew Powelson
 * @date May 18, 2020
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
#ifndef TRAJOPT_SQP_INCLUDE_JOINT_STATE_PLOTTER_H_
#define TRAJOPT_SQP_INCLUDE_JOINT_STATE_PLOTTER_H_

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_visualization/fwd.h>
#include <tesseract_state_solver/fwd.h>
#include <trajopt_ifopt/fwd.h>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/sqp_callback.h>

namespace trajopt_sqp
{
/**
 * @brief SQPCallback that plots joint positions as a Tesseract Trajectory
 */
class JointStatePlottingCallback : public SQPCallback
{
public:
  using Ptr = std::shared_ptr<JointStatePlottingCallback>;
  using ConstPtr = std::shared_ptr<const JointStatePlottingCallback>;

  /**
   * @brief Constructor for the callback
   * @param plotter Plotter used to plot the joint state
   */
  JointStatePlottingCallback(std::shared_ptr<tesseract_visualization::Visualization> plotter,
                             std::unique_ptr<tesseract_scene_graph::StateSolver> state_solver);

  /**
   * @brief Plot the joint_position variables as a tesseract trajectory
   * @param nlp Unused
   */
  void plot(const QPProblem& problem);

  /**
   * @brief Add a variable set to be plotted
   * @param joint_position JointPosition variable to be plotted. They should all be the same size
   */
  void addVariableSet(const std::shared_ptr<const trajopt_ifopt::JointPosition>& joint_position);

  /**
   * @brief Adds multiple variable sets to be plotted
   * @param joint_positions JointPosition variables to be plotted. They should all be the same size
   */
  void addVariableSet(const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& joint_positions);

  bool execute(const QPProblem& problem, const SQPResults& sqp_results) override;

protected:
  std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>> joint_positions_;
  std::shared_ptr<tesseract_visualization::Visualization> plotter_;
  std::unique_ptr<tesseract_scene_graph::StateSolver> state_solver_;
};
}  // namespace trajopt_sqp

#endif
