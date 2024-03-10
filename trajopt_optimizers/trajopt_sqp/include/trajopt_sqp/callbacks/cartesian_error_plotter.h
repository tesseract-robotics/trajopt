/**
 * @file cartesian_error_plotter.h
 * @brief A callback to plot cartesian constraints
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
#ifndef TRAJOPT_SQP_INCLUDE_CARTESIAN_ERROR_PLOTTING_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_CARTESIAN_ERROR_PLOTTING_CALLBACK_H_

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_visualization/fwd.h>
#include <trajopt_ifopt/fwd.h>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/sqp_callback.h>

namespace trajopt_sqp
{
/**
 * @brief SQPCallback the error of a CartPosConstraint. It plots an axis at the target and the current position with an
 * arrow between them
 */
class CartesianErrorPlottingCallback : public SQPCallback
{
public:
  using Ptr = std::shared_ptr<CartesianErrorPlottingCallback>;
  using ConstPtr = std::shared_ptr<const CartesianErrorPlottingCallback>;

  /**
   * @brief Callback constructor
   * @param plotter Plotter used to plot the error
   */
  CartesianErrorPlottingCallback(std::shared_ptr<tesseract_visualization::Visualization> plotter);

  /**
   * @brief Plots the error
   * @param nlp Unused
   */
  void plot(const QPProblem& problem);

  /**
   * @brief addConstraintSet Adds a constraint set to be plotted
   * @param cart_position_cnt Constraint to be plotted
   */
  void addConstraintSet(const std::shared_ptr<const trajopt_ifopt::CartPosConstraint>& cart_position_cnt);

  /**
   * @brief addConstraintSet Adds a vector of constraint sets to be plotted
   * @param cart_position_cnts Vector of constraints to be plotted
   */
  void addConstraintSet(const std::vector<std::shared_ptr<const trajopt_ifopt::CartPosConstraint>>& cart_position_cnts);

  bool execute(const QPProblem& problem, const SQPResults& sqp_results) override;

protected:
  std::vector<std::shared_ptr<const trajopt_ifopt::CartPosConstraint>> cart_position_cnts_;
  std::shared_ptr<tesseract_visualization::Visualization> plotter_;
};
}  // namespace trajopt_sqp

#endif
