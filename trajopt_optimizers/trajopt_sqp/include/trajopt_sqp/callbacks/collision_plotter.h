/**
 * @file collision_plotter.h
 * @brief Plots collision results
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug This has not been implemented
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
#ifndef TRAJOPT_SQP_INCLUDE_COLLISION_PLOTTING_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_COLLISION_PLOTTING_CALLBACK_H_

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_visualization/fwd.h>
#include <trajopt_ifopt/trajopt/fwd.h>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/sqp_callback.h>

namespace trajopt_sqp
{
class CollisionPlottingCallback : public SQPCallback
{
public:
  using Ptr = std::shared_ptr<CollisionPlottingCallback>;
  using ConstPtr = std::shared_ptr<const CollisionPlottingCallback>;

  CollisionPlottingCallback(std::shared_ptr<tesseract_visualization::Visualization> plotter);

  void plot(const QPProblem& problem);

  void addConstraintSet(const std::shared_ptr<const trajopt_ifopt::DiscreteCollisionConstraint>& collision_constraint);

  void addConstraintSet(
      const std::vector<std::shared_ptr<const trajopt_ifopt::DiscreteCollisionConstraint>>& collision_constraints);

  bool execute(const QPProblem& problem, const SQPResults& sqp_results) override;

protected:
  std::vector<std::shared_ptr<const trajopt_ifopt::DiscreteCollisionConstraint>> collision_constraints_;
  std::shared_ptr<tesseract_visualization::Visualization> plotter_;
};
}  // namespace trajopt_sqp

#endif
