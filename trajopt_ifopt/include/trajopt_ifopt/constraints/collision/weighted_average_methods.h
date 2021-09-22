/**
 * @file weighted_average_methods.h
 * @brief Contains weighted average methods for combining collision results
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
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
#ifndef TRAJOPT_IFOPT_COLLISION_WEIGHTED_AVERAGE_METHODS_H
#define TRAJOPT_IFOPT_COLLISION_WEIGHTED_AVERAGE_METHODS_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision/collision_types.h>

namespace trajopt_ifopt
{
Eigen::VectorXd getWeightedAvgGradientT0(const GradientResultsSet& grad_results_set,
                                         double max_error_with_buffer,
                                         Eigen::Index size);
Eigen::VectorXd getWeightedAvgGradientT1(const GradientResultsSet& grad_results_set,
                                         double max_error_with_buffer,
                                         Eigen::Index size);
}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_COLLISION_WEIGHTED_AVERAGE_METHODS_H
