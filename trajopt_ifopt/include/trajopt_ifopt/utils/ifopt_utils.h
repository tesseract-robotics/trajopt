/**
 * @file ifopt_utils.h
 * @brief Contains utilities for converting IFOPT types to other types
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
#ifndef TRAJOPT_IFOPT_IFOPT_UTILS_H
#define TRAJOPT_IFOPT_IFOPT_UTILS_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/cost_term.h>
#include <ifopt/problem.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
bool isFinite(double value);

/**
 * @brief Check if bounds are equality where upper and lower are finite and equal
 * @param bounds The bounds to check
 * @return True if equality bounds, otherwise false
 */
bool isBoundsEquality(const ifopt::Bounds& bounds);

/**
 * @brief Check if bounds are [finite, ifopt::inf]
 * @param bounds The bounds to check
 * @return True if bounds are [finite, ifopt::inf], otherwise false
 */
bool isBoundsGreaterFinite(const ifopt::Bounds& bounds);

/**
 * @brief Check if bounds are [-ifopt::inf, finite]
 * @param bounds The bounds to check
 * @return True if bounds are [-ifopt::inf, finite], otherwise false
 */
bool isBoundsSmallerFinite(const ifopt::Bounds& bounds);

/**
 * @brief Check if bounds are [finite, ifopt::inf] or [-ifopt::inf, finite]
 * @param bounds The bounds to check
 * @return True if bounds are [finite, ifopt::inf] or [-ifopt::inf, finite], otherwise false
 */
bool isBoundsInEquality(const ifopt::Bounds& bounds);

/**
 * @brief Check if bounds are [finite, finite]
 * @param bounds The bounds to check
 * @return True if bounds are [finite, finite], otherwise false
 */
bool isBoundsFiniteFinite(const ifopt::Bounds& bounds);

/**
 * @brief Converts a MatrixX2d (e.g. from forward_kinematics->getLimits()) to a vector of ifopt Bounds
 * @param limits MatrixX2d of bounds. Column 0 will be lower bound. Column 1 will be upper bound
 * @return Vector of ifopt::Bounds
 */
std::vector<ifopt::Bounds> toBounds(const Eigen::Ref<const Eigen::MatrixX2d>& limits);

/**
 * @brief Converts 2 VectorXd to a vector of ifopt Bounds
 * @param lower_limits Lower limits
 * @param upper_limits Upper limits
 * @return  Vector of ifopt::Bounds
 */
std::vector<ifopt::Bounds> toBounds(const Eigen::Ref<const Eigen::VectorXd>& lower_limits,
                                    const Eigen::Ref<const Eigen::VectorXd>& upper_limits);

/**
 * @brief Interpolates between two Eigen::VectorXds
 * @param start Start vector. This will be the first vector in the results
 * @param end End vector. This will be the last vector in the results
 * @param steps Length of the returned vector
 * @return A vector of Eigen vectors interpolated from start to end
 */
std::vector<Eigen::VectorXd> interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& end,
                                         Eigen::Index steps);

/**
 * @brief Gets the closes vector to the input given the bounds
 * @param input Input vector
 * @param bounds Bounds on that vector
 * @return Output vector. If input is outside a bound, force it to the boundary
 */
Eigen::VectorXd getClosestValidPoint(const Eigen::Ref<const Eigen::VectorXd>& input,
                                     const std::vector<ifopt::Bounds>& bounds);

/**
 * @brief Calculate errors of the bounds
 * @param input The input values
 * @param bounds The bounds
 * @return The error given the bounds, if within the bounds it will be zero
 */
Eigen::VectorXd calcBoundsErrors(const Eigen::Ref<const Eigen::VectorXd>& input,
                                 const std::vector<ifopt::Bounds>& bounds);

/**
 * @brief The absolute value of the Bounds Errors
 * @param input The input values
 * @param bounds The bounds
 * @return The absolute errors given the bounds, if within the bounds it will be zero
 */
Eigen::VectorXd calcBoundsViolations(const Eigen::Ref<const Eigen::VectorXd>& input,
                                     const std::vector<ifopt::Bounds>& bounds);

ifopt::Problem::VectorXd calcNumericalCostGradient(const double* x, ifopt::Problem& nlp, double epsilon = 1e-8);

ifopt::Problem::Jacobian calcNumericalConstraintGradient(const double* x, ifopt::Problem& nlp, double epsilon = 1e-8);

}  // namespace trajopt_ifopt
#endif
