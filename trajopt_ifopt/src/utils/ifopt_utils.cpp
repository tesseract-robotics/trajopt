/**
 * @file ifopt_utils.cpp
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

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/composite.h>
#include <ifopt/constraint_set.h>
#include <ifopt/problem.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/utils/ifopt_utils.h>

namespace trajopt_ifopt
{
bool isFinite(double value) { return std::isfinite(value) && (value < ifopt::inf) && (value > -ifopt::inf); }

bool isBoundsEquality(const ifopt::Bounds& bounds)
{
  return isFinite(bounds.lower_) && isFinite(bounds.upper_) && (std::abs(bounds.upper_ - bounds.lower_) < 1e-8);
}

bool isBoundsGreaterFinite(const ifopt::Bounds& bounds)
{
  return (isFinite(bounds.lower_) && !isFinite(bounds.upper_));
}

bool isBoundsSmallerFinite(const ifopt::Bounds& bounds)
{
  return (std::isfinite(bounds.upper_) && !isFinite(bounds.lower_));
}

bool isBoundsInEquality(const ifopt::Bounds& bounds)
{
  return isBoundsGreaterFinite(bounds) || isBoundsSmallerFinite(bounds);
}

bool isBoundsFiniteFinite(const ifopt::Bounds& bounds) { return (isFinite(bounds.lower_) && isFinite(bounds.upper_)); }

std::vector<ifopt::Bounds> toBounds(const Eigen::Ref<const Eigen::MatrixX2d>& limits)
{
  std::vector<ifopt::Bounds> bounds;
  bounds.reserve(static_cast<std::size_t>(limits.rows()));
  for (Eigen::Index i = 0; i < limits.rows(); i++)
    bounds.emplace_back(limits(i, 0), limits(i, 1));
  return bounds;
}

std::vector<ifopt::Bounds> toBounds(const Eigen::Ref<const Eigen::VectorXd>& lower_limits,
                                    const Eigen::Ref<const Eigen::VectorXd>& upper_limits)
{
  assert(lower_limits.size() == upper_limits.size());  // NOLINT
  Eigen::MatrixX2d limits(lower_limits.rows(), 2);
  limits.col(0) = lower_limits;
  limits.col(1) = upper_limits;
  return toBounds(limits);
}

std::vector<Eigen::VectorXd> interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& end,
                                         Eigen::Index steps)
{
  assert(start.size() == end.size());  // NOLINT
  const Eigen::VectorXd delta = (end - start) / static_cast<double>(steps - 1);
  Eigen::VectorXd running = start;
  std::vector<Eigen::VectorXd> results;
  for (Eigen::Index i = 0; i < steps; i++)
  {
    results.push_back(running);
    running += delta;
  }
  return results;
}

Eigen::VectorXd getClosestValidPoint(const Eigen::Ref<const Eigen::VectorXd>& input,
                                     const std::vector<ifopt::Bounds>& bounds)
{
  // Convert Bounds to VectorXds
  Eigen::VectorXd bound_lower(static_cast<Eigen::Index>(bounds.size()));
  Eigen::VectorXd bound_upper(static_cast<Eigen::Index>(bounds.size()));
  for (std::size_t i = 0; i < bounds.size(); i++)
  {
    bound_lower[static_cast<Eigen::Index>(i)] = bounds[i].lower_;
    bound_upper[static_cast<Eigen::Index>(i)] = bounds[i].upper_;
  }

  // If input is outside a bound, force it to the boundary
  return input.cwiseMax(bound_lower).cwiseMin(bound_upper);
}

Eigen::VectorXd calcBoundsErrors(const Eigen::Ref<const Eigen::VectorXd>& input,
                                 const std::vector<ifopt::Bounds>& bounds)
{
  assert(input.rows() == static_cast<Eigen::Index>(bounds.size()));  // NOLINT

  // Convert constraint bounds to VectorXd
  Eigen::ArrayXd bound_lower(input.rows());
  Eigen::ArrayXd bound_upper(input.rows());
  for (std::size_t i = 0; i < bounds.size(); i++)
  {
    bound_lower[static_cast<Eigen::Index>(i)] = bounds[i].lower_;
    bound_upper[static_cast<Eigen::Index>(i)] = bounds[i].upper_;
  }

  // Values will be negative if they violate the constrain
  const Eigen::ArrayXd zero = Eigen::ArrayXd::Zero(input.rows());
  const Eigen::ArrayXd dist_from_lower = (input.array() - bound_lower).min(zero);
  const Eigen::ArrayXd dist_from_upper = (input.array() - bound_upper).max(zero);
  const Eigen::ArrayXd worst_error =
      (dist_from_upper.abs() > dist_from_lower.abs()).select(dist_from_upper, dist_from_lower);

  return worst_error;
}

Eigen::VectorXd calcBoundsViolations(const Eigen::Ref<const Eigen::VectorXd>& input,
                                     const std::vector<ifopt::Bounds>& bounds)
{
  return calcBoundsErrors(input, bounds).cwiseAbs();  // NOLINT
}

ifopt::VectorXd calcNumericalCostGradient(const double* x, ifopt::Problem& nlp, double epsilon)
{
  auto cache_vars = nlp.GetVariableValues();

  const int n = nlp.GetNumberOfOptimizationVariables();
  ifopt::Problem::Jacobian jac(1, n);
  if (nlp.HasCostTerms())
  {
    const double step_size = epsilon;

    // calculate forward difference by disturbing each optimization variable
    const double g = nlp.EvaluateCostFunction(x);
    std::vector<double> x_new(x, x + n);
    for (int i = 0; i < n; ++i)
    {
      x_new[static_cast<std::size_t>(i)] += step_size;  // disturb
      const double g_new = nlp.EvaluateCostFunction(x_new.data());
      jac.coeffRef(0, i) = (g_new - g) / step_size;
      x_new[static_cast<std::size_t>(i)] = x[i];  // reset for next iteration
    }
  }

  // Set problem values back to the original values.
  nlp.SetVariables(cache_vars.data());

  return jac.row(0).transpose();
}

ifopt::Jacobian calcNumericalConstraintGradient(const double* x, ifopt::Problem& nlp, double epsilon)
{
  auto cache_vars = nlp.GetVariableValues();

  const int n = nlp.GetNumberOfOptimizationVariables();
  const int m = nlp.GetConstraints().GetRows();
  ifopt::Problem::Jacobian jac(m, n);
  jac.reserve(static_cast<Eigen::Index>(m) * static_cast<Eigen::Index>(n));

  if (nlp.GetNumberOfConstraints() > 0)
  {
    const double step_size = epsilon;

    // calculate forward difference by disturbing each optimization variable
    const ifopt::Problem::VectorXd g = nlp.EvaluateConstraints(x);
    std::vector<double> x_new(x, x + n);
    for (int i = 0; i < n; ++i)
    {
      x_new[static_cast<std::size_t>(i)] += step_size;  // disturb
      const ifopt::Problem::VectorXd g_new = nlp.EvaluateConstraints(x_new.data());
      ifopt::Problem::VectorXd delta_g = (g_new - g) / step_size;

      for (int j = 0; j < m; ++j)
        jac.coeffRef(j, i) = delta_g(j);

      x_new[static_cast<std::size_t>(i)] = x[i];  // reset for next iteration
    }
  }

  // Set problem values back to the original values.
  nlp.SetVariables(cache_vars.data());

  return jac;
}

ifopt::Jacobian calcNumericalConstraintGradient(ifopt::Component& variables,
                                                ifopt::ConstraintSet& constraint_set,
                                                double epsilon)
{
  Eigen::VectorXd x = variables.GetValues();

  const int n = variables.GetRows();
  const int m = constraint_set.GetRows();
  ifopt::Problem::Jacobian jac(m, n);
  jac.reserve(static_cast<Eigen::Index>(m) * static_cast<Eigen::Index>(n));

  if (!constraint_set.GetBounds().empty())
  {
    // calculate forward difference by disturbing each optimization variable
    const ifopt::Problem::VectorXd g = constraint_set.GetValues();
    Eigen::VectorXd x_new = x;
    for (Eigen::Index i = 0; i < n; ++i)
    {
      x_new(i) = x(i) + epsilon;  // disturb
      variables.SetVariables(x_new);
      const ifopt::Problem::VectorXd g_new = constraint_set.GetValues();
      ifopt::Problem::VectorXd delta_g = (g_new - g) / epsilon;

      for (int j = 0; j < m; ++j)
        jac.coeffRef(j, i) = delta_g(j);

      x_new(i) = x(i);  // reset for next iteration
    }
  }

  // Set problem values back to the original values.
  variables.SetVariables(x);

  return jac;
}
}  // namespace trajopt_ifopt
