/**
 * @file ifopt_utils.cpp
 * @brief Contains utilities for converting IFOPT types to other types
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

#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/composite.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

namespace trajopt_ifopt
{
std::vector<Bounds> toBounds(const Eigen::Ref<const Eigen::MatrixX2d>& limits)
{
  std::vector<Bounds> bounds;
  bounds.reserve(static_cast<std::size_t>(limits.rows()));
  for (Eigen::Index i = 0; i < limits.rows(); i++)
    bounds.emplace_back(limits(i, 0), limits(i, 1));
  return bounds;
}

std::vector<Bounds> toBounds(const Eigen::Ref<const Eigen::VectorXd>& lower_limits,
                             const Eigen::Ref<const Eigen::VectorXd>& upper_limits)
{
  assert(lower_limits.size() == upper_limits.size());  // NOLINT

  const auto n = lower_limits.size();
  std::vector<Bounds> bounds;
  bounds.reserve(static_cast<std::size_t>(n));

  for (Eigen::Index i = 0; i < n; ++i)
    bounds.emplace_back(lower_limits[i], upper_limits[i]);

  return bounds;
}

std::vector<Eigen::VectorXd> interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& end,
                                         Eigen::Index steps)
{
  assert(start.size() == end.size());  // NOLINT
  assert(steps >= 2);                  // NOLINT: avoid division by zero

  const Eigen::VectorXd delta = (end - start) / static_cast<double>(steps - 1);
  Eigen::VectorXd running = start;

  std::vector<Eigen::VectorXd> results;
  results.reserve(static_cast<std::size_t>(steps));

  for (Eigen::Index i = 0; i < steps; ++i)
  {
    results.push_back(running);
    running += delta;
  }

  return results;
}

Eigen::VectorXd getClosestValidPoint(const Eigen::Ref<const Eigen::VectorXd>& input, const std::vector<Bounds>& bounds)
{
  assert(input.size() == static_cast<Eigen::Index>(bounds.size()));  // NOLINT

  Eigen::VectorXd lower(input.size());
  Eigen::VectorXd upper(input.size());

  for (Eigen::Index i = 0; i < input.size(); ++i)
  {
    lower[i] = bounds[static_cast<std::size_t>(i)].getLower();
    upper[i] = bounds[static_cast<std::size_t>(i)].getUpper();
  }

  return input.cwiseMax(lower).cwiseMin(upper);
}

void calcBoundsErrors(Eigen::Ref<Eigen::VectorXd> out,
                      const Eigen::Ref<const Eigen::VectorXd>& input,
                      const std::vector<Bounds>& bounds)
{
  assert(input.size() == static_cast<Eigen::Index>(bounds.size()));  // NOLINT
  assert(out.size() == input.size());                                // NOLINT

  for (Eigen::Index i = 0; i < input.size(); ++i)
  {
    const auto& b = bounds[static_cast<std::size_t>(i)];
    const double x = input[i];
    const double lb = b.getLower();
    const double ub = b.getUpper();

    // Signed "worst" error: negative if below lower, positive if above upper, else 0.
    // This matches your vectorized logic.
    if (x < lb)
      out[i] = x - lb;  // negative
    else if (x > ub)
      out[i] = x - ub;  // positive
    else
      out[i] = 0.0;
  }
}

void calcBoundsViolations(Eigen::Ref<Eigen::VectorXd> out,
                          const Eigen::Ref<const Eigen::VectorXd>& input,
                          const std::vector<Bounds>& bounds)
{
  assert(input.size() == static_cast<Eigen::Index>(bounds.size()));  // NOLINT
  assert(out.size() == input.size());                                // NOLINT

  for (Eigen::Index i = 0; i < input.size(); ++i)
  {
    const auto& b = bounds[static_cast<std::size_t>(i)];
    const double x = input[i];
    const double lb = b.getLower();
    const double ub = b.getUpper();

    // Signed "worst" error: negative if below lower, positive if above upper, else 0.
    // This matches your vectorized logic.
    if (x < lb)
      out[i] = std::abs(x - lb);  // negative
    else if (x > ub)
      out[i] = std::abs(x - ub);  // positive
    else
      out[i] = 0.0;
  }
}

Eigen::VectorXd calcNumericalCostGradient(const double* x, Problem& nlp, double epsilon)
{
  auto cache_vars = nlp.getVariableValues();

  const int n = nlp.getNumberOfOptimizationVariables();
  Jacobian jac(1, n);

  if (nlp.hasCostTerms())
  {
    const double g = nlp.evaluateCostFunction(x);
    std::vector<double> x_new(x, x + n);

    for (int i = 0; i < n; ++i)
    {
      x_new[static_cast<std::size_t>(i)] += epsilon;  // disturb
      const double g_new = nlp.evaluateCostFunction(x_new.data());
      jac.coeffRef(0, i) = (g_new - g) / epsilon;
      x_new[static_cast<std::size_t>(i)] = x[i];  // reset
    }
  }
  else
  {
    jac.setZero();
  }

  nlp.setVariables(cache_vars.data());
  return jac.row(0).transpose();
}

Jacobian calcNumericalConstraintGradient(const double* x, Problem& nlp, double epsilon)
{
  auto cache_vars = nlp.getVariableValues();

  const int n = nlp.getNumberOfOptimizationVariables();
  const int m = nlp.getConstraints().getRows();
  Jacobian jac(m, n);
  jac.reserve(static_cast<Eigen::Index>(m) * static_cast<Eigen::Index>(n));

  if (nlp.getNumberOfConstraints() > 0 && n > 0)
  {
    const Eigen::VectorXd g = nlp.evaluateConstraints(x);
    std::vector<double> x_new(x, x + n);
    Eigen::VectorXd delta_g(m);

    for (int i = 0; i < n; ++i)
    {
      x_new[static_cast<std::size_t>(i)] += epsilon;
      const Eigen::VectorXd g_new = nlp.evaluateConstraints(x_new.data());
      delta_g = (g_new - g) / epsilon;

      for (int j = 0; j < m; ++j)
        jac.coeffRef(j, i) = delta_g[j];

      x_new[static_cast<std::size_t>(i)] = x[i];
    }
  }

  nlp.setVariables(cache_vars.data());
  return jac;
}

Jacobian calcNumericalConstraintGradient(Variables& variables, ConstraintSet& constraint_set, double epsilon)
{
  const int n = variables.getRows();
  const int m = constraint_set.getRows();

  Jacobian jac(m, n);
  jac.reserve(static_cast<Eigen::Index>(m) * static_cast<Eigen::Index>(n));

  // Nothing to do if there are no constraints, no variables, or no bounds
  if (m == 0 || n == 0 || constraint_set.getBounds().empty())
    return jac;

  // Cache current variable values
  Eigen::VectorXd x = variables.getValues();

  // Base constraint values at x
  const Eigen::VectorXd g = constraint_set.getValues();

  Eigen::VectorXd x_new = x;
  Eigen::VectorXd delta_g(m);

  // Forward-difference approximation for each variable
  for (Eigen::Index i = 0; i < n; ++i)
  {
    x_new(i) = x(i) + epsilon;  // disturb variable i
    variables.setVariables(x_new);
    constraint_set.update();

    const Eigen::VectorXd& g_new = constraint_set.getValues();
    delta_g = (g_new - g) / epsilon;

    for (int j = 0; j < m; ++j)
      jac.coeffRef(j, static_cast<int>(i)) = delta_g(j);

    x_new(i) = x(i);  // reset for next iteration
  }

  // Restore original variables
  variables.setVariables(x);

  return jac;
}
}  // namespace trajopt_ifopt
