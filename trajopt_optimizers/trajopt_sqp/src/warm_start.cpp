/**
 * @file warm_start.cpp
 * @brief The QP start point at a linearization point
 *
 * @author Roelof Oomen
 * @date October 1, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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
#include <trajopt_sqp/warm_start.h>
#include <trajopt_sqp/qp_problem.h>

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <cassert>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_sqp
{
Eigen::VectorXd qpStartPoint(const QPProblem& qp_problem)
{
  const Eigen::VectorXd nlp_values = qp_problem.getVariableValues()
                                         .cwiseMax(qp_problem.getNLPVariableBoundsLower())
                                         .cwiseMin(qp_problem.getNLPVariableBoundsUpper());
  return withMinimalSlacks(
      qp_problem.getConstraintMatrix(), qp_problem.getBoundsLower(), qp_problem.getBoundsUpper(), nlp_values);
}

Eigen::VectorXd withMinimalSlacks(const trajopt_ifopt::Jacobian& A,
                                  const Eigen::Ref<const Eigen::VectorXd>& lower,
                                  const Eigen::Ref<const Eigen::VectorXd>& upper,
                                  const Eigen::Ref<const Eigen::VectorXd>& nlp_values)
{
  const Eigen::Index n_nlp = nlp_values.size();
  assert(n_nlp <= A.cols());
  assert(lower.size() == A.rows() && upper.size() == A.rows());

  Eigen::VectorXd x = Eigen::VectorXd::Zero(A.cols());
  x.head(n_nlp) = nlp_values;

  for (Eigen::Index r = 0; r < A.outerSize(); ++r)
  {
    double v{ 0 };
    for (trajopt_ifopt::Jacobian::InnerIterator it(A, r); it; ++it)
    {
      if (it.col() < n_nlp)
        v += it.value() * x(it.col());
    }

    double shortfall{ 0 };  // signed: positive needs a slack of positive coefficient, negative one of negative
    if (v < lower(r))
      shortfall = lower(r) - v;
    else if (v > upper(r))
      shortfall = upper(r) - v;
    else
      continue;

    for (trajopt_ifopt::Jacobian::InnerIterator it(A, r); it; ++it)
    {
      if (it.col() >= n_nlp && it.value() * shortfall > 0)
      {
        x(it.col()) = std::max(x(it.col()), shortfall / it.value());
        break;
      }
    }
  }
  return x;
}
}  // namespace trajopt_sqp
