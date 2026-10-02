/**
 * @file warm_start.h
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
#ifndef TRAJOPT_SQP_WARM_START_H
#define TRAJOPT_SQP_WARM_START_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/eigen_types.h>
#include <trajopt_sqp/fwd.h>

namespace trajopt_sqp
{
/**
 * @brief The primal start point of the convexified QP of @p qp_problem
 * @details The NLP variables at the current iterate clamped into their bounds (the variable limits within the trust
 * box), and each slack at the smallest non-negative value that makes its rows hold there (see withMinimalSlacks()).
 * @return One entry per QP variable
 */
Eigen::VectorXd qpStartPoint(const QPProblem& qp_problem);

/**
 * @brief @p nlp_values followed by the smallest non-negative slacks that make the rows lower <= A x <= upper hold
 * @details Columns [0, nlp_values.size()) of @p A are the NLP variables and the remaining columns are slacks.
 * @p nlp_values are taken as given; a violated row with no slack of the sign it needs stays violated. Each slack
 * column must appear in at most one row that also has NLP variables, as in TrajOptQPProblem's layout: a row is sized
 * by its NLP part alone, with its other slacks taken at zero.
 * @return One entry per column of @p A
 */
Eigen::VectorXd withMinimalSlacks(const trajopt_ifopt::Jacobian& A,
                                  const Eigen::Ref<const Eigen::VectorXd>& lower,
                                  const Eigen::Ref<const Eigen::VectorXd>& upper,
                                  const Eigen::Ref<const Eigen::VectorXd>& nlp_values);
}  // namespace trajopt_sqp

#endif
