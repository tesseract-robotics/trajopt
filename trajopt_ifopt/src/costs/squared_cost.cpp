/**
 * @file squared_cost.cpp
 * @brief The squared cost. Converts a constraint into a cost.
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
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
SquaredCost::SquaredCost(const ConstraintSet::Ptr& constraint)
  : SquaredCost(constraint, Eigen::VectorXd::Ones(constraint->getRows()))
{
}

SquaredCost::SquaredCost(ConstraintSet::Ptr constraint, const Eigen::Ref<const Eigen::VectorXd>& weights)
  : CostTerm(constraint->getName() + "_squared_cost")
  , constraint_(std::move(constraint))
  , n_constraints_(constraint_->getRows())
  , weights_(weights.cwiseAbs())
{
  non_zeros_ = weights.size();
}

int SquaredCost::update()
{
  constraint_->update();
  return rows_;
}

double SquaredCost::getCost() const
{
  scratch_error_.resize(constraint_->getRows());
  calcBoundsErrors(scratch_error_, constraint_->getValues(), constraint_->getBounds());
  // cost = sum_i w_i * e_i^2
  return (weights_.array() * scratch_error_.array().square()).sum();
}

Eigen::VectorXd SquaredCost::getCoefficients() const { return constraint_->getCoefficients(); }

Jacobian SquaredCost::getJacobian() const
{
  // Get the Jacobian block from the underlying constraint
  Jacobian cnt_jac_block = constraint_->getJacobian();

  // error = bounds error vector (length = n_constraints_)
  scratch_error_.resize(constraint_->getRows());
  calcBoundsErrors(scratch_error_, constraint_->getValues(), constraint_->getBounds());

  // coeff_i = 2 * w_i * e_i
  // Scale each row of the sparse Jacobian in-place by coeff_i
  for (Eigen::Index r = 0; r < cnt_jac_block.outerSize(); ++r)
  {
    const double c = 2.0 * weights_[r] * scratch_error_[r];
    for (Jacobian::InnerIterator it(cnt_jac_block, r); it; ++it)
      it.valueRef() *= c;
  }

  // Sum all rows into a single gradient row (1 x n_vars)
  Jacobian grad(1, cnt_jac_block.cols());
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(cnt_jac_block.nonZeros()));

  // Accumulate column sums via a dense scratch vector to avoid duplicate triplet handling
  Eigen::VectorXd col_sums = Eigen::VectorXd::Zero(cnt_jac_block.cols());
  for (Eigen::Index r = 0; r < cnt_jac_block.outerSize(); ++r)
    for (Jacobian::InnerIterator it(cnt_jac_block, r); it; ++it)
      col_sums[it.col()] += it.value();

  for (Eigen::Index j = 0; j < col_sums.size(); ++j)
  {
    if (col_sums[j] != 0.0)
      trips.emplace_back(0, j, col_sums[j]);
  }

  grad.setFromTriplets(trips.begin(), trips.end());
  return grad;
}

}  // namespace trajopt_ifopt
