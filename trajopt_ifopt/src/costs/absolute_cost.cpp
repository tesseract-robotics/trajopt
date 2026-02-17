/**
 * @file absolute_cost.cpp
 * @brief The absolute cost. Converts a constraint into a cost.
 *
 * @author Levi Armstrong
 * @date May 20, 20201
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <trajopt_ifopt/costs/absolute_cost.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
AbsoluteCost::AbsoluteCost(const ConstraintSet::Ptr& constraint)
  : AbsoluteCost(constraint, Eigen::VectorXd::Ones(constraint->getRows()))
{
}

AbsoluteCost::AbsoluteCost(ConstraintSet::Ptr constraint, const Eigen::Ref<const Eigen::VectorXd>& weights)
  : CostTerm(constraint->getName() + "_absolute_cost")
  , constraint_(std::move(constraint))
  , n_constraints_(constraint_->getRows())
  , weights_(weights.cwiseAbs())  // must be positive
{
  non_zeros_ = weights.size();
}

int AbsoluteCost::update()
{
  constraint_->update();
  return rows_;
}

double AbsoluteCost::getCost() const
{
  // This takes the absolute value of the errors
  scratch_error_.resize(constraint_->getRows());
  calcBoundsViolations(scratch_error_, constraint_->getValues(), constraint_->getBounds());
  return weights_.dot(scratch_error_);
}

Eigen::VectorXd AbsoluteCost::getCoefficients() const { return constraint_->getCoefficients(); }

Jacobian AbsoluteCost::getJacobian() const
{
  // Get the Jacobian block from the underlying constraint
  Jacobian cnt_jac_block = constraint_->getJacobian();

  // Compute signed coefficients: coeff_i = weights_[i] * sign(error_i)
  scratch_error_.resize(constraint_->getRows());
  calcBoundsErrors(scratch_error_, constraint_->getValues(), constraint_->getBounds());

  Eigen::VectorXd coeff(n_constraints_);
  for (Eigen::Index i = 0; i < scratch_error_.size(); ++i)
  {
    const double e = scratch_error_[i];
    if (std::abs(e) < 1e-12)
      coeff[i] = 0.0;  // subgradient at 0
    else if (e > 0.0)
      coeff[i] = weights_[i];
    else
      coeff[i] = -weights_[i];
  }

  // Gradient row: 1 x var_size
  // CostTerm reports rows_ = 1, so the Jacobian must be a single row.
  // (dense row vector = coeff^T * J)
  const Eigen::RowVectorXd grad = coeff.transpose() * cnt_jac_block;

  // Convert to sparse and return
  return grad.sparseView();
}

}  // namespace trajopt_ifopt
