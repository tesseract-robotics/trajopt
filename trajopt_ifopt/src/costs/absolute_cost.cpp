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
  : ConstraintSet(constraint->getName() + "_absolute_cost", 1)
  , constraint_(std::move(constraint))
  , n_constraints_(constraint_->getRows())
  , weights_(weights.cwiseAbs())  // must be positive

{
  values_ = Eigen::VectorXd::Zero(1);
  coeffs_ = Eigen::VectorXd::Ones(1);
  bounds_ = std::vector<Bounds>(1, NoBound);
}

int AbsoluteCost::update()
{
  constraint_->update();

  Eigen::VectorXd error(constraint_->getRows());
  calcBoundsViolations(error, constraint_->getValues(), constraint_->getBounds());
  values_[0] = weights_.dot(error);

  return rows_;
}

const Eigen::VectorXd& AbsoluteCost::getValues() const { return values_; }

const Eigen::VectorXd& AbsoluteCost::getCoefficients() const { return coeffs_; }

const std::vector<Bounds>& AbsoluteCost::getBounds() const { return bounds_; }

void AbsoluteCost::fillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Get a Jacobian block the size necessary for the constraint
  int var_size = 0;
  for (const auto& vars : getVariables()->getComponents())
  {
    if (vars->getName() == var_set)  // NOLINT
    {
      var_size = vars->getRows();
      break;
    }
  }

  if (var_size == 0)  // NOLINT
    throw std::runtime_error("AbsoluteCost: Unable to find var_set '" + var_set + "'.");

  // Get the Jacobian block from the underlying constraint
  Jacobian cnt_jac_block;
  cnt_jac_block.resize(n_constraints_, var_size);  // NOLINT
  constraint_->fillJacobianBlock(var_set, cnt_jac_block);

  // Compute signed coefficients: coeff_i = weights_[i] * sign(error_i)
  Eigen::ArrayXd error(constraint_->getRows());
  calcBoundsErrors(error, constraint_->getValues(), constraint_->getBounds());

  Eigen::VectorXd coeff(n_constraints_);
  for (Eigen::Index i = 0; i < error.size(); ++i)
  {
    const double e = error[i];
    if (std::abs(e) < 1e-12)
      coeff[i] = 0.0;  // subgradient at 0
    else if (e > 0.0)
      coeff[i] = weights_[i];
    else
      coeff[i] = -weights_[i];
  }

  // Scale each row of the constraint Jacobian by coeff[row]
  for (int outer = 0; outer < cnt_jac_block.outerSize(); ++outer)
  {
    for (Jacobian::InnerIterator it(cnt_jac_block, outer); it; ++it)
    {
      // it.row() is the row index in [0, n_constraints_)
      it.valueRef() *= coeff[it.row()];
    }
  }

  // Output the scaled block
  jac_block = cnt_jac_block;
}

}  // namespace trajopt_ifopt
