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
  : ConstraintSet(constraint->getName() + "_squared_cost", 1)
  , constraint_(std::move(constraint))
  , n_constraints_(constraint_->getRows())
  , weights_(weights.cwiseAbs())
{
  values_ = Eigen::VectorXd::Zero(1);
  coeffs_ = Eigen::VectorXd::Ones(1);
  bounds_ = std::vector<Bounds>(1, NoBound);
}

int SquaredCost::update()
{
  constraint_->update();

  Eigen::VectorXd error(constraint_->getRows());
  calcBoundsErrors(error, constraint_->getValues(), constraint_->getBounds());
  // cost = sum_i w_i * e_i^2
  values_[0] = (weights_.array() * error.array().square()).sum();

  return rows_;
}

const Eigen::VectorXd& SquaredCost::getValues() const { return values_; }

const Eigen::VectorXd& SquaredCost::getCoefficients() const { return coeffs_; }

const std::vector<Bounds>& SquaredCost::getBounds() const { return bounds_; }

void SquaredCost::fillJacobianBlock(std::string var_set, Jacobian& jac_block) const
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
    throw std::runtime_error("SquaredCost: Unable to find var_set '" + var_set + "'.");

  // Get the Jacobian block from the constraint
  Jacobian cnt_jac_block;
  cnt_jac_block.resize(n_constraints_, var_size);  // NOLINT
  constraint_->fillJacobianBlock(var_set, cnt_jac_block);

  // error = bounds error vector (length = n_constraints_)
  Eigen::VectorXd error(constraint_->getRows());
  calcBoundsErrors(error, constraint_->getValues(), constraint_->getBounds());

  // coeff_i = 2 * w_i * e_i
  const Eigen::VectorXd coeff = (2.0 * (weights_.array() * error.array())).matrix();

  // Gradient row: 1 x var_size
  // (dense row vector = coeff^T * J)
  const Eigen::RowVectorXd grad = coeff.transpose() * cnt_jac_block;

  // Convert to sparse and return
  jac_block = grad.sparseView();
}

}  // namespace trajopt_ifopt
