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
}

int SquaredCost::update()
{
  constraint_->update();
  return rows_;
}

double SquaredCost::getCost() const
{
  Eigen::VectorXd error(constraint_->getRows());
  calcBoundsErrors(error, constraint_->getValues(), constraint_->getBounds());
  // cost = sum_i w_i * e_i^2
  return (weights_.array() * error.array().square()).sum();
}

Eigen::VectorXd SquaredCost::getCoefficients() const { return constraint_->getCoefficients(); }

void SquaredCost::fillJacobianBlock(std::vector<Eigen::Triplet<double>>& jac_block,
                                    const std::string& var_set,
                                    Eigen::Index row_index,
                                    Eigen::Index col_index) const
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
  std::vector<Eigen::Triplet<double>> triplets;
  constraint_->fillJacobianBlock(triplets, var_set, 0, 0);

  Jacobian cnt_jac_block;
  cnt_jac_block.resize(n_constraints_, var_size);  // NOLINT
  cnt_jac_block.setFromTriplets(triplets.begin(), triplets.end());

  // error = bounds error vector (length = n_constraints_)
  Eigen::VectorXd error(constraint_->getRows());
  calcBoundsErrors(error, constraint_->getValues(), constraint_->getBounds());

  // coeff_i = 2 * w_i * e_i
  const Eigen::VectorXd coeff = (2.0 * (weights_.array() * error.array())).matrix();

  // Gradient row: 1 x var_size
  // (dense row vector = coeff^T * J)
  const Eigen::RowVectorXd grad = coeff.transpose() * cnt_jac_block;

  // Convert to sparse and return
  Jacobian jac = grad.sparseView();

  for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
    for (Jacobian::InnerIterator it(jac, k); it; ++it)
      jac_block.emplace_back(row_index + it.row(), col_index + it.col(), it.value());
}

}  // namespace trajopt_ifopt
