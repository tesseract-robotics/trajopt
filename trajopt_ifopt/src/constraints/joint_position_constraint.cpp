/**
 * @file joint_position_constraint.cpp
 * @brief The joint position constraint
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
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
JointPosConstraint::JointPosConstraint(const Eigen::VectorXd& target,
                                       const std::shared_ptr<const Var>& position_var,
                                       const Eigen::VectorXd& coeffs,
                                       std::string name)
  : ConstraintSet(std::move(name), static_cast<int>(target.size()))
  , n_dof_(target.size())
  , coeffs_(coeffs)
  , position_var_(position_var)
{
  // Set the n_dof and n_vars for convenience
  assert(n_dof_ > 0);

  if (!(coeffs_.array() > 0).all())
    throw std::runtime_error("JointPosConstraint, coeff must be greater than zero.");

  if (coeffs_.rows() == 1)
    coeffs_ = Eigen::VectorXd::Constant(n_dof_, coeffs(0));

  if (coeffs_.rows() != n_dof_)
    throw std::runtime_error("JointPosConstraint, coeff must be the same size of the joint postion.");

  // Check and make sure the targets size aligns with the vars passed in
  if (target.size() != position_var->size())
    CONSOLE_BRIDGE_logError("Targets size does not align with variables provided");

  // Set the bounds to the input targets
  std::vector<Bounds> bounds(static_cast<std::size_t>(rows_));
  // All of the positions should be exactly at their targets

  for (long i = 0; i < n_dof_; i++)
  {
    const double w_target = target[i];
    bounds[static_cast<std::size_t>(i)] = Bounds(w_target, w_target);
  }

  bounds_ = bounds;
}

JointPosConstraint::JointPosConstraint(const std::vector<Bounds>& bounds,
                                       const std::shared_ptr<const Var>& position_var,
                                       const Eigen::VectorXd& coeffs,
                                       std::string name)
  : ConstraintSet(std::move(name), static_cast<int>(bounds.size()))
  , coeffs_(coeffs)
  , bounds_(bounds)
  , position_var_(position_var)
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = static_cast<long>(bounds_.size());

  assert(n_dof_ > 0);

  values_ = Eigen::VectorXd::Zero(rows_);
  if (!(coeffs_.array() > 0).all())
    throw std::runtime_error("JointPosConstraint, coeff must be greater than zero.");

  if (coeffs_.rows() == 0)
    coeffs_ = Eigen::VectorXd::Ones(n_dof_);
  else if (coeffs_.rows() == 1)
    coeffs_ = Eigen::VectorXd::Constant(n_dof_, coeffs(0));
  else if (coeffs_.rows() != n_dof_)
    throw std::runtime_error("JointPosConstraint, coeff must be the same size of the joint postion.");

  // Check and make sure the targets size aligns with the vars passed in
  if (static_cast<long>(bounds_.size()) != position_var_->size())
    CONSOLE_BRIDGE_logError("Bounds size does not align with variables provided");
}

int JointPosConstraint::update()
{
  values_ = position_var_->value();
  return rows_;
}

const Eigen::VectorXd& JointPosConstraint::getValues() const { return values_; }

const Eigen::VectorXd& JointPosConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values
const std::vector<Bounds>& JointPosConstraint::getBounds() const { return bounds_; }

void JointPosConstraint::fillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Check if this constraint use the var_set
  // Only modify the jacobian if this constraint uses var_set
  if (var_set != position_var_->getParent()->getParent()->getName())
    return;

  // Reserve enough room in the sparse matrix
  std::vector<Eigen::Triplet<double>> triplet_list;
  triplet_list.reserve(static_cast<std::size_t>(n_dof_));

  // Loop over all of the variables this constraint uses
  for (int j = 0; j < n_dof_; j++)  // NOLINT
    triplet_list.emplace_back(j, position_var_->getIndex() + j, 1.0);

  jac_block.setFromTriplets(triplet_list.begin(), triplet_list.end());  // NOLINT
}
}  // namespace trajopt_ifopt
