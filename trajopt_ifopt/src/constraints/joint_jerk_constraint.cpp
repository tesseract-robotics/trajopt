/**
 * @file joint_jerk_constraint.cpp
 * @brief The joint jerk constraint
 *
 * @author Levi Armstrong
 * @date November 1, 2021
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
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cassert>
#include <stdexcept>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
JointJerkConstraint::JointJerkConstraint(const Eigen::VectorXd& targets,
                                         const std::vector<std::shared_ptr<const Var>>& position_vars,
                                         const Eigen::VectorXd& coeffs,
                                         std::string name)
  : ConstraintSet(std::move(name),
                  static_cast<int>(targets.size()) * (static_cast<int>(position_vars.size()) - DIFFERENCE_ORDER))
  , n_dof_(targets.size())
  , n_vars_(static_cast<long>(position_vars.size()))
  , position_vars_(position_vars)
{
  if (position_vars_.size() < DIFFERENCE_ORDER + 1)
    throw std::runtime_error("JointJerkConstraint requires a minimum of four position variables!");

  // Check and make sure the targets size aligns with the vars passed in
  for (const auto& position_var : position_vars_)
  {
    if (targets.size() != position_var->size())
      throw std::runtime_error("JointJerkConstraint, targets must be the same size as each position variable.");
  }

  assert(n_dof_ > 0);

  // Each third difference contributes 4 nonzeros per DOF
  non_zeros_ = (DIFFERENCE_ORDER + 1) * n_dof_ * (n_vars_ - DIFFERENCE_ORDER);

  if (!coeffs.allFinite() || !(coeffs.array() > 0).all())
    throw std::runtime_error("JointJerkConstraint, coeff must be finite and greater than zero.");

  if (coeffs.rows() == 0)
    coeffs_ = Eigen::VectorXd::Ones(rows_);
  else if (coeffs.rows() == 1)
    coeffs_ = Eigen::VectorXd::Constant(rows_, coeffs(0));
  else if (coeffs.rows() == n_dof_)
    coeffs_ = coeffs.replicate(n_vars_ - DIFFERENCE_ORDER, 1);
  else
    throw std::runtime_error("JointJerkConstraint, coeff must be the same size of the joint position.");

  // Set the bounds to the input targets
  const Eigen::VectorXd row_targets = targets.replicate(n_vars_ - DIFFERENCE_ORDER, 1);
  bounds_ = toBounds(row_targets, row_targets);
}

Eigen::VectorXd JointJerkConstraint::getValues() const
{
  const std::size_t n = position_vars_.size();
  Eigen::VectorXd jerk(rows_);

  // Forward difference: j_i = -q_i + 3*q_{i+1} - 3*q_{i+2} + q_{i+3}
  for (std::size_t i = 0; i < n - DIFFERENCE_ORDER; ++i)
  {
    const Eigen::VectorXd& q0 = position_vars_[i]->value();
    const Eigen::VectorXd& q1 = position_vars_[i + 1]->value();
    const Eigen::VectorXd& q2 = position_vars_[i + 2]->value();
    const Eigen::VectorXd& q3 = position_vars_[i + 3]->value();

    jerk.segment(static_cast<Eigen::Index>(i) * n_dof_, n_dof_) = -q0 + 3.0 * q1 - 3.0 * q2 + q3;
  }

  return jerk;
}

Eigen::VectorXd JointJerkConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values (in this case just the targets)
std::vector<Bounds> JointJerkConstraint::getBounds() const { return bounds_; }

Jacobian JointJerkConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  const std::size_t n = position_vars_.size();

  for (std::size_t i = 0; i < n - DIFFERENCE_ORDER; ++i)
  {
    const Eigen::Index row_offset = static_cast<Eigen::Index>(i) * n_dof_;

    // Forward difference: j_i = -q_i + 3*q_{i+1} - 3*q_{i+2} + q_{i+3}
    const Eigen::Index col_i = position_vars_[i]->getIndex();
    const Eigen::Index col_ip1 = position_vars_[i + 1]->getIndex();
    const Eigen::Index col_ip2 = position_vars_[i + 2]->getIndex();
    const Eigen::Index col_ip3 = position_vars_[i + 3]->getIndex();

    for (Eigen::Index k = 0; k < n_dof_; ++k)
    {
      const Eigen::Index row = row_offset + k;
      jac.startVec(row);
      jac.insertBack(row, col_i + k) = -1;      // ∂j_i/∂q_i
      jac.insertBack(row, col_ip1 + k) = 3.0;   // ∂j_i/∂q_{i+1}
      jac.insertBack(row, col_ip2 + k) = -3.0;  // ∂j_i/∂q_{i+2}
      jac.insertBack(row, col_ip3 + k) = 1;     // ∂j_i/∂q_{i+3}
    }
  }

  jac.finalize();  // NOLINT
  return jac;
}

}  // namespace trajopt_ifopt
