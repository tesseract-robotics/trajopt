/**
 * @file joint_acceleration_constraint.h
 * @brief The joint_acceleration constraint
 *
 * @author Ben Greenberg
 * @date April 22, 2021
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
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
JointAccelConstraint::JointAccelConstraint(const Eigen::VectorXd& targets,
                                           const std::vector<std::shared_ptr<const Var>>& position_vars,
                                           const Eigen::VectorXd& coeffs,
                                           std::string name)
  : ConstraintSet(std::move(name), static_cast<int>(targets.size()) * static_cast<int>(position_vars.size()))
  , n_dof_(targets.size())
  , n_vars_(static_cast<long>(position_vars.size()))
  , position_vars_(position_vars)
{
  if (position_vars_.size() < 4)
    throw std::runtime_error("JointAccelConstraint requires a minimum of four position variables!");

  // Check and make sure the targets size aligns with the vars passed in
  for (const auto& position_var : position_vars_)
  {
    if (targets.size() != position_var->size())
      CONSOLE_BRIDGE_logError("Targets size does not align with variables provided");
  }

  // Set n_dof and n_vars
  assert(n_dof_ > 0);
  assert(n_vars_ > 0);

  // Each timestep contributes 3 nonzeros per DOF
  non_zeros_ = 3 * n_dof_ * n_vars_;

  if (!(coeffs.array() > 0).all())
    throw std::runtime_error("JointAccelConstraint, coeff must be greater than zero.");

  if (coeffs.rows() == 0)
    coeffs_ = Eigen::VectorXd::Ones(n_dof_ * n_vars_);
  else if (coeffs.rows() == 1)
    coeffs_ = Eigen::VectorXd::Constant(n_dof_ * n_vars_, coeffs(0));
  else if (coeffs.rows() != n_dof_)
    throw std::runtime_error("JointAccelConstraint, coeff must be the same size of the joint postion.");

  if (coeffs.rows() == n_dof_)
  {
    coeffs_.resize(n_dof_ * n_vars_);
    for (long j = 0; j < n_vars_; j++)
      coeffs_.segment(j * n_dof_, n_dof_) = coeffs;
  }

  // Set the bounds to the input targets
  // All of the positions should be exactly at their targets
  std::vector<Bounds> bounds(static_cast<std::size_t>(rows_));
  for (long j = 0; j < n_vars_; j++)
  {
    for (long i = 0; i < n_dof_; i++)
      bounds[static_cast<std::size_t>(i + (j * n_dof_))] = Bounds(targets[i], targets[i]);
  }
  bounds_ = bounds;
}

Eigen::VectorXd JointAccelConstraint::getValues() const
{
  const std::size_t n = position_vars_.size();
  Eigen::VectorXd acceleration(n_dof_ * static_cast<Eigen::Index>(n));

  // Forward diff for timesteps [0, n-3]
  for (std::size_t i = 0; i < n - 2; ++i)
  {
    const Eigen::VectorXd& q0 = position_vars_[i]->value();
    const Eigen::VectorXd& q1 = position_vars_[i + 1]->value();
    const Eigen::VectorXd& q2 = position_vars_[i + 2]->value();

    const Eigen::VectorXd single_step = q2 - 2.0 * q1 + q0;
    acceleration.segment(static_cast<Eigen::Index>(i) * n_dof_, n_dof_) = single_step;
  }

  // Backward diff for the last two timesteps [n-2, n-1]
  for (std::size_t i = n - 2; i < n; ++i)
  {
    const Eigen::VectorXd& q0 = position_vars_[i]->value();      // q_i
    const Eigen::VectorXd& q1 = position_vars_[i - 1]->value();  // q_{i-1}
    const Eigen::VectorXd& q2 = position_vars_[i - 2]->value();  // q_{i-2}

    const Eigen::VectorXd single_step = q2 - 2.0 * q1 + q0;
    acceleration.segment(static_cast<Eigen::Index>(i) * n_dof_, n_dof_) = single_step;
  }

  return acceleration;
}

Eigen::VectorXd JointAccelConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values (in this case just the targets)
std::vector<Bounds> JointAccelConstraint::getBounds() const { return bounds_; }

Jacobian JointAccelConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  const std::size_t n = position_vars_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const Eigen::Index row_offset = static_cast<Eigen::Index>(i) * n_dof_;

    if (i < n - 2)
    {
      // Forward diff at timestep i: depends on q_i, q_{i+1}, q_{i+2}
      const Eigen::Index col_i = position_vars_[i]->getIndex();
      const Eigen::Index col_ip1 = position_vars_[i + 1]->getIndex();
      const Eigen::Index col_ip2 = position_vars_[i + 2]->getIndex();

      for (Eigen::Index k = 0; k < n_dof_; ++k)
      {
        const Eigen::Index row = row_offset + k;
        jac.startVec(row);
        // a_i = c * (q_{i+2} - 2*q_{i+1} + q_i)
        jac.insertBack(row, col_i + k) = 1;       // ∂a/∂q_i
        jac.insertBack(row, col_ip1 + k) = -2.0;  // ∂a/∂q_{i+1}
        jac.insertBack(row, col_ip2 + k) = 1;     // ∂a/∂q_{i+2}
      }
    }
    else
    {
      // Backward diff at timesteps n-2 and n-1: depends on q_{i-2}, q_{i-1}, q_i
      const Eigen::Index col_im2 = position_vars_[i - 2]->getIndex();
      const Eigen::Index col_im1 = position_vars_[i - 1]->getIndex();
      const Eigen::Index col_i = position_vars_[i]->getIndex();

      for (Eigen::Index k = 0; k < n_dof_; ++k)
      {
        const Eigen::Index row = row_offset + k;
        jac.startVec(row);
        // a_i = c * (q_{i-2} - 2*q_{i-1} + q_i)
        jac.insertBack(row, col_im2 + k) = 1;     // ∂a/∂q_{i-2}
        jac.insertBack(row, col_im1 + k) = -2.0;  // ∂a/∂q_{i-1}
        jac.insertBack(row, col_i + k) = 1;       // ∂a/∂q_i
      }
    }
  }

  jac.finalize();  // NOLINT
  return jac;
}

}  // namespace trajopt_ifopt
