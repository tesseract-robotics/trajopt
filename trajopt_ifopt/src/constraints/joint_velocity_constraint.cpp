/**
 * @file joint_velocity_constraint.h
 * @brief The joint_velocity constraint
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
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
JointVelConstraint::JointVelConstraint(const Eigen::VectorXd& targets,
                                       const std::vector<std::shared_ptr<const Var>>& position_vars,
                                       const Eigen::VectorXd& coeffs,
                                       std::string name)
  : ConstraintSet(std::move(name), static_cast<int>(targets.size()) * static_cast<int>(position_vars.size() - 1))
  , n_dof_(targets.size())
  , n_vars_(static_cast<long>(position_vars.size()))
  , position_vars_(position_vars)
{
  if (position_vars_.size() < 2)
    throw std::runtime_error("JointVelConstraint, requires minimum of three position variables!");

  // Check and make sure the targets size aligns with the vars passed in
  for (const auto& position_var : position_vars_)
  {
    if (targets.size() != position_var->size())
      CONSOLE_BRIDGE_logError("Targets size does not align with variables provided");
  }

  // Set n_dof and n_vars
  assert(n_dof_ > 0);
  assert(n_vars_ > 0);

  // Each segment contributes 2 * n_dof_ nonzeros (− and +)
  non_zeros_ = 2 * (n_vars_ - 1) * n_dof_;

  if (!(coeffs.array() > 0).all())
    throw std::runtime_error("JointVelConstraint, coeff must be greater than zero.");

  if (coeffs.rows() == 0)
    coeffs_ = Eigen::VectorXd::Constant(n_dof_ * (n_vars_ - 1), 5);
  else if (coeffs.rows() == 1)
    coeffs_ = Eigen::VectorXd::Constant(n_dof_ * (n_vars_ - 1), coeffs(0));
  else if (coeffs.rows() != n_dof_)
    throw std::runtime_error("JointVelConstraint, coeff must be the same size of the joint position.");

  if (coeffs.rows() == n_dof_)
  {
    coeffs_.resize(n_dof_ * (n_vars_ - 1));
    for (long j = 0; j < n_vars_ - 1; j++)
      coeffs_.segment(j * n_dof_, n_dof_) = coeffs;
  }

  // Set the bounds to the input targets
  std::vector<Bounds> bounds(static_cast<std::size_t>(rows_));
  // All of the positions should be exactly at their targets
  for (long j = 0; j < n_vars_ - 1; j++)
  {
    for (long i = 0; i < n_dof_; i++)
      bounds[static_cast<std::size_t>(i + (j * n_dof_))] = Bounds(targets[i], targets[i]);
  }
  bounds_ = bounds;
}

Eigen::VectorXd JointVelConstraint::getValues() const
{
  // i - represents the trajectory timestep index
  // k - represents the DOF index
  // var[i, k] - represents the variable index
  // vel(var[0, 0]) - represents the joint velocity of DOF index 0 at timestep 0
  // vel(var[1, 1]) - represents the joint velocity of DOF index 1 at timestep 1
  //
  // Velocity V = vel(var[0, 0]), vel(var[0, 1]), vel(var[0, 2]), vel(var[1, 0]), vel(var[1, 1]), vel(var[1, 2]), etc

  // Number of velocity segments (one less than number of position vars)
  const Eigen::Index n_segments = n_vars_ - 1;

  Eigen::VectorXd velocity(n_dof_ * n_segments);

  for (Eigen::Index seg = 0; seg < n_segments; ++seg)
  {
    // q_i and q_{i+1}
    const Eigen::VectorXd& q0 = position_vars_[static_cast<std::size_t>(seg)]->value();
    const Eigen::VectorXd& q1 = position_vars_[static_cast<std::size_t>(seg + 1)]->value();

    // v_i = coeffs_ .* (q_{i+1} - q_i)
    velocity.segment(seg * n_dof_, n_dof_) = (q1 - q0);
  }

  return velocity;
}

Eigen::VectorXd JointVelConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values (in this case just the targets)
std::vector<Bounds> JointVelConstraint::getBounds() const { return bounds_; }

Jacobian JointVelConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  for (Eigen::Index seg = 0; seg < (n_vars_ - 1); ++seg)
  {
    const Eigen::Index row_offset = seg * n_dof_;

    // Column indices in this var_set for q_seg and q_{seg+1}
    const Eigen::Index col0 = position_vars_[static_cast<std::size_t>(seg)]->getIndex();
    const Eigen::Index col1 = position_vars_[static_cast<std::size_t>(seg + 1)]->getIndex();

    for (Eigen::Index k = 0; k < n_dof_; ++k)
    {
      const Eigen::Index row = row_offset + k;
      jac.startVec(row);
      // v(seg,k) = c * (q1 - q0)
      jac.insertBack(row, col0 + k) = -1;  // ∂v/∂q_seg
      jac.insertBack(row, col1 + k) = 1;   // ∂v/∂q_{seg+1}
    }
  }

  jac.finalize();  // NOLINT
  return jac;
}

}  // namespace trajopt_ifopt
