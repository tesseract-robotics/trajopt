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
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
JointJerkConstraint::JointJerkConstraint(const Eigen::VectorXd& targets,
                                         const std::vector<std::shared_ptr<const Var>>& position_vars,
                                         const Eigen::VectorXd& coeffs,
                                         const std::string& name)
  : ifopt::ConstraintSet(static_cast<int>(targets.size()) * static_cast<int>(position_vars.size()), name)
  , n_dof_(targets.size())
  , n_vars_(static_cast<long>(position_vars.size()))
  , coeffs_(coeffs)
  , position_vars_(position_vars)
{
  if (position_vars_.size() < 6)
    throw std::runtime_error("JointJerkConstraint requires a minimum of six position variables!");

  // Check and make sure the targets size aligns with the vars passed in
  for (const auto& position_var : position_vars)
  {
    if (targets.size() != position_var->size())
      CONSOLE_BRIDGE_logError("Targets size does not align with variables provided");
  }

  // Set n_dof and n_vars
  assert(n_dof_ > 0);
  assert(n_vars_ > 0);

  if (!(coeffs_.array() > 0).all())
    throw std::runtime_error("JointJerkConstraint, coeff must be greater than zero.");

  if (coeffs_.rows() == 1)
    coeffs_ = Eigen::VectorXd::Constant(n_dof_, coeffs(0));

  if (coeffs_.rows() != n_dof_)
    throw std::runtime_error("JointJerkConstraint, coeff must be the same size of the joint postion.");

  // Set the bounds to the input targets
  std::vector<ifopt::Bounds> bounds(static_cast<std::size_t>(GetRows()));
  // All of the positions should be exactly at their targets
  for (long j = 0; j < n_vars_; j++)
  {
    for (long i = 0; i < n_dof_; i++)
      bounds[static_cast<std::size_t>(i + (j * n_dof_))] = ifopt::Bounds(targets[i], targets[i]);
  }
  bounds_ = bounds;
}

Eigen::VectorXd JointJerkConstraint::GetValues() const
{
  Eigen::VectorXd acceleration(static_cast<std::size_t>(n_dof_) * position_vars_.size());
  // Forward Diff
  for (std::size_t ind = 0; ind < position_vars_.size() - 3; ind++)
  {
    auto vals1 = position_vars_[ind]->value();
    auto vals2 = position_vars_[ind + 1]->value();
    auto vals3 = position_vars_[ind + 2]->value();
    auto vals4 = position_vars_[ind + 3]->value();
    const Eigen::VectorXd single_step = (3.0 * vals2) - (3.0 * vals3) - vals1 + vals4;
    acceleration.block(n_dof_ * static_cast<Eigen::Index>(ind), 0, n_dof_, 1) = coeffs_.cwiseProduct(single_step);
  }

  // Backward Diff
  for (std::size_t ind = position_vars_.size() - 3; ind < position_vars_.size(); ind++)
  {
    auto vals1 = position_vars_[ind]->value();
    auto vals2 = position_vars_[ind - 1]->value();
    auto vals3 = position_vars_[ind - 2]->value();
    auto vals4 = position_vars_[ind - 3]->value();
    const Eigen::VectorXd single_step = vals1 - (3.0 * vals2) + (3.0 * vals3) - vals4;
    acceleration.block(n_dof_ * static_cast<Eigen::Index>(ind), 0, n_dof_, 1) = coeffs_.cwiseProduct(single_step);
  }

  return acceleration;
}

// Set the limits on the constraint values (in this case just the targets)
std::vector<ifopt::Bounds> JointJerkConstraint::GetBounds() const { return bounds_; }

void JointJerkConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Check if this constraint use the var_set
  // Only modify the jacobian if this constraint uses var_set
  if (var_set != position_vars_.front()->getParent()->getParent()->GetName())
    return;

  // Reserve enough room in the sparse matrix
  std::vector<Eigen::Triplet<double>> triplet_list;
  triplet_list.reserve(static_cast<std::size_t>(n_dof_ * 4));

  // jac block will be (n_vars-1)*n_dof x n_dof
  Eigen::Index prev_idx3 = -1;
  Eigen::Index prev_idx2 = -1;
  Eigen::Index prev_idx1 = -1;
  Eigen::Index idx = -1;
  Eigen::Index post_idx1 = -1;
  Eigen::Index post_idx2 = -1;
  Eigen::Index post_idx3 = -1;

  for (std::size_t i = 0; i < n_vars_; i++)
  {
    idx = position_vars_[i]->getIndex();

    if (i > 0)
      prev_idx1 = position_vars_[i - 1]->getIndex();

    if (i > 1)
      prev_idx2 = position_vars_[i - 2]->getIndex();

    if (i > 2)
      prev_idx3 = position_vars_[i - 3]->getIndex();

    if (i < n_vars_ - 1)
      post_idx1 = position_vars_[i + 1]->getIndex();

    if (i < n_vars_ - 2)
      post_idx2 = position_vars_[i + 2]->getIndex();

    if (i < n_vars_ - 3)
      post_idx3 = position_vars_[i + 3]->getIndex();

    for (int j = 0; j < n_dof_; j++)
    {
      // The last two variable are special and only effect the last two constraints.
      // Everything else effects 3
      if (i < n_vars_ - 3)
        triplet_list.emplace_back(idx + j, idx + j, -1.0 * coeffs_[j]);

      if (i > 0 && i < n_vars_ - 2)
        triplet_list.emplace_back(prev_idx1 + j, idx + j, 3.0 * coeffs_[j]);

      if (i > 1 && i < n_vars_ - 1)
        triplet_list.emplace_back(prev_idx2 + j, idx + j, -3.0 * coeffs_[j]);

      if (i > 2)
        triplet_list.emplace_back(prev_idx3 + j, idx + j, 1.0 * coeffs_[j]);

      if (i >= (n_vars_ - 3) && i <= (n_vars_ - 1))
        triplet_list.emplace_back(idx + j, idx + j, 1.0 * coeffs_[j]);

      if (i >= (n_vars_ - 4) && i <= (n_vars_ - 2))
        triplet_list.emplace_back(post_idx1 + j, idx + j, -3.0 * coeffs_[j]);

      if (i >= (n_vars_ - 5) && i <= (n_vars_ - 3))
        triplet_list.emplace_back(post_idx2 + j, idx + j, 3.0 * coeffs_[j]);

      if (i >= (n_vars_ - 6) && i <= (n_vars_ - 4))
        triplet_list.emplace_back(post_idx3 + j, idx + j, -1.0 * coeffs_[j]);
    }
  }
  jac_block.setFromTriplets(triplet_list.begin(), triplet_list.end());  // NOLINT
}

}  // namespace trajopt_ifopt
