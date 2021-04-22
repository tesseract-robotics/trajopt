/**
 * @file joint_acceleration_constraint.h
 * @brief The joint_acceleration constraint
 *
 * @author Ben Greenberg
 * @date April 22, 2021
 * @version TODO
 * @bug No known bugs
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

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
JointAccelConstraint::JointAccelConstraint(const Eigen::VectorXd& targets,
<<<<<<< HEAD
                                           std::vector<JointPosition::ConstPtr> position_vars,
                                           const std::string& name)
  : ifopt::ConstraintSet(static_cast<int>(targets.size()) * static_cast<int>(position_vars.size() - 2), name)
  , position_vars_(position_vars)
{
  // Check and make sure the targets size aligns with the vars passed in
  for (auto& position_var : position_vars)
  {
    if (targets.size() != position_var->GetRows())
=======
                                       std::vector<JointVelocity::ConstPtr> velocity_vars,
                                       const std::string& name)
  : ifopt::ConstraintSet(static_cast<int>(targets.size()) * static_cast<int>(velocity_vars.size() - 1), name)
  , velocity_vars_(velocity_vars)
{
  // Check and make sure the targets size aligns with the vars passed in
  for (auto& velocity_var : velocity_vars)
  {
    if (targets.size() != velocity_var->GetRows())
>>>>>>> add joint acceleration constraint
      CONSOLE_BRIDGE_logError("Targets size does not align with variables provided");
  }

  // Set n_dof and n_vars
  n_dof_ = targets.size();
<<<<<<< HEAD
  n_vars_ = static_cast<long>(position_vars.size());
=======
  n_vars_ = static_cast<long>(velocity_vars.size());
>>>>>>> add joint acceleration constraint
  assert(n_dof_ > 0);
  assert(n_vars_ > 0);
  //  assert(n_vars_ == 2);

  // Set the bounds to the input targets
  std::vector<ifopt::Bounds> bounds(static_cast<size_t>(GetRows()));
  // All of the positions should be exactly at their targets
<<<<<<< HEAD
  for (long j = 0; j < n_vars_ - 2; j++)
  {
    index_map_[position_vars[static_cast<std::size_t>(j)]->GetName()] = j;
=======
  for (long j = 0; j < n_vars_ - 1; j++)
  {
>>>>>>> add joint acceleration constraint
    for (long i = 0; i < n_dof_; i++)
    {
      bounds[static_cast<size_t>(i + j * n_dof_)] = ifopt::Bounds(targets[i], targets[i]);
    }
  }
<<<<<<< HEAD
  index_map_[position_vars.back()->GetName()] = static_cast<Eigen::Index>(position_vars.size()) - 1;
=======
>>>>>>> add joint acceleration constraint
  bounds_ = bounds;
}

Eigen::VectorXd JointAccelConstraint::GetValues() const
{
<<<<<<< HEAD
  Eigen::VectorXd acceleration(static_cast<size_t>(n_dof_) * (position_vars_.size() - 2));
  for (std::size_t ind = 0; ind < position_vars_.size() - 2; ind++)
  {
    auto vals1 = GetVariables()->GetComponent(position_vars_[ind]->GetName())->GetValues();
    auto vals2 = GetVariables()->GetComponent(position_vars_[ind + 1]->GetName())->GetValues();
    auto vals3 = GetVariables()->GetComponent(position_vars_[ind + 2]->GetName())->GetValues();
    Eigen::VectorXd single_step = (vals3 - 2*vals2 + vals1);
=======
  Eigen::VectorXd acceleration(static_cast<size_t>(n_dof_) * (velocity_vars_.size() - 1));
  for (std::size_t ind = 0; ind < velocity_vars_.size() - 1; ind++)
  {
    auto vals1 = GetVariables()->GetComponent(velocity_vars_[ind]->GetName())->GetValues();
    auto vals2 = GetVariables()->GetComponent(velocity_vars_[ind + 1]->GetName())->GetValues();
    Eigen::VectorXd single_step = vals1 - vals2;
>>>>>>> add joint acceleration constraint
    acceleration.block(n_dof_ * static_cast<Eigen::Index>(ind), 0, n_dof_, 1) = single_step;
  }

  return acceleration;
}

// Set the limits on the constraint values (in this case just the targets)
std::vector<ifopt::Bounds> JointAccelConstraint::GetBounds() const { return bounds_; }

void JointAccelConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Loop over all of the variables this constraint uses
  for (int i = 0; i < n_vars_; i++)
  {
    // Only modify the jacobian if this constraint uses var_set
<<<<<<< HEAD
    if (var_set == position_vars_[static_cast<size_t>(i)]->GetName())
=======
    if (var_set == velocity_vars_[static_cast<size_t>(i)]->GetName())
>>>>>>> add joint acceleration constraint
    {
      // Reserve enough room in the sparse matrix
      jac_block.reserve(n_dof_ * 2);

      // jac block will be (n_vars-1)*n_dof x n_dof
      for (int j = 0; j < n_dof_; j++)
      {
        // The first and last variable are special and only effect the first and last constraint. Everything else
        // effects 2
<<<<<<< HEAD
        if (i < n_vars_ - 2)
          jac_block.coeffRef(i * n_dof_ + j, j) = 1.0;
        if (i > 0 && i < n_vars_ - 1)
          jac_block.coeffRef((i - 1) * n_dof_ + j, j) = -2.0;
        if (i > 1)
          jac_block.coeffRef((i - 2) * n_dof_ + j, j) = 1.0;
=======
        if (i < n_vars_ - 1)
          jac_block.coeffRef(i * n_dof_ + j, j) = 1.0;
        if (i > 0)
          jac_block.coeffRef((i - 1) * n_dof_ + j, j) = -1.0;
>>>>>>> add joint acceleration constraint
      }
    }
  }
}
}  // namespace trajopt
