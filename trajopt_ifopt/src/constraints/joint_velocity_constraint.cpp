/**
 * @file joint_velocity_constraint.h
 * @brief The joint_velocity constraint
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
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

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
JointVelConstraint::JointVelConstraint(const Eigen::VectorXd& targets,
                                       const std::vector<JointPosition::ConstPtr>& position_vars,
                                       const Eigen::VectorXd& coeffs,
                                       const std::string& name)
  : ifopt::ConstraintSet(static_cast<int>(targets.size()) * static_cast<int>(position_vars.size() - 1), name)
  , coeffs_(coeffs)
  , position_vars_(position_vars)
{
  if (position_vars_.size() < 2)
    throw std::runtime_error("JointVelConstraint, requires minimum of three position variables!");

  // Check and make sure the targets size aligns with the vars passed in
  for (const auto& position_var : position_vars_)
  {
    if (targets.size() != position_var->GetRows())
      CONSOLE_BRIDGE_logError("Targets size does not align with variables provided");
  }

  // Set n_dof and n_vars
  n_dof_ = targets.size();
  n_vars_ = static_cast<long>(position_vars_.size());
  assert(n_dof_ > 0);
  assert(n_vars_ > 0);
  //  assert(n_vars_ == 2);

  if (!(coeffs_.array() > 0).all())
    throw std::runtime_error("JointVelConstraint, coeff must be greater than zero.");

  if (coeffs_.rows() == 1)
    coeffs_ = Eigen::VectorXd::Constant(n_dof_, coeffs(0));

  if (coeffs_.rows() != n_dof_)
    throw std::runtime_error("JointVelConstraint, coeff must be the same size of the joint position.");

  // Set the bounds to the input targets
  std::vector<ifopt::Bounds> bounds(static_cast<size_t>(GetRows()));
  // All of the positions should be exactly at their targets
  for (long j = 0; j < n_vars_ - 1; j++)
  {
    index_map_[position_vars_[static_cast<std::size_t>(j)]->GetName()] = j;
    for (long i = 0; i < n_dof_; i++)
    {
      bounds[static_cast<size_t>(i + j * n_dof_)] = ifopt::Bounds(targets[i], targets[i]);
    }
  }
  index_map_[position_vars_.back()->GetName()] = (n_vars_ - 1);
  bounds_ = bounds;
}

Eigen::VectorXd JointVelConstraint::GetValues() const
{
  // i - represents the trajectory timestep index
  // k - represents the DOF index
  // var[i, k] - represents the variable index
  // vel(var[0, 0]) - represents the joint velocity of DOF index 0 at timestep 0
  // vel(var[1, 1]) - represents the joint velocity of DOF index 1 at timestep 1
  //
  // Velocity V = vel(var[0, 0]), vel(var[0, 1]), vel(var[0, 2]), vel(var[1, 0]), vel(var[1, 1]), vel(var[1, 2]), etc
  Eigen::VectorXd velocity(static_cast<size_t>(n_dof_) * (static_cast<size_t>(n_vars_) - 1));

  // Forward differentiation for the first point
  for (std::size_t ind = 0; ind < position_vars_.size() - 1; ind++)
  {
    auto vals1 = this->GetVariables()->GetComponent(position_vars_[ind]->GetName())->GetValues();
    auto vals2 = this->GetVariables()->GetComponent(position_vars_[ind + 1]->GetName())->GetValues();
    Eigen::VectorXd single_step = (vals2 - vals1);
    velocity.block(n_dof_ * static_cast<Eigen::Index>(ind), 0, n_dof_, 1) = coeffs_.cwiseProduct(single_step);
  }

  return velocity;
}

// Set the limits on the constraint values (in this case just the targets)
std::vector<ifopt::Bounds> JointVelConstraint::GetBounds() const { return bounds_; }

void JointVelConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  //  FillJacobianBlockNumerical(var_set, jac_block);

  // Check if this constraint use the var_set
  // Only modify the jacobian if this constraint uses var_set
  auto it = index_map_.find(var_set);
  if (it == index_map_.end())  // NOLINT
    return;

  Eigen::Index i = it->second;

  // Reserve enough room in the sparse matrix
  std::vector<Eigen::Triplet<double> > triplet_list;
  triplet_list.reserve(static_cast<std::size_t>(3 * n_dof_));

  for (int j = 0; j < n_dof_; j++)
  {
    // The first and last variable are special and only effect the first and last constraint. Everything else
    // effects 2
    if (i < n_vars_ - 1)
      triplet_list.emplace_back((i * n_dof_) + j, j, -1.0 * coeffs_[j]);

    if (i > 0)
      triplet_list.emplace_back(((i - 1) * n_dof_) + j, j, 1.0 * coeffs_[j]);
  }
  jac_block.setFromTriplets(triplet_list.begin(), triplet_list.end());  // NOLINT
}

}  // namespace trajopt_ifopt
