/**
 * @file discrete_collision_constraint.cpp
 * @brief The single timestep collision position constraint
 *
 * @author Levi Armstrong
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

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_utils.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

namespace trajopt_ifopt
{
DiscreteCollisionConstraint::DiscreteCollisionConstraint(DiscreteCollisionEvaluator::Ptr collision_evaluator,
                                                         JointPosition::ConstPtr position_var,
                                                         int max_num_cnt,
                                                         bool fixed_sparsity,
                                                         const std::string& name)
  : ifopt::ConstraintSet(max_num_cnt, name)
  , position_var_(std::move(position_var))
  , collision_evaluator_(std::move(collision_evaluator))
{
  // Set n_dof_ for convenience
  n_dof_ = position_var_->GetRows();
  assert(n_dof_ > 0);

  if (max_num_cnt < 1)
    throw std::runtime_error("max_num_cnt must be greater than zero!");

  bounds_ = std::vector<ifopt::Bounds>(static_cast<std::size_t>(max_num_cnt), ifopt::BoundSmallerZero);

  // Setting to zeros because snopt sparsity cannot change
  if (fixed_sparsity)
  {
    triplet_list_.reserve(static_cast<std::size_t>(bounds_.size()) *
                          static_cast<std::size_t>(position_var_->GetRows()));
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(bounds_.size()); i++)
      for (Eigen::Index j = 0; j < n_dof_; j++)
        triplet_list_.emplace_back(i, j, 0);
  }
}

Eigen::VectorXd DiscreteCollisionConstraint::GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

  return CalcValues(joint_vals);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> DiscreteCollisionConstraint::GetBounds() const { return bounds_; }

void DiscreteCollisionConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set != position_var_->GetName())  // NOLINT
    return;

  // Get current joint values
  VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

  CalcJacobianBlock(joint_vals, jac_block);  // NOLINT
}

Eigen::VectorXd DiscreteCollisionConstraint::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  // Check the collisions
  trajopt_common::CollisionCacheData::ConstPtr collision_data =
      collision_evaluator_->CalcCollisions(joint_vals, bounds_.size());
  double margin_buffer = collision_evaluator_->GetCollisionConfig().collision_margin_buffer;
  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  if (collision_data->gradient_results_sets.empty())
    return values;

  const std::size_t cnt = std::min(bounds_.size(), collision_data->gradient_results_sets.size());
  for (std::size_t i = 0; i < cnt; ++i)
  {
    const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
    values(static_cast<Eigen::Index>(i)) = r.coeff * r.getMaxErrorT0();
  }

  return values;
}

void DiscreteCollisionConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void DiscreteCollisionConstraint::CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                    Jacobian& jac_block) const
{
  // Setting to zeros because snopt sparsity cannot change
  if (!triplet_list_.empty())                                               // NOLINT
    jac_block.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  trajopt_common::CollisionCacheData::ConstPtr collision_data =
      collision_evaluator_->CalcCollisions(joint_vals, bounds_.size());
  if (collision_data->gradient_results_sets.empty())
    return;

  /** @todo Probably should use a triplet list and setFromTriplets */
  const std::size_t cnt = std::min(bounds_.size(), collision_data->gradient_results_sets.size());
  for (std::size_t i = 0; i < cnt; ++i)
  {
    const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
    Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(r, r.getMaxErrorWithBufferT0(), position_var_->GetRows());

    // Collision is 1 x n_dof
    for (int j = 0; j < n_dof_; j++)
      jac_block.coeffRef(static_cast<int>(i), j) = -1.0 * grad_vec[j];
  }
}

DiscreteCollisionEvaluator::Ptr DiscreteCollisionConstraint::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
