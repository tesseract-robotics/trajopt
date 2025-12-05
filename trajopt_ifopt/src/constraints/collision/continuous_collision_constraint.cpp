/**
 * @file continuous_collision_constraint.cpp
 * @brief The continuous collision position constraint
 *
 * @author Levi Armstrong
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

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <trajopt_common/collision_types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

namespace trajopt_ifopt
{
ContinuousCollisionConstraint::ContinuousCollisionConstraint(
    std::shared_ptr<ContinuousCollisionEvaluator> collision_evaluator,
    std::array<std::shared_ptr<const Var>, 2> position_vars,
    bool vars0_fixed,
    bool vars1_fixed,
    int max_num_cnt,
    bool fixed_sparsity,
    const std::string& name)
  : ifopt::ConstraintSet(max_num_cnt, name)
  , position_vars_(std::move(position_vars))
  , vars0_fixed_(vars0_fixed)
  , vars1_fixed_(vars1_fixed)
  , collision_evaluator_(std::move(collision_evaluator))
  , fixed_sparsity_(fixed_sparsity)
{
  if (position_vars_[0] == nullptr && position_vars_[1] == nullptr)
    throw std::runtime_error("position_vars contains a nullptr!");

  // Set n_dof_ for convenience
  n_dof_ = position_vars_[0]->size();
  if (!(n_dof_ > 0))
    throw std::runtime_error("position_vars[0] is empty!");

  if (position_vars_[0]->size() != position_vars_[1]->size())
    throw std::runtime_error("position_vars are not the same size!");

  if (vars0_fixed_ && vars1_fixed_)
    throw std::runtime_error("position_vars are both fixed!");

  if (max_num_cnt < 1)
    throw std::runtime_error("max_num_cnt must be greater than zero!");

  bounds_ = std::vector<ifopt::Bounds>(static_cast<std::size_t>(max_num_cnt), ifopt::BoundSmallerZero);
}

Eigen::VectorXd ContinuousCollisionConstraint::GetValues() const
{
  const double margin_buffer = collision_evaluator_->GetCollisionMarginBuffer();
  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  auto collision_data = collision_evaluator_->CalcCollisionData(
      position_vars_[0]->value(), position_vars_[1]->value(), vars0_fixed_, vars1_fixed_, bounds_.size());

  if (collision_data->gradient_results_sets.empty())
    return values;

  const std::size_t cnt = std::min(collision_data->gradient_results_sets.size(), bounds_.size());
  if (!vars0_fixed_ && !vars1_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
      values(static_cast<Eigen::Index>(i)) = r.coeff * r.getMaxError();
    }
  }
  else if (!vars0_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
      if (r.max_error[0].has_error[0] || r.max_error[1].has_error[0])
        values(static_cast<Eigen::Index>(i)) = r.coeff * r.getMaxErrorT0();
    }
  }
  else
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
      if (r.max_error[0].has_error[1] || r.max_error[1].has_error[1])
        values(static_cast<Eigen::Index>(i)) = r.coeff * r.getMaxErrorT1();
    }
  }

  return values;
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> ContinuousCollisionConstraint::GetBounds() const { return bounds_; }

void ContinuousCollisionConstraint::initSparsity() const
{
  if (!fixed_sparsity_)
    return;

  // Setting to zeros because snopt sparsity cannot change
  triplet_list_.reserve(static_cast<std::size_t>(bounds_.size()) * static_cast<std::size_t>(position_vars_[0]->size()));

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(bounds_.size()); i++)  // NOLINT
  {
    for (Eigen::Index j = 0; j < n_dof_; j++)
    {
      triplet_list_.emplace_back(i, position_vars_[0]->getIndex() + j, 0);
      triplet_list_.emplace_back(i, position_vars_[1]->getIndex() + j, 0);
    }
  }
}

void ContinuousCollisionConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set != position_vars_.front()->getParent()->getParent()->GetName())  // NOLINT
    return;

  // Setting to zeros because snopt sparsity cannot change
  std::call_once(init_flag_, &ContinuousCollisionConstraint::initSparsity, this);

  // Setting to zeros because snopt sparsity cannot change
  if (!triplet_list_.empty())                                               // NOLINT
    jac_block.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  // Calculate collisions
  auto collision_data = collision_evaluator_->CalcCollisionData(
      position_vars_[0]->value(), position_vars_[1]->value(), vars0_fixed_, vars1_fixed_, bounds_.size());

  if (collision_data->gradient_results_sets.empty())
    return;

  /** @todo Should use triplet list and setFromTriplets */
  const std::size_t cnt = std::min(collision_data->gradient_results_sets.size(), bounds_.size());
  if (!vars0_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
      if (r.max_error[0].has_error[0] || r.max_error[1].has_error[0])
      {
        double max_error_with_buffer = r.getMaxErrorWithBufferT0();
        if (!vars1_fixed_)
          max_error_with_buffer = r.getMaxErrorWithBuffer();

        Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(r, max_error_with_buffer, position_vars_[0]->size());

        // Collision is 1 x n_dof
        for (int j = 0; j < n_dof_; j++)
          jac_block.coeffRef(static_cast<int>(i), position_vars_[0]->getIndex() + j) = -1.0 * grad_vec[j];
      }
    }
  }

  if (!vars1_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
      if (r.max_error[0].has_error[1] || r.max_error[1].has_error[1])
      {
        double max_error_with_buffer = r.getMaxErrorWithBufferT1();
        if (!vars0_fixed_)
          max_error_with_buffer = r.getMaxErrorWithBuffer();

        Eigen::VectorXd grad_vec = getWeightedAvgGradientT1(r, max_error_with_buffer, position_vars_[1]->size());

        // Collision is 1 x n_dof
        for (int j = 0; j < n_dof_; j++)
          jac_block.coeffRef(static_cast<int>(i), position_vars_[1]->getIndex() + j) = -1.0 * grad_vec[j];
      }
    }
  }
}

void ContinuousCollisionConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

std::shared_ptr<ContinuousCollisionEvaluator> ContinuousCollisionConstraint::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
