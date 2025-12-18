/**
 * @file discrete_collision_constraint.cpp
 * @brief The single timestep collision position constraint
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

#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

#include <trajopt_ifopt/utils/numeric_differentiation.h>

namespace trajopt_ifopt
{
DiscreteCollisionConstraint::DiscreteCollisionConstraint(
    std::shared_ptr<DiscreteCollisionEvaluator> collision_evaluator,
    std::shared_ptr<const Var> position_var,
    int max_num_cnt,
    bool fixed_sparsity,
    std::string name)
  : DiscreteCollisionConstraint(std::move(collision_evaluator),
                                std::vector<std::shared_ptr<const Var>>{ std::move(position_var) },
                                max_num_cnt,
                                fixed_sparsity,
                                std::move(name))
{
}

DiscreteCollisionConstraint::DiscreteCollisionConstraint(
    std::shared_ptr<DiscreteCollisionEvaluator> collision_evaluator,
    std::vector<std::shared_ptr<const Var>> position_vars,
    int max_num_cnt,
    bool fixed_sparsity,
    std::string name)
  : ConstraintSet(std::move(name), max_num_cnt * static_cast<int>(position_vars.size()))
  , position_vars_(std::move(position_vars))
  , collision_evaluator_(std::move(collision_evaluator))
  , fixed_sparsity_(fixed_sparsity)
{
  if (position_vars_.empty())
    throw std::runtime_error("DiscreteCollisionConstraint: position_vars must not be empty");

  if (max_num_cnt < 1)
    throw std::runtime_error("max_num_cnt must be greater than zero!");

  max_num_cnt_per_var_ = static_cast<std::size_t>(max_num_cnt);

  // Total DOFs = sum of sizes (no contiguity assumption)
  n_dof_ = 0;
  for (const auto& v : position_vars_)
    n_dof_ += v->size();

  assert(n_dof_ > 0);

  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(GetRows()), BoundSmallerZero);
}

Eigen::VectorXd DiscreteCollisionConstraint::GetValues() const
{
  const double margin_buffer = collision_evaluator_->GetCollisionMarginBuffer();

  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  std::size_t offset = 0;
  for (const auto& v : position_vars_)
  {
    const trajopt_common::CollisionCacheData::ConstPtr collision_data =
        collision_evaluator_->CalcCollisions(v->value(), max_num_cnt_per_var_);
    if (collision_data->gradient_results_sets.empty())
    {
      offset += max_num_cnt_per_var_;
      continue;
    }

    const std::size_t cnt = std::min<std::size_t>(max_num_cnt_per_var_, collision_data->gradient_results_sets.size());
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
      values(static_cast<Eigen::Index>(offset + i)) = r.coeff * r.getMaxErrorT0();
    }

    offset += max_num_cnt_per_var_;
  }

  return values;
}

// Set the limits on the constraint values
std::vector<Bounds> DiscreteCollisionConstraint::GetBounds() const { return bounds_; }

void DiscreteCollisionConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  std::call_once(init_flag_, &DiscreteCollisionConstraint::init, this);

  // Only modify the jacobian if this constraint uses var_set
  if (var_set != var_set_name_)  // NOLINT
    return;

  // Setting to zeros because SNOPT sparsity cannot change
  if (!triplet_list_.empty())                                               // NOLINT
    jac_block.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  for (std::size_t idx = 0; idx < position_vars_.size(); ++idx)
  {
    const auto& v = position_vars_[idx];
    const trajopt_common::CollisionCacheData::ConstPtr collision_data =
        collision_evaluator_->CalcCollisions(v->value(), max_num_cnt_per_var_);

    if (collision_data->gradient_results_sets.empty())
      continue;

    const std::size_t cnt = std::min<std::size_t>(max_num_cnt_per_var_, collision_data->gradient_results_sets.size());
    const std::size_t row_offset = idx * max_num_cnt_per_var_;

    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
      Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(r, r.getMaxErrorWithBufferT0(), v->size());

      const Eigen::Index base_idx = v->getIndex();
      const Eigen::Index sz = v->size();
      const int row = static_cast<int>(row_offset + i);

      for (Eigen::Index j = 0; j < sz; ++j)
        jac_block.coeffRef(row, static_cast<int>(base_idx + j)) = -1.0 * grad_vec[j];
    }
  }
}

void DiscreteCollisionConstraint::SetBounds(const std::vector<Bounds>& bounds)
{
  assert(bounds.size() == GetRows());
  bounds_ = bounds;
}

void DiscreteCollisionConstraint::init() const
{
  const auto* parent_set = position_vars_.front()->getParent()->getParent();
  var_set_name_ = parent_set->GetName();
  for (const auto& v : position_vars_)
  {
    const auto* v_parent_set = v->getParent()->getParent();
    if (v_parent_set != parent_set)
      throw std::runtime_error("DiscreteCollisionConstraint: all vars must belong to the same variable set");
  }

  if (!fixed_sparsity_)
    return;

  const Eigen::Index n_rows = GetRows();
  triplet_list_.clear();
  triplet_list_.reserve(static_cast<std::size_t>(n_rows) * static_cast<std::size_t>(n_dof_));

  for (Eigen::Index i = 0; i < n_rows; ++i)
  {
    for (const auto& v : position_vars_)
    {
      const Eigen::Index base_idx = v->getIndex();
      const Eigen::Index sz = v->size();
      for (Eigen::Index j = 0; j < sz; ++j)
        triplet_list_.emplace_back(i, base_idx + j, 0.0);
    }
  }
}

std::shared_ptr<DiscreteCollisionEvaluator> DiscreteCollisionConstraint::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
