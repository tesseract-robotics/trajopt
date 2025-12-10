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
  : ContinuousCollisionConstraint(
        std::move(collision_evaluator),
        std::vector<std::shared_ptr<const Var>>{ std::move(position_vars[0]), std::move(position_vars[1]) },
        vars0_fixed,
        vars1_fixed,
        max_num_cnt,
        fixed_sparsity,
        name)
{
}

ContinuousCollisionConstraint::ContinuousCollisionConstraint(
    std::shared_ptr<ContinuousCollisionEvaluator> collision_evaluator,
    std::vector<std::shared_ptr<const Var>> position_vars,
    bool vars0_fixed,
    bool vars1_fixed,
    int max_num_cnt,
    bool fixed_sparsity,
    const std::string& name)
  : ifopt::ConstraintSet(max_num_cnt * static_cast<int>(position_vars.size() > 1 ? (position_vars.size() - 1) : 0),
                         name)
  , position_vars_(std::move(position_vars))
  , vars0_fixed_(vars0_fixed)
  , vars1_fixed_(vars1_fixed)
  , collision_evaluator_(std::move(collision_evaluator))
  , fixed_sparsity_(fixed_sparsity)
{
  if (position_vars_.size() < 2)
    throw std::runtime_error("ContinuousCollisionConstraint: position_vars must contain at least two variables for "
                             "continuous collision");

  for (const auto& v : position_vars_)
  {
    if (v == nullptr)
      throw std::runtime_error("ContinuousCollisionConstraint: position_vars contains a nullptr!");
  }

  // All vars must have same size
  n_dof_ = position_vars_.front()->size();
  if (n_dof_ <= 0)
    throw std::runtime_error("ContinuousCollisionConstraint: first position var is empty!");

  for (const auto& v : position_vars_)
  {
    if (v->size() != n_dof_)
      throw std::runtime_error("ContinuousCollisionConstraint: all position_vars must have same size!");
  }

  if (position_vars_.size() == 2 && vars0_fixed_ && vars1_fixed_)
    throw std::runtime_error("ContinuousCollisionConstraint: when number vars is two, both ends cannot be fixed!");

  if (max_num_cnt < 1)
    throw std::runtime_error("ContinuousCollisionConstraint: max_num_cnt must be greater than zero!");

  max_num_cnt_per_segment_ = static_cast<std::size_t>(max_num_cnt);

  const std::size_t num_segments = position_vars_.size() - 1;
  const std::size_t total_rows = max_num_cnt_per_segment_ * num_segments;

  if (static_cast<int>(total_rows) != GetRows())
    throw std::runtime_error("ContinuousCollisionConstraint: internal row count mismatch");

  bounds_ = std::vector<ifopt::Bounds>(total_rows, ifopt::BoundSmallerZero);
}

Eigen::VectorXd ContinuousCollisionConstraint::GetValues() const
{
  const double margin_buffer = collision_evaluator_->GetCollisionMarginBuffer();
  Eigen::VectorXd values = Eigen::VectorXd::Constant(GetRows(), -margin_buffer);

  const std::size_t num_segments = position_vars_.size() - 1;

  for (std::size_t seg = 0; seg < num_segments; ++seg)
  {
    const auto& var0 = position_vars_[seg];
    const auto& var1 = position_vars_[seg + 1];

    const bool seg_vars0_fixed = (seg == 0) ? vars0_fixed_ : false;
    const bool seg_vars1_fixed = (seg == num_segments - 1) ? vars1_fixed_ : false;

    auto collision_data = collision_evaluator_->CalcCollisionData(
        var0->value(), var1->value(), seg_vars0_fixed, seg_vars1_fixed, max_num_cnt_per_segment_);

    if (collision_data->gradient_results_sets.empty())
      continue;

    const std::size_t row_offset = seg * max_num_cnt_per_segment_;
    const std::size_t cnt =
        std::min<std::size_t>(max_num_cnt_per_segment_, collision_data->gradient_results_sets.size());

    if (!seg_vars0_fixed && !seg_vars1_fixed)
    {
      for (std::size_t i = 0; i < cnt; ++i)
      {
        const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
        values(static_cast<Eigen::Index>(row_offset + i)) = r.coeff * r.getMaxError();
      }
    }
    else if (!seg_vars0_fixed)
    {
      for (std::size_t i = 0; i < cnt; ++i)
      {
        const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
        if (r.max_error[0].has_error[0] || r.max_error[1].has_error[0])
          values(static_cast<Eigen::Index>(row_offset + i)) = r.coeff * r.getMaxErrorT0();
      }
    }
    else  // !seg_vars1_fixed
    {
      for (std::size_t i = 0; i < cnt; ++i)
      {
        const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
        if (r.max_error[0].has_error[1] || r.max_error[1].has_error[1])
          values(static_cast<Eigen::Index>(row_offset + i)) = r.coeff * r.getMaxErrorT1();
      }
    }
  }

  return values;
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> ContinuousCollisionConstraint::GetBounds() const { return bounds_; }

void ContinuousCollisionConstraint::init() const
{
  const auto* parent_set0 = position_vars_.front()->getParent()->getParent();
  if (parent_set0 == nullptr)
    throw std::runtime_error("ContinuousCollisionConstraint: invalid variable parent");

  for (const auto& v : position_vars_)
  {
    const auto* ps = v->getParent()->getParent();
    if (ps != parent_set0)
      throw std::runtime_error("ContinuousCollisionConstraint: all vars must belong to the same variable set");
  }

  var_set_name_ = parent_set0->GetName();

  if (!fixed_sparsity_)
    return;

  triplet_list_.clear();

  const std::size_t num_segments = position_vars_.size() - 1;
  const Eigen::Index total_rows = GetRows();

  triplet_list_.reserve(static_cast<std::size_t>(total_rows) * static_cast<std::size_t>(2 * n_dof_));

  for (std::size_t seg = 0; seg < num_segments; ++seg)
  {
    const auto& var0 = position_vars_[seg];
    const auto& var1 = position_vars_[seg + 1];

    const Eigen::Index base_idx0 = var0->getIndex();
    const Eigen::Index base_idx1 = var1->getIndex();

    const std::size_t row_base = seg * max_num_cnt_per_segment_;
    for (int k = 0; k < max_num_cnt_per_segment_; ++k)
    {
      const auto row = static_cast<Eigen::Index>(row_base + static_cast<std::size_t>(k));
      for (Eigen::Index j = 0; j < n_dof_; ++j)
      {
        triplet_list_.emplace_back(row, base_idx0 + j, 0.0);
        triplet_list_.emplace_back(row, base_idx1 + j, 0.0);
      }
    }
  }
}

void ContinuousCollisionConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  std::call_once(init_flag_, &ContinuousCollisionConstraint::init, this);

  // Only modify the jacobian if this constraint uses var_set
  if (var_set != var_set_name_)  // NOLINT
    return;

  // Setting to zeros because SNOPT sparsity cannot change
  if (!triplet_list_.empty())                                               // NOLINT
    jac_block.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  const std::size_t num_segments = position_vars_.size() - 1;

  for (std::size_t seg = 0; seg < num_segments; ++seg)
  {
    const auto& var0 = position_vars_[seg];
    const auto& var1 = position_vars_[seg + 1];

    const bool seg_vars0_fixed = (seg == 0) ? vars0_fixed_ : false;
    const bool seg_vars1_fixed = (seg == num_segments - 1) ? vars1_fixed_ : false;

    auto collision_data = collision_evaluator_->CalcCollisionData(
        var0->value(), var1->value(), seg_vars0_fixed, seg_vars1_fixed, max_num_cnt_per_segment_);

    if (collision_data->gradient_results_sets.empty())
      continue;

    const std::size_t row_offset = seg * max_num_cnt_per_segment_;
    const std::size_t cnt =
        std::min<std::size_t>(max_num_cnt_per_segment_, collision_data->gradient_results_sets.size());

    // T0 contributions
    if (!seg_vars0_fixed)
    {
      const Eigen::Index base_idx0 = var0->getIndex();
      for (std::size_t i = 0; i < cnt; ++i)
      {
        const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
        if (r.max_error[0].has_error[0] || r.max_error[1].has_error[0])
        {
          double max_error_with_buffer = r.getMaxErrorWithBufferT0();
          if (!seg_vars1_fixed)
            max_error_with_buffer = r.getMaxErrorWithBuffer();

          Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(r, max_error_with_buffer, n_dof_);

          const int row = static_cast<int>(row_offset + i);
          for (Eigen::Index j = 0; j < n_dof_; ++j)
            jac_block.coeffRef(row, static_cast<int>(base_idx0 + j)) = -1.0 * grad_vec[j];
        }
      }
    }

    // T1 contributions
    if (!seg_vars1_fixed)
    {
      const Eigen::Index base_idx1 = var1->getIndex();
      for (std::size_t i = 0; i < cnt; ++i)
      {
        const trajopt_common::GradientResultsSet& r = collision_data->gradient_results_sets[i];
        if (r.max_error[0].has_error[1] || r.max_error[1].has_error[1])
        {
          double max_error_with_buffer = r.getMaxErrorWithBufferT1();
          if (!seg_vars0_fixed)
            max_error_with_buffer = r.getMaxErrorWithBuffer();

          Eigen::VectorXd grad_vec = getWeightedAvgGradientT1(r, max_error_with_buffer, n_dof_);

          const int row = static_cast<int>(row_offset + i);
          for (Eigen::Index j = 0; j < n_dof_; ++j)
            jac_block.coeffRef(row, static_cast<int>(base_idx1 + j)) = -1.0 * grad_vec[j];
        }
      }
    }
  }
}

void ContinuousCollisionConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == GetRows());
  bounds_ = bounds;
}

std::shared_ptr<ContinuousCollisionEvaluator> ContinuousCollisionConstraint::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
