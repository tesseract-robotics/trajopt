/**
 * @file continuous_collision_numerical_constraint.cpp
 * @brief The continuous collision numerical constraint
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

#include <trajopt_ifopt/constraints/collision/continuous_collision_numerical_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>

namespace trajopt_ifopt
{
ContinuousCollisionNumericalConstraint::ContinuousCollisionNumericalConstraint(
    std::shared_ptr<ContinuousCollisionEvaluator> collision_evaluator,
    std::array<std::shared_ptr<const Var>, 2> position_vars,
    bool vars0_fixed,
    bool vars1_fixed,
    int max_num_cnt,
    bool fixed_sparsity,
    std::string name)
  : ConstraintSet(std::move(name), max_num_cnt)
  , position_vars_(std::move(position_vars))
  , vars0_fixed_(vars0_fixed)
  , vars1_fixed_(vars1_fixed)
  , collision_evaluator_(std::move(collision_evaluator))
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

  coeffs_ = Eigen::VectorXd::Constant(rows_, 1);
  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(max_num_cnt), BoundSmallerZero);

  if (fixed_sparsity)
  {
    // Setting to zeros because snopt sparsity cannot change
    triplet_list_.reserve(static_cast<std::size_t>(bounds_.size()) *
                          static_cast<std::size_t>(position_vars_[0]->size()));

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(bounds_.size()); i++)  // NOLINT
      for (Eigen::Index j = 0; j < n_dof_; j++)
      {
        triplet_list_.emplace_back(i, position_vars_[0]->getIndex() + j, 0);
        triplet_list_.emplace_back(i, position_vars_[1]->getIndex() + j, 0);
      }
  }
}

int ContinuousCollisionNumericalConstraint::update()
{
  std::size_t variable_hash = position_vars_[0]->getParent()->getParent()->getHash();
  auto cache_data = collision_data_cache_.getOrAcquire(variable_hash);

  collision_data_ = cache_data.first;
  if (!cache_data.second)
  {
    collision_data_->contact_results_map.clear();
    collision_data_->gradient_results_sets.clear();
    collision_evaluator_->calcCollisionData(*collision_data_,
                                            position_vars_[0]->value(),
                                            position_vars_[1]->value(),
                                            vars0_fixed_,
                                            vars1_fixed_,
                                            bounds_.size());
    collision_data_cache_.put(variable_hash, collision_data_);
  }

  const auto cnt = std::min<std::size_t>(bounds_.size(), collision_data_->gradient_results_sets.size());
  for (std::size_t i = 0; i < cnt; ++i)
    coeffs_(static_cast<Eigen::Index>(i)) = collision_data_->gradient_results_sets[i].coeff;

  non_zeros_ = 2 * static_cast<Eigen::Index>(rows_) * n_dof_;
  return rows_;
}

Eigen::VectorXd ContinuousCollisionNumericalConstraint::getValues() const
{
  // Get current joint values
  const double margin_buffer = collision_evaluator_->getCollisionMarginBuffer();
  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  if (collision_data_->gradient_results_sets.empty())
    return values;

  const std::size_t cnt = std::min(bounds_.size(), collision_data_->gradient_results_sets.size());

  if (!vars0_fixed_ && !vars1_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
      values(static_cast<Eigen::Index>(i)) = r.getMaxError();
    }
  }
  else if (!vars0_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
      values(static_cast<Eigen::Index>(i)) = r.getMaxErrorT0();
    }
  }
  else
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
      values(static_cast<Eigen::Index>(i)) = r.getMaxErrorT1();
    }
  }

  return values;
}

// Set the limits on the constraint values
std::vector<Bounds> ContinuousCollisionNumericalConstraint::getBounds() const { return bounds_; }

Jacobian ContinuousCollisionNumericalConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  // Setting to zeros because snopt sparsity cannot change
  if (!triplet_list_.empty())                                         // NOLINT
    jac.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  if (collision_data_->gradient_results_sets.empty())
    return jac;

  const double margin_buffer = collision_evaluator_->getCollisionMarginBuffer();

  // Calculate collisions
  Eigen::VectorXd joint_vals0 = position_vars_[0]->value();
  const Eigen::VectorXd joint_vals1 = position_vars_[1]->value();

  trajopt_common::CollisionCacheData collision_data_delta;
  const std::size_t cnt = std::min(bounds_.size(), collision_data_->gradient_results_sets.size());
  const double delta = 1e-8;
  for (std::size_t p = 0; p < 2; p++)
  {
    Eigen::VectorXd jv = position_vars_[p]->value();
    for (int j = 0; j < n_dof_; j++)
    {
      jv(j) += delta;
      collision_evaluator_->calcCollisionData(
          collision_data_delta, jv, joint_vals1, vars0_fixed_, vars1_fixed_, bounds_.size());

      for (int i = 0; i < static_cast<int>(cnt); ++i)
      {
        const auto& baseline = collision_data_->gradient_results_sets[static_cast<std::size_t>(i)];
        auto fn = [&baseline](const trajopt_common::GradientResultsSet& cr) {
          return (cr.key == baseline.key && cr.shape_key == baseline.shape_key);
        };
        auto it = std::find_if(
            collision_data_delta.gradient_results_sets.begin(), collision_data_delta.gradient_results_sets.end(), fn);
        if (it != collision_data_delta.gradient_results_sets.end())
        {
          double dist_delta{ 0 };
          if (!vars0_fixed_ && !vars1_fixed_)
            dist_delta = (it->getMaxError() - baseline.getMaxError());
          else if (!vars0_fixed_)
            dist_delta = (it->getMaxErrorT0() - baseline.getMaxErrorT0());
          else
            dist_delta = (it->getMaxErrorT1() - baseline.getMaxErrorT1());

          jac.coeffRef(i, position_vars_[p]->getIndex() + j) = dist_delta / delta;
        }
        else
        {
          double dist_delta{ 0 };
          if (!vars0_fixed_ && !vars1_fixed_)
            dist_delta = ((-1.0 * margin_buffer) - baseline.getMaxError());
          else if (!vars0_fixed_)
            dist_delta = ((-1.0 * margin_buffer) - baseline.getMaxErrorT0());
          else
            dist_delta = ((-1.0 * margin_buffer) - baseline.getMaxErrorT1());

          jac.coeffRef(i, position_vars_[p]->getIndex() + j) = dist_delta / delta;
        }
      }
      jv = position_vars_[p]->value();
    }
  }
  return jac;
}

Eigen::VectorXd ContinuousCollisionNumericalConstraint::getCoefficients() const { return coeffs_; }

void ContinuousCollisionNumericalConstraint::setBounds(const std::vector<Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

std::shared_ptr<ContinuousCollisionEvaluator> ContinuousCollisionNumericalConstraint::getCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
