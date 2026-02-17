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
#include <trajopt_common/collision_utils.h>
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
  : ConstraintSet(std::move(name), max_num_cnt)
  , position_var_(std::move(position_var))
  , collision_evaluator_(std::move(collision_evaluator))
  , fixed_sparsity_(fixed_sparsity)
{
  // Set n_dof_ for convenience
  n_dof_ = position_var_->size();
  assert(n_dof_ > 0);

  if (max_num_cnt < 1)
    throw std::runtime_error("max_num_cnt must be greater than zero!");

  coeffs_ = Eigen::VectorXd::Constant(rows_, 1);
  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(max_num_cnt), BoundSmallerZero);
  non_zeros_ = rows_ * n_dof_;
}

int DiscreteCollisionConstraint::update()
{
  std::size_t variable_hash = position_var_->getParent()->getParent()->getHash();
  auto cache_data = collision_data_cache_.getOrAcquire(variable_hash);

  collision_data_ = cache_data.first;
  if (!cache_data.second)
  {
    collision_data_->contact_results_map.clear();
    collision_data_->gradient_results_sets.clear();
    collision_evaluator_->calcCollisions(*collision_data_, position_var_->value(), bounds_.size());
    collision_data_cache_.put(variable_hash, collision_data_);
  }

  const auto cnt = std::min<std::size_t>(bounds_.size(), collision_data_->gradient_results_sets.size());
  for (std::size_t i = 0; i < cnt; ++i)
    coeffs_(static_cast<Eigen::Index>(i)) = collision_data_->gradient_results_sets[i].coeff;

  std::call_once(init_flag_, &DiscreteCollisionConstraint::init, this);

  return rows_;
}

Eigen::VectorXd DiscreteCollisionConstraint::getValues() const
{
  // Check the collisions
  const double margin_buffer = collision_evaluator_->getCollisionMarginBuffer();

  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  if (collision_data_->gradient_results_sets.empty())
    return values;

  const std::size_t cnt = std::min(bounds_.size(), collision_data_->gradient_results_sets.size());
  for (std::size_t i = 0; i < cnt; ++i)
  {
    const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
    values(static_cast<Eigen::Index>(i)) = r.getMaxErrorT0();
  }

  return values;
}

Eigen::VectorXd DiscreteCollisionConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values
std::vector<Bounds> DiscreteCollisionConstraint::getBounds() const { return bounds_; }

Jacobian DiscreteCollisionConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  // Setting to zeros because SNOPT sparsity cannot change
  if (!triplet_list_.empty())                                         // NOLINT
    jac.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  if (collision_data_->gradient_results_sets.empty())
    return jac;

  /** @todo Probably should use a triplet list and setFromTriplets */
  const std::size_t cnt = std::min(bounds_.size(), collision_data_->gradient_results_sets.size());
  for (std::size_t i = 0; i < cnt; ++i)
  {
    const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
    Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(r, r.getMaxErrorWithBufferT0(), position_var_->size());

    // Collision is 1 x n_dof
    for (int j = 0; j < n_dof_; j++)
      jac.coeffRef(static_cast<int>(i), position_var_->getIndex() + j) = -1.0 * grad_vec[j];
  }

  return jac;
}

void DiscreteCollisionConstraint::setBounds(const std::vector<Bounds>& bounds)
{
  assert(bounds.size() == rows_);
  bounds_ = bounds;
}

void DiscreteCollisionConstraint::init() const
{
  if (!fixed_sparsity_)
    return;

  triplet_list_.reserve(static_cast<std::size_t>(bounds_.size()) * static_cast<std::size_t>(position_var_->size()));
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(bounds_.size()); i++)
    for (Eigen::Index j = 0; j < n_dof_; j++)
      triplet_list_.emplace_back(i, position_var_->getIndex() + j, 0);
}

std::shared_ptr<DiscreteCollisionEvaluator> DiscreteCollisionConstraint::getCollisionEvaluator() const
{
  return collision_evaluator_;
}

DiscreteCollisionConstraintD::DiscreteCollisionConstraintD(
    std::shared_ptr<DiscreteCollisionEvaluator> collision_evaluator,
    std::shared_ptr<const Var> position_var,
    std::string name)
  : ConstraintSet(std::move(name), true)
  , position_var_(std::move(position_var))
  , collision_evaluator_(std::move(collision_evaluator))
{
  n_dof_ = position_var_->size();
  assert(n_dof_ > 0);
}

int DiscreteCollisionConstraintD::update()
{
  std::size_t variable_hash = position_var_->getParent()->getParent()->getHash();
  auto cache_data = collision_data_cache_.getOrAcquire(variable_hash);

  if (cache_data.second)
  {
    collision_data_ = cache_data.first;
  }
  else
  {
    cache_data.first->contact_results_map.clear();
    cache_data.first->gradient_results_sets.clear();
    collision_evaluator_->calcCollisions(*cache_data.first, position_var_->value(), 0);
    collision_data_ = cache_data.first;
    collision_data_cache_.put(variable_hash, cache_data.first);
  }

  rows_ = static_cast<int>(collision_data_->contact_results_map.count());
  non_zeros_ = rows_ * n_dof_;
  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(rows_), BoundSmallerZero);

  coeffs_.resize(rows_);
  values_.resize(rows_);

  Eigen::Index i{ 0 };
  for (const auto& pair : collision_data_->contact_results_map)
  {
    const double margin =
        collision_evaluator_->getCollisionMarginData().getCollisionMargin(pair.first.first, pair.first.second);
    const double coeff =
        collision_evaluator_->getCollisionCoeffData().getCollisionCoeff(pair.first.first, pair.first.second);
    for (const auto& contact_results : pair.second)
    {
      coeffs_(i) = coeff;
      values_(i++) = margin - contact_results.distance;
    }
  }

  assert(rows_ == i);

  return rows_;
}

Eigen::VectorXd DiscreteCollisionConstraintD::getValues() const { return values_; }

Eigen::VectorXd DiscreteCollisionConstraintD::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values
std::vector<Bounds> DiscreteCollisionConstraintD::getBounds() const { return bounds_; }

Jacobian DiscreteCollisionConstraintD::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  if (collision_data_->contact_results_map.empty())
    return jac;

  auto jp = position_var_->value();

  trajopt_common::GradientResults result;
  Eigen::Index i{ 0 };
  for (const auto& pair : collision_data_->contact_results_map)
  {
    for (const auto& contact_results : pair.second)
    {
      result.clear();
      trajopt_common::getGradient(result, jp, contact_results, 0, 0, collision_evaluator_->getJointGroup());
      jac.startVec(i);
      assert(result.gradients[0].has_gradient || result.gradients[1].has_gradient);
      if (result.gradients[0].has_gradient && result.gradients[1].has_gradient)
      {
        // This does work but could be faster
        for (int j = 0; j < n_dof_; j++)
          jac.insertBack(i, position_var_->getIndex() + j) =
              -1.0 * (result.gradients[0].gradient[j] + result.gradients[1].gradient[j]);
      }
      else if (result.gradients[0].has_gradient)
      {
        // This does work but could be faster
        for (int j = 0; j < n_dof_; j++)
          jac.insertBack(i, position_var_->getIndex() + j) = -1.0 * result.gradients[0].gradient[j];
      }
      else if (result.gradients[1].has_gradient)
      {
        // This does work but could be faster
        for (int j = 0; j < n_dof_; j++)
          jac.insertBack(i, position_var_->getIndex() + j) = -1.0 * result.gradients[1].gradient[j];
      }
      ++i;
    }
  }
  jac.finalize();
  assert(rows_ == i);
  return jac;
}

void DiscreteCollisionConstraintD::setBounds(const std::vector<Bounds>& bounds)
{
  assert(bounds.size() == rows_);
  bounds_ = bounds;
}

std::shared_ptr<DiscreteCollisionEvaluator> DiscreteCollisionConstraintD::getCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
