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
#include <trajopt_common/collision_utils.h>
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
    std::string name)
  : ConstraintSet(std::move(name), max_num_cnt)
  , position_vars_(std::move(position_vars))
  , vars0_fixed_(vars0_fixed)
  , vars1_fixed_(vars1_fixed)
  , collision_evaluator_(std::move(collision_evaluator))
  , fixed_sparsity_(fixed_sparsity)
{
  if (position_vars_[0] == nullptr && position_vars_[1] == nullptr)
    throw std::runtime_error("ContinuousCollisionConstraint: position_vars contains a nullptr!");

  // All vars must have same size
  n_dof_ = position_vars_.front()->size();
  if (n_dof_ <= 0)
    throw std::runtime_error("ContinuousCollisionConstraint: first position var is empty!");

  if (position_vars_[0]->size() != position_vars_[1]->size())
    throw std::runtime_error("ContinuousCollisionConstraint: all position_vars must have same size!");

  if (vars0_fixed_ && vars1_fixed_)
    throw std::runtime_error("ContinuousCollisionConstraint: both ends cannot be fixed!");

  if (max_num_cnt < 1)
    throw std::runtime_error("ContinuousCollisionConstraint: max_num_cnt must be greater than zero!");

  coeffs_ = Eigen::VectorXd::Constant(rows_, 1);
  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(max_num_cnt), BoundSmallerZero);
  non_zeros_ = 2 * static_cast<Eigen::Index>(rows_) * n_dof_;
}

int ContinuousCollisionConstraint::update()
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

  std::call_once(init_flag_, &ContinuousCollisionConstraint::init, this);

  return rows_;
}

Eigen::VectorXd ContinuousCollisionConstraint::getValues() const
{
  const double margin_buffer = collision_evaluator_->getCollisionMarginBuffer();
  Eigen::VectorXd values = Eigen::VectorXd::Constant(rows_, -margin_buffer);

  if (collision_data_->gradient_results_sets.empty())
    return values;

  const auto cnt = std::min<std::size_t>(bounds_.size(), collision_data_->gradient_results_sets.size());
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
      if (r.max_error[0].has_error[0] || r.max_error[1].has_error[0])
        values(static_cast<Eigen::Index>(i)) = r.getMaxErrorT0();
    }
  }
  else
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
      if (r.max_error[0].has_error[1] || r.max_error[1].has_error[1])
        values(static_cast<Eigen::Index>(i)) = r.getMaxErrorT1();
    }
  }

  return values;
}

Eigen::VectorXd ContinuousCollisionConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values
std::vector<Bounds> ContinuousCollisionConstraint::getBounds() const { return bounds_; }

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

  if (!fixed_sparsity_)
    return;

  triplet_list_.clear();

  // Setting to zeros because snopt sparsity cannot change
  triplet_list_.reserve(bounds_.size() * static_cast<std::size_t>(position_vars_[0]->size()));
  for (Eigen::Index i = 0; i < rows_; i++)  // NOLINT
  {
    for (Eigen::Index j = 0; j < n_dof_; j++)
    {
      triplet_list_.emplace_back(i, position_vars_[0]->getIndex() + j, 0);
      triplet_list_.emplace_back(i, position_vars_[1]->getIndex() + j, 0);
    }
  }
}

Jacobian ContinuousCollisionConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  // Setting to zeros because snopt sparsity cannot change
  if (!triplet_list_.empty())                                         // NOLINT
    jac.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  if (collision_data_->gradient_results_sets.empty())
    return jac;

  /** @todo Should use triplet list and setFromTriplets */
  const std::size_t cnt = std::min(collision_data_->gradient_results_sets.size(), bounds_.size());
  if (!vars0_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
      if (r.max_error[0].has_error[0] || r.max_error[1].has_error[0])
      {
        double max_error_with_buffer = r.getMaxErrorWithBufferT0();
        if (!vars1_fixed_)
          max_error_with_buffer = r.getMaxErrorWithBuffer();

        Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(r, max_error_with_buffer, position_vars_[0]->size());

        // Collision is 1 x n_dof
        for (int j = 0; j < n_dof_; j++)
          jac.coeffRef(static_cast<int>(i), position_vars_[0]->getIndex() + j) = -1.0 * grad_vec[j];
      }
    }
  }

  if (!vars1_fixed_)
  {
    for (std::size_t i = 0; i < cnt; ++i)
    {
      const trajopt_common::GradientResultsSet& r = collision_data_->gradient_results_sets[i];
      if (r.max_error[0].has_error[1] || r.max_error[1].has_error[1])
      {
        double max_error_with_buffer = r.getMaxErrorWithBufferT1();
        if (!vars0_fixed_)
          max_error_with_buffer = r.getMaxErrorWithBuffer();

        Eigen::VectorXd grad_vec = getWeightedAvgGradientT1(r, max_error_with_buffer, position_vars_[1]->size());

        // Collision is 1 x n_dof
        for (int j = 0; j < n_dof_; j++)
          jac.coeffRef(static_cast<int>(i), position_vars_[1]->getIndex() + j) = -1.0 * grad_vec[j];
      }
    }
  }
  return jac;
}

void ContinuousCollisionConstraint::setBounds(const std::vector<Bounds>& bounds)
{
  assert(bounds.size() == rows_);
  bounds_ = bounds;
}

std::shared_ptr<ContinuousCollisionEvaluator> ContinuousCollisionConstraint::getCollisionEvaluator() const
{
  return collision_evaluator_;
}

ContinuousCollisionConstraintD::ContinuousCollisionConstraintD(
    std::shared_ptr<ContinuousCollisionEvaluator> collision_evaluator,
    std::array<std::shared_ptr<const Var>, 2> position_vars,
    bool vars0_fixed,
    bool vars1_fixed,
    std::string name)
  : ConstraintSet(std::move(name), true)
  , position_vars_(std::move(position_vars))
  , vars0_fixed_(vars0_fixed)
  , vars1_fixed_(vars1_fixed)
  , collision_evaluator_(std::move(collision_evaluator))
{
  if (position_vars_[0] == nullptr && position_vars_[1] == nullptr)
    throw std::runtime_error("ContinuousCollisionConstraint: position_vars contains a nullptr!");

  // All vars must have same size
  n_dof_ = position_vars_.front()->size();
  if (n_dof_ <= 0)
    throw std::runtime_error("ContinuousCollisionConstraint: first position var is empty!");

  if (position_vars_[0]->size() != position_vars_[1]->size())
    throw std::runtime_error("ContinuousCollisionConstraint: all position_vars must have same size!");

  if (vars0_fixed_ && vars1_fixed_)
    throw std::runtime_error("ContinuousCollisionConstraint: both ends cannot be fixed!");
}

int ContinuousCollisionConstraintD::update()
{
  std::size_t variable_hash = position_vars_[0]->getParent()->getParent()->getHash();
  auto cache_data = collision_data_cache_.getOrAcquire(variable_hash);

  collision_data_ = cache_data.first;
  if (!cache_data.second)
  {
    collision_data_->contact_results_map.clear();
    collision_data_->gradient_results_sets.clear();
    collision_evaluator_->calcCollisionData(
        *collision_data_, position_vars_[0]->value(), position_vars_[1]->value(), vars0_fixed_, vars1_fixed_, 0);
    collision_data_cache_.put(variable_hash, collision_data_);
  }

  rows_ = static_cast<int>(collision_data_->contact_results_map.count());
  non_zeros_ = 2 * static_cast<Eigen::Index>(rows_) * n_dof_;
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

Eigen::VectorXd ContinuousCollisionConstraintD::getValues() const { return values_; }

// Set the limits on the constraint values
std::vector<Bounds> ContinuousCollisionConstraintD::getBounds() const { return bounds_; }

Jacobian ContinuousCollisionConstraintD::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  if (collision_data_->contact_results_map.empty())
    return jac;

  auto jp0 = position_vars_[0]->value();
  auto jp1 = position_vars_[1]->value();

  trajopt_common::GradientResults result;
  Eigen::Index i{ 0 };
  for (const auto& pair : collision_data_->contact_results_map)
  {
    for (const auto& contact_results : pair.second)
    {
      result.clear();
      trajopt_common::getGradient(result, jp0, jp1, contact_results, 0, 0, collision_evaluator_->getJointGroup());
      jac.startVec(i);
      assert(result.gradients[0].has_gradient || result.gradients[1].has_gradient ||
             result.cc_gradients[0].has_gradient || result.cc_gradients[1].has_gradient);

      if (!vars0_fixed_)
      {
        const Eigen::Index offset = position_vars_[0]->getIndex();
        if (result.gradients[0].has_gradient && result.gradients[1].has_gradient)
        {
          const auto& lgr0 = result.gradients[0];
          const auto& lgr1 = result.gradients[1];

          // This does work but could be faster
          for (int j = 0; j < n_dof_; j++)
            jac.insertBack(i, offset + j) = -1.0 * ((lgr0.scale * lgr0.gradient[j]) + (lgr1.scale * lgr1.gradient[j]));
        }
        else if (result.gradients[0].has_gradient)
        {
          const auto& lgr = result.gradients[0];

          // This does work but could be faster
          for (int j = 0; j < n_dof_; j++)
            jac.insertBack(i, offset + j) = -1.0 * lgr.scale * lgr.gradient[j];
        }
        else if (result.gradients[1].has_gradient)
        {
          const auto& lgr = result.gradients[1];

          // This does work but could be faster
          for (int j = 0; j < n_dof_; j++)
            jac.insertBack(i, offset + j) = -1.0 * lgr.scale * lgr.gradient[j];
        }
      }

      if (!vars1_fixed_)
      {
        const Eigen::Index offset = position_vars_[1]->getIndex();
        if (result.cc_gradients[0].has_gradient && result.cc_gradients[1].has_gradient)
        {
          const auto& lgr0 = result.cc_gradients[0];
          const auto& lgr1 = result.cc_gradients[1];

          // This does work but could be faster
          for (int j = 0; j < n_dof_; j++)
            jac.insertBack(i, offset + j) = -1.0 * ((lgr0.scale * lgr0.gradient[j]) + (lgr1.scale * lgr1.gradient[j]));
        }
        else if (result.cc_gradients[0].has_gradient)
        {
          const auto& lgr = result.cc_gradients[0];

          // This does work but could be faster
          for (int j = 0; j < n_dof_; j++)
            jac.insertBack(i, offset + j) = -1.0 * lgr.scale * lgr.gradient[j];
        }
        else if (result.cc_gradients[1].has_gradient)
        {
          const auto& lgr = result.cc_gradients[1];

          // This does work but could be faster
          for (int j = 0; j < n_dof_; j++)
            jac.insertBack(i, offset + j) = -1.0 * lgr.scale * lgr.gradient[j];
        }
      }
      ++i;
    }
  }
  jac.finalize();
  assert(rows_ == i);
  return jac;
}

Eigen::VectorXd ContinuousCollisionConstraintD::getCoefficients() const { return coeffs_; }

void ContinuousCollisionConstraintD::setBounds(const std::vector<Bounds>& bounds)
{
  assert(bounds.size() == rows_);
  bounds_ = bounds;
}

std::shared_ptr<ContinuousCollisionEvaluator> ContinuousCollisionConstraintD::getCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
