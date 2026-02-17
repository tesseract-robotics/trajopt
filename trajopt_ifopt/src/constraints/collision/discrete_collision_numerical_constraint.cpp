/**
 * @file discrete_collision_numerical_constraint.cpp
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

#include <trajopt_ifopt/constraints/collision/discrete_collision_numerical_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

namespace trajopt_ifopt
{
DiscreteCollisionNumericalConstraint::DiscreteCollisionNumericalConstraint(
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
  non_zeros_ = rows_ * n_dof_;
  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(max_num_cnt), BoundSmallerZero);
}

int DiscreteCollisionNumericalConstraint::update()
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

  // Setting to zeros because snopt sparsity cannot change
  std::call_once(init_flag_, &DiscreteCollisionNumericalConstraint::init, this);

  return rows_;
}

Eigen::VectorXd DiscreteCollisionNumericalConstraint::getValues() const { return calcValues(position_var_->value()); }

Eigen::VectorXd DiscreteCollisionNumericalConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values
std::vector<Bounds> DiscreteCollisionNumericalConstraint::getBounds() const { return bounds_; }

void DiscreteCollisionNumericalConstraint::init() const
{
  if (!fixed_sparsity_)
    return;

  triplet_list_.reserve(static_cast<std::size_t>(bounds_.size()) * static_cast<std::size_t>(position_var_->size()));
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(bounds_.size()); i++)
    for (Eigen::Index j = 0; j < n_dof_; j++)
      triplet_list_.emplace_back(i, position_var_->getIndex() + j, 0);
}

Jacobian DiscreteCollisionNumericalConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  calcJacobianBlock(jac, position_var_->value());  // NOLINT
  return jac;
}

Eigen::VectorXd
DiscreteCollisionNumericalConstraint::calcValues(const Eigen::Ref<const Eigen::VectorXd>& /*joint_vals*/) const
{
  // Check the collisions
  const double margin_buffer = collision_evaluator_->getCollisionMarginBuffer();
  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  if (collision_data_->gradient_results_sets.empty())
    return values;

  const std::size_t cnt = std::min(bounds_.size(), collision_data_->gradient_results_sets.size());
  for (std::size_t i = 0; i < cnt; ++i)
  {
    const auto& grs = collision_data_->gradient_results_sets[i];
    values(static_cast<Eigen::Index>(i)) = grs.getMaxErrorT0();
  }

  return values;
}

void DiscreteCollisionNumericalConstraint::setBounds(const std::vector<Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void DiscreteCollisionNumericalConstraint::calcJacobianBlock(Jacobian& jac_block,
                                                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  // Setting to zeros because snopt sparsity cannot change
  if (!triplet_list_.empty())                                               // NOLINT
    jac_block.setFromTriplets(triplet_list_.begin(), triplet_list_.end());  // NOLINT

  if (collision_data_->gradient_results_sets.empty())
    return;

  const std::size_t cnt = std::min(bounds_.size(), collision_data_->gradient_results_sets.size());

  const double margin_buffer = collision_evaluator_->getCollisionMarginBuffer();

  trajopt_common::CollisionCacheData collision_data_delta;
  Eigen::VectorXd jv = joint_vals;
  const double delta = 1e-8;
  for (int j = 0; j < n_dof_; j++)
  {
    jv(j) = joint_vals(j) + delta;
    collision_evaluator_->calcCollisions(collision_data_delta, jv, bounds_.size());
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
        const double dist_delta = (it->getMaxErrorT0() - baseline.getMaxErrorT0());
        jac_block.coeffRef(i, position_var_->getIndex() + j) = dist_delta / delta;
      }
      else
      {
        const double dist_delta = ((-1.0 * margin_buffer) - baseline.getMaxErrorT0());
        jac_block.coeffRef(i, position_var_->getIndex() + j) = dist_delta / delta;
      }
    }
    jv(j) = joint_vals(j);
  }

  //    //#ifdef 0
  //    Jacobian jac_block_debug(static_cast<Eigen::Index>(bounds_.size()), position_var_->GetRows());
  //    jac_block_debug.reserve(static_cast<Eigen::Index>(bounds_.size()) * position_var_->GetRows());
  //    Eigen::Index i{ 0 };
  //    for (const auto& grs : collision_data->gradient_results_set_map)
  //    {
  //      Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(grs.second, position_var_->GetRows());
  //      for (int j = 0; j < n_dof_; j++)
  //        jac_block_debug.coeffRef(static_cast<int>(i), position_var_->getIndex() + j) = -1.0 * grad_vec[j];
  //      ++i;
  //    }
  //    std::cout << "Numerical Jacobian:" << '\n' << jac_block << '\n';
  //    std::cout << "Col Grad Jacobian:" << '\n' << jac_block_debug << '\n';
  //    //#endif
}

std::shared_ptr<DiscreteCollisionEvaluator> DiscreteCollisionNumericalConstraint::getCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
