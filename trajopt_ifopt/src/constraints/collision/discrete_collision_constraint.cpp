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

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/collision_utils.h>
#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

namespace trajopt_ifopt
{
DiscreteCollisionConstraint::DiscreteCollisionConstraint(DiscreteCollisionEvaluator::Ptr collision_evaluator,
                                                         JointPosition::ConstPtr position_var,
                                                         int max_num_cnt,
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
  if (var_set == position_var_->GetName())
  {
    // Get current joint values
    VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

    CalcJacobianBlock(joint_vals, jac_block);
  }
}

Eigen::VectorXd DiscreteCollisionConstraint::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  // Check the collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisions(joint_vals);
  double margin_buffer = collision_evaluator_->GetCollisionConfig().collision_margin_buffer;
  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  if (collision_data->gradient_results_set_map.empty())
    return values;

  if (collision_data->gradient_results_set_map.size() <= bounds_.size())
  {
    Eigen::Index i{ 0 };
    for (const auto& grs : collision_data->gradient_results_set_map)
      values(i++) = grs.second.coeff * grs.second.getMaxErrorT0();
  }
  else
  {
    std::vector<std::reference_wrapper<const GradientResultsSet>> rs;
    rs.reserve(collision_data->gradient_results_set_map.size());
    std::transform(collision_data->gradient_results_set_map.begin(),
                   collision_data->gradient_results_set_map.end(),
                   std::back_inserter(rs),
                   [](const std::map<std::pair<std::string, std::string>, GradientResultsSet>::value_type& val) {
                     return std::cref(val.second);
                   });
    std::sort(rs.begin(), rs.end(), [](const GradientResultsSet& a, const GradientResultsSet& b) {
      return a.max_error[0].error > b.max_error[0].error;
    });

    for (std::size_t i = 0; i < bounds_.size(); ++i)
    {
      const GradientResultsSet& r = rs[i].get();
      values(static_cast<Eigen::Index>(i)) = r.coeff * r.getMaxErrorT0();
    }
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
  // Calculate collisions
  jac_block.reserve(static_cast<Eigen::Index>(bounds_.size()) * position_var_->GetRows());
  // Setting to zeros because snopt sparsity cannot change
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(bounds_.size()); i++)
    for (Eigen::Index j = 0; j < n_dof_; j++)
      jac_block.coeffRef(i, j) = 0;

  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisions(joint_vals);
  if (collision_data->gradient_results_set_map.empty())
    return;

  if (collision_data->gradient_results_set_map.size() <= bounds_.size())
  {
    Eigen::Index i{ 0 };
    for (const auto& grs : collision_data->gradient_results_set_map)
    {
      Eigen::VectorXd grad_vec =
          getWeightedAvgGradientT0(grs.second, grs.second.getMaxErrorWithBufferT0(), position_var_->GetRows());

      // Collision is 1 x n_dof
      for (int j = 0; j < n_dof_; j++)
        jac_block.coeffRef(i, j) = -1.0 * grad_vec[j];

      ++i;
    }
  }
  else
  {
    std::vector<std::reference_wrapper<const GradientResultsSet>> rs;
    rs.reserve(collision_data->gradient_results_set_map.size());
    std::transform(collision_data->gradient_results_set_map.begin(),
                   collision_data->gradient_results_set_map.end(),
                   std::back_inserter(rs),
                   [](const std::map<std::pair<std::string, std::string>, GradientResultsSet>::value_type& val) {
                     return std::cref(val.second);
                   });
    std::sort(rs.begin(), rs.end(), [](const GradientResultsSet& a, const GradientResultsSet& b) {
      return a.max_error[0].error > b.max_error[0].error;
    });

    for (std::size_t i = 0; i < bounds_.size(); ++i)
    {
      const GradientResultsSet& r = rs[i];
      Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(r, r.getMaxErrorWithBufferT0(), position_var_->GetRows());

      // Collision is 1 x n_dof
      for (int j = 0; j < n_dof_; j++)
        jac_block.coeffRef(static_cast<int>(i), j) = -1.0 * grad_vec[j];
    }
  }
}

DiscreteCollisionEvaluator::Ptr DiscreteCollisionConstraint::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
