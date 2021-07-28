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

#include <trajopt_ifopt/constraints/collision_v2/discrete_collision_constraint_v3.h>
#include <trajopt_ifopt/constraints/collision/collision_utils.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

namespace trajopt_ifopt
{
DiscreteCollisionConstraintIfoptV3::DiscreteCollisionConstraintIfoptV3(
    DiscreteCollisionEvaluator::Ptr collision_evaluator,
    std::vector<tesseract_collision::ObjectPairKey> collision_object_pairs,
    int max_num_cnt,
    JointPosition::ConstPtr position_var,
    const std::string& name)
  : ifopt::ConstraintSet(std::min(static_cast<int>(collision_object_pairs.size()), max_num_cnt), name)
  , position_var_(std::move(position_var))
  , collision_evaluator_(std::move(collision_evaluator))
  , collision_object_pairs_(std::move(collision_object_pairs))
{
  // Set n_dof_ for convenience
  n_dof_ = position_var_->GetRows();
  assert(n_dof_ > 0);

  bounds_ = std::vector<ifopt::Bounds>(std::min(collision_object_pairs_.size(), static_cast<std::size_t>(max_num_cnt)),
                                       ifopt::BoundSmallerZero);
}

Eigen::VectorXd DiscreteCollisionConstraintIfoptV3::GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

  return CalcValues(joint_vals);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> DiscreteCollisionConstraintIfoptV3::GetBounds() const { return bounds_; }

void DiscreteCollisionConstraintIfoptV3::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var_->GetName())
  {
    // Get current joint values
    VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

    CalcJacobianBlock(joint_vals, jac_block);
  }
}

Eigen::VectorXd
DiscreteCollisionConstraintIfoptV3::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
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
      values(i++) = getAverageWeightedValuesPost(grs.second)[0];
  }
  else
  {
    std::vector<GradientResultsSet> rs;
    rs.reserve(collision_data->gradient_results_set_map.size());
    std::transform(collision_data->gradient_results_set_map.begin(),
                   collision_data->gradient_results_set_map.end(),
                   std::back_inserter(rs),
                   std::bind(&std::map<std::pair<std::string, std::string>, GradientResultsSet>::value_type::second,
                             std::placeholders::_1));
    std::sort(rs.begin(), rs.end(), [](const GradientResultsSet& a, const GradientResultsSet& b) {
      return a.max_error > b.max_error;
    });

    for (std::size_t i = 0; i < bounds_.size(); ++i)
      values(static_cast<Eigen::Index>(i)) = getAverageWeightedValuesPost(rs[i])[0];
  }

  return values;
}

void DiscreteCollisionConstraintIfoptV3::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void DiscreteCollisionConstraintIfoptV3::CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                           Jacobian& jac_block) const
{
  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisions(joint_vals);
  if (collision_data->gradient_results_set_map.empty())
    return;

  jac_block.reserve(static_cast<Eigen::Index>(bounds_.size()) * position_var_->GetRows());

  if (collision_data->gradient_results_set_map.size() <= bounds_.size())
  {
    Eigen::Index i{ 0 };
    for (const auto& grs : collision_data->gradient_results_set_map)
    {
      Eigen::VectorXd grad_vec = getWeightedAvgGradientPost(grs.second);

      // Collision is 1 x n_dof
      for (int j = 0; j < n_dof_; j++)
        jac_block.coeffRef(i, j) = -1.0 * grad_vec[j];

      ++i;
    }
  }
  else
  {
    std::vector<GradientResultsSet> rs;
    rs.reserve(collision_data->gradient_results_set_map.size());
    std::transform(collision_data->gradient_results_set_map.begin(),
                   collision_data->gradient_results_set_map.end(),
                   std::back_inserter(rs),
                   std::bind(&std::map<std::pair<std::string, std::string>, GradientResultsSet>::value_type::second,
                             std::placeholders::_1));
    std::sort(rs.begin(), rs.end(), [](const GradientResultsSet& a, const GradientResultsSet& b) {
      return a.max_error > b.max_error;
    });

    for (std::size_t i = 0; i < bounds_.size(); ++i)
    {
      Eigen::VectorXd grad_vec = getWeightedAvgGradientPost(rs[i]);

      // Collision is 1 x n_dof
      for (int j = 0; j < n_dof_; j++)
        jac_block.coeffRef(static_cast<int>(i), j) = -1.0 * grad_vec[j];
    }
  }
}

DiscreteCollisionEvaluator::Ptr DiscreteCollisionConstraintIfoptV3::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
