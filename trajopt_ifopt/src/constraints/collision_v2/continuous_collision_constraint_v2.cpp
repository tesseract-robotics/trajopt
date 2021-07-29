/**
 * @file continuous_collision_constraint.cpp
 * @brief The continuous collision position constraint
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
#include <tesseract_collision/core/common.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_v2/continuous_collision_constraint_v2.h>
#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>

namespace trajopt_ifopt
{
ContinuousCollisionConstraintIfoptV2::ContinuousCollisionConstraintIfoptV2(
    ContinuousCollisionEvaluator::Ptr collision_evaluator,
    std::array<JointPosition::ConstPtr, 2> position_vars,
    std::array<bool, 2> position_vars_fixed,
    int max_num_cnt,
    const std::vector<tesseract_collision::ObjectPairKey>& collision_object_pairs,
    const std::string& name)
  : ifopt::ConstraintSet(1, name)
  , position_vars_(std::move(position_vars))
  , position_vars_fixed_(std::move(position_vars_fixed))
  , collision_evaluator_(std::move(collision_evaluator))
{
  if (position_vars_[0] == nullptr && position_vars_[1] == nullptr)
    throw std::runtime_error("position_vars contains a nullptr!");

  // Set n_dof_ for convenience
  n_dof_ = position_vars_[0]->GetRows();
  if (!(n_dof_ > 0))
    throw std::runtime_error("position_vars[0] is empty!");

  if (position_vars_[0]->GetRows() != position_vars_[1]->GetRows())
    throw std::runtime_error("position_vars are not the same size!");

  if (position_vars_fixed_[0] && position_vars_fixed_[1])
    throw std::runtime_error("position_vars are both fixed!");

  bounds_ = std::vector<ifopt::Bounds>(std::min(collision_object_pairs.size(), static_cast<std::size_t>(max_num_cnt)),
                                       ifopt::BoundSmallerZero);
}

Eigen::VectorXd ContinuousCollisionConstraintIfoptV2::GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
  Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
  double margin_buffer = collision_evaluator_->GetCollisionConfig().collision_margin_buffer;
  Eigen::VectorXd values = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(bounds_.size()), -margin_buffer);

  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals0, joint_vals1);

  if (collision_data->gradient_results_set_map.empty())
    return values;

  if (collision_data->gradient_results_set_map.size() <= bounds_.size())
  {
    Eigen::Index i{ 0 };
    if (!position_vars_fixed_[0] && !position_vars_fixed_[0])
    {
      for (const auto& grs : collision_data->gradient_results_set_map)
        values(i++) = getWeightedAvgValues(grs.second)[0];
    }
    else if (!position_vars_fixed_[0])
    {
      for (const auto& grs : collision_data->gradient_results_set_map)
        values(i++) = getWeightedAvgValuesT0(grs.second)[0];
    }
    else
    {
      for (const auto& grs : collision_data->gradient_results_set_map)
        values(i++) = getWeightedAvgValuesT1(grs.second)[0];
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

    if (!position_vars_fixed_[0] && !position_vars_fixed_[0])
    {
      for (std::size_t i = 0; i < bounds_.size(); ++i)
        values(static_cast<Eigen::Index>(i)) = getWeightedAvgValues(rs[i])[0];
    }
    else if (!position_vars_fixed_[0])
    {
      for (std::size_t i = 0; i < bounds_.size(); ++i)
        values(static_cast<Eigen::Index>(i)) = getWeightedAvgValuesT0(rs[i])[0];
    }
    else
    {
      for (std::size_t i = 0; i < bounds_.size(); ++i)
        values(static_cast<Eigen::Index>(i)) = getWeightedAvgValuesT1(rs[i])[0];
    }
  }

  return values;
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> ContinuousCollisionConstraintIfoptV2::GetBounds() const { return bounds_; }

void ContinuousCollisionConstraintIfoptV2::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_vars_[0]->GetName() && !position_vars_fixed_[0])
  {
    // Calculate collisions
    Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
    Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();

    CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals0, joint_vals1);
    if (collision_data->gradient_results_set_map.empty())
      return;

    jac_block.reserve(static_cast<Eigen::Index>(bounds_.size()) * position_vars_[0]->GetRows());

    if (collision_data->gradient_results_set_map.size() <= bounds_.size())
    {
      Eigen::Index i{ 0 };
      for (const auto& grs : collision_data->gradient_results_set_map)
      {
        Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(grs.second);

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
        Eigen::VectorXd grad_vec = getWeightedAvgGradientT0(rs[i]);

        // Collision is 1 x n_dof
        for (int j = 0; j < n_dof_; j++)
          jac_block.coeffRef(static_cast<int>(i), j) = -1.0 * grad_vec[j];
      }
    }
  }
  else if (var_set == position_vars_[1]->GetName() && !position_vars_fixed_[1])
  {
    // Calculate collisions
    Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
    Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();

    CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals0, joint_vals1);
    if (collision_data->gradient_results_set_map.empty())
      return;

    jac_block.reserve(static_cast<Eigen::Index>(bounds_.size()) * position_vars_[0]->GetRows());

    if (collision_data->gradient_results_set_map.size() <= bounds_.size())
    {
      Eigen::Index i{ 0 };
      for (const auto& grs : collision_data->gradient_results_set_map)
      {
        Eigen::VectorXd grad_vec = getWeightedAvgGradientT1(grs.second);

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
        Eigen::VectorXd grad_vec = getWeightedAvgGradientT1(rs[i]);

        // Collision is 1 x n_dof
        for (int j = 0; j < n_dof_; j++)
          jac_block.coeffRef(static_cast<int>(i), j) = -1.0 * grad_vec[j];
      }
    }
  }
}

void ContinuousCollisionConstraintIfoptV2::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

ContinuousCollisionEvaluator::Ptr ContinuousCollisionConstraintIfoptV2::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt_ifopt
