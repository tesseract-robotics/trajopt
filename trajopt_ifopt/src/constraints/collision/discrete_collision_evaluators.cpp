/**
 * @file discrete_collision_evaluators.h
 * @brief Contains discrete evaluators for the collision constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
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

#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_common/collision_utils.h>

namespace trajopt_ifopt
{
SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(
    std::shared_ptr<trajopt_common::CollisionCache> collision_cache,
    tesseract_kinematics::JointGroup::ConstPtr manip,
    tesseract_environment::Environment::ConstPtr env,
    trajopt_common::TrajOptCollisionConfig::ConstPtr collision_config,
    bool dynamic_environment)
  : collision_cache_(std::move(collision_cache))
  , manip_(std::move(manip))
  , env_(std::move(env))
  , collision_config_(std::move(collision_config))
  , dynamic_environment_(dynamic_environment)
{
  manip_active_link_names_ = manip_->getActiveLinkNames();

  // If the environment is not expected to change, then the cloned state solver may be used each time.
  if (dynamic_environment_)
  {
    get_state_fn_ = [&](const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return env_->getState(manip_->getJointNames(), joint_values).link_transforms;
    };
    env_active_link_names_ = env_->getActiveLinkNames();

    std::sort(manip_active_link_names_.begin(), manip_active_link_names_.end());
    std::sort(env_active_link_names_.begin(), env_active_link_names_.end());
    std::set_difference(env_active_link_names_.begin(),
                        env_active_link_names_.end(),
                        manip_active_link_names_.begin(),
                        manip_active_link_names_.end(),
                        std::inserter(diff_active_link_names_, diff_active_link_names_.begin()));
  }
  else
  {
    get_state_fn_ = [&](const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return manip_->calcFwdKin(joint_values);
    };
    env_active_link_names_ = manip_->getActiveLinkNames();
  }

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(manip_active_link_names_);
  contact_manager_->setCollisionMarginData(collision_config_->contact_manager_config.margin_data);
  // Increase the default by the buffer
  contact_manager_->setDefaultCollisionMarginData(
      collision_config_->contact_manager_config.margin_data.getMaxCollisionMargin() +
      collision_config_->collision_margin_buffer);
}

trajopt_common::CollisionCacheData::ConstPtr
SingleTimestepCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                                                 std::size_t bounds_size)
{
  size_t key = getHash(*collision_config_, dof_vals);
  auto* it = collision_cache_->get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    return *it;
  }

  CONSOLE_BRIDGE_logDebug("Not using cached collision check");
  auto data = std::make_shared<trajopt_common::CollisionCacheData>();
  CalcCollisionsHelper(dof_vals, data->contact_results_map);

  for (const auto& pair : data->contact_results_map)
  {
    using ShapeGrsType = std::map<std::pair<std::size_t, std::size_t>, trajopt_common::GradientResultsSet>;
    ShapeGrsType shape_grs;
    const double coeff =
        collision_config_->collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
    for (const tesseract_collision::ContactResult& dist_result : pair.second)
    {
      const std::size_t shape_hash0 = trajopt_common::cantorHash(dist_result.shape_id[0], dist_result.subshape_id[0]);
      const std::size_t shape_hash1 = trajopt_common::cantorHash(dist_result.shape_id[1], dist_result.subshape_id[1]);
      auto shape_key = std::make_pair(shape_hash0, shape_hash1);
      auto it = shape_grs.find(shape_key);
      if (it == shape_grs.end())
      {
        trajopt_common::GradientResultsSet grs;
        grs.key = pair.first;
        grs.shape_key = shape_key;
        grs.coeff = coeff;
        grs.results.reserve(pair.second.size());
        grs.add(GetGradient(dof_vals, dist_result));
        shape_grs[shape_key] = grs;
      }
      else
      {
        it->second.add(GetGradient(dof_vals, dist_result));
      }
    }

    // This is not as efficient as it could be. Need to update Tesseract to store per subhshape key
    const std::size_t new_size = data->gradient_results_sets.size() + shape_grs.size();
    data->gradient_results_sets.reserve(new_size);

    std::transform(shape_grs.begin(),
                   shape_grs.end(),
                   std::back_inserter(data->gradient_results_sets),
                   std::bind(&ShapeGrsType::value_type::second, std::placeholders::_1));  // NOLINT
  }

  if (data->gradient_results_sets.size() > bounds_size)
  {
    std::sort(data->gradient_results_sets.begin(),
              data->gradient_results_sets.end(),
              [](const trajopt_common::GradientResultsSet& a, const trajopt_common::GradientResultsSet& b) {
                return a.max_error[0].error > b.max_error[0].error;
              });
  }

  collision_cache_->put(key, data);
  return data;
}

void SingleTimestepCollisionEvaluator::CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                                                            tesseract_collision::ContactResultMap& dist_results)
{
  tesseract_common::TransformMap state = get_state_fn_(dof_vals);

  // If not empty then there are links that are not part of the kinematics object that can move (dynamic environment)
  for (const auto& link_name : diff_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);

  for (const auto& link_name : manip_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);

  contact_manager_->contactTest(dist_results, collision_config_->contact_request);

  // Don't include contacts at the fixed state
  // Don't include contacts with zero coeffs
  const auto& zero_coeff_pairs = collision_config_->collision_coeff_data.getPairsWithZeroCoeff();
  auto filter = [this, &zero_coeff_pairs](tesseract_collision::ContactResultMap::PairType& pair) {
    // Remove pairs with zero coeffs
    if (std::find(zero_coeff_pairs.begin(), zero_coeff_pairs.end(), pair.first) != zero_coeff_pairs.end())
    {
      pair.second.clear();
      return;
    }

    // Contains the contact distance threshold and coefficient for the given link pair
    double dist = collision_config_->contact_manager_config.margin_data.getPairCollisionMargin(pair.first.first,
                                                                                               pair.first.second);
    double coeff = collision_config_->collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
    const Eigen::Vector2d data = { dist, coeff };
    auto end = std::remove_if(
        pair.second.begin(), pair.second.end(), [&data, this](const tesseract_collision::ContactResult& r) {
          return (!((data[0] + collision_config_->collision_margin_buffer) > r.distance));
        });
    pair.second.erase(end, pair.second.end());
  };

  dist_results.filter(filter);
}

trajopt_common::GradientResults
SingleTimestepCollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals,
                                              const tesseract_collision::ContactResult& contact_result)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  double margin = collision_config_->contact_manager_config.margin_data.getPairCollisionMargin(
      contact_result.link_names[0], contact_result.link_names[1]);

  return trajopt_common::getGradient(
      dofvals, contact_result, margin, collision_config_->collision_margin_buffer, manip_);
}

const trajopt_common::TrajOptCollisionConfig& SingleTimestepCollisionEvaluator::GetCollisionConfig() const
{
  return *collision_config_;
}

}  // namespace trajopt_ifopt
