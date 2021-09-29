/**
 * @file discrete_collision_evaluators.h
 * @brief Contains discrete evaluators for the collision constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
 * @bug Only Discrete Evaluator is implemented
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
#include <trajopt_ifopt/constraints/collision/collision_utils.h>

namespace trajopt_ifopt
{
SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(std::shared_ptr<CollisionCache> collision_cache,
                                                                   tesseract_kinematics::JointGroup::ConstPtr manip,
                                                                   tesseract_environment::Environment::ConstPtr env,
                                                                   TrajOptCollisionConfig::ConstPtr collision_config,
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
  contact_manager_->setCollisionMarginData(collision_config_->collision_margin_data);
  // Increase the default by the buffer
  contact_manager_->setDefaultCollisionMarginData(collision_config_->collision_margin_data.getMaxCollisionMargin() +
                                                  collision_config_->collision_margin_buffer);
}

CollisionCacheData::ConstPtr
SingleTimestepCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals)
{
  size_t key = getHash(*collision_config_, dof_vals);
  auto* it = collision_cache_->get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    return *it;
  }

  CONSOLE_BRIDGE_logDebug("Not using cached collision check");
  auto data = std::make_shared<CollisionCacheData>();
  CalcCollisionsHelper(dof_vals, data->contact_results_map);

  for (const auto& pair : data->contact_results_map)
  {
    GradientResultsSet grs;
    grs.key = pair.first;
    grs.coeff = collision_config_->collision_coeff_data.getPairCollisionCoeff(grs.key.first, grs.key.second);
    grs.results.reserve(pair.second.size());
    for (const tesseract_collision::ContactResult& dist_result : pair.second)
      grs.add(GetGradient(dof_vals, dist_result));

    data->gradient_results_set_map[pair.first] = grs;
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

  for (auto& pair : dist_results)
  {
    // Contains the contact distance threshold and coefficient for the given link pair
    double dist = collision_config_->collision_margin_data.getPairCollisionMargin(pair.first.first, pair.first.second);
    double coeff = collision_config_->collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
    const Eigen::Vector2d data = { dist, coeff };
    auto end = std::remove_if(
        pair.second.begin(), pair.second.end(), [&data, this](const tesseract_collision::ContactResult& r) {
          return (!((data[0] + collision_config_->collision_margin_buffer) > r.distance));
        });
    pair.second.erase(end, pair.second.end());
  }
}

GradientResults SingleTimestepCollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals,
                                                              const tesseract_collision::ContactResult& contact_result)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  double margin = collision_config_->collision_margin_data.getPairCollisionMargin(contact_result.link_names[0],
                                                                                  contact_result.link_names[1]);

  return getGradient(dofvals, contact_result, margin, collision_config_->collision_margin_buffer, manip_);
}

const TrajOptCollisionConfig& SingleTimestepCollisionEvaluator::GetCollisionConfig() const
{
  return *collision_config_;
}

}  // namespace trajopt_ifopt
