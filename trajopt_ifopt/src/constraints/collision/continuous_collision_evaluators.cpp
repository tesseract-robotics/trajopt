/**
 * @file continuous_collision_evaluators.h
 * @brief Contains continuous evaluators for the collision constraint
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

#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/collision_utils.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
LVSContinuousCollisionEvaluator::LVSContinuousCollisionEvaluator(
    std::shared_ptr<CollisionCache> collision_cache,
    std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
    std::shared_ptr<const tesseract_environment::Environment> env,
    const trajopt_common::TrajOptCollisionConfig& collision_config,
    bool dynamic_environment)
  : collision_cache_(std::move(collision_cache))
  , manip_(std::move(manip))
  , env_(std::move(env))
  , collision_check_config_(collision_config.collision_check_config)
  , dynamic_environment_(dynamic_environment)
{
  if (collision_check_config_.type != tesseract_collision::CollisionEvaluatorType::CONTINUOUS &&
      collision_check_config_.type != tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    throw std::runtime_error("LVSContinuousCollisionEvaluator, should be configured with CONTINUOUS or LVS_CONTINUOUS");
  }

  single_timestep_ = (collision_check_config_.type == tesseract_collision::CollisionEvaluatorType::CONTINUOUS);

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

  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(manip_active_link_names_);
  contact_manager_->applyContactManagerConfig(collision_config.contact_manager_config);
  // Must make a copy after applying the config but before we increment the margin data
  margin_data_ = contact_manager_->getCollisionMarginData();
  coeff_data_ = collision_config.collision_coeff_data;
  margin_buffer_ = collision_config.collision_margin_buffer;

  // Increase the default by the buffer
  contact_manager_->incrementCollisionMargin(margin_buffer_);
}

std::shared_ptr<const trajopt_common::CollisionCacheData>
LVSContinuousCollisionEvaluator::CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                   bool vars0_fixed,
                                                   bool vars1_fixed,
                                                   std::size_t bounds_size)
{
  const std::size_t key = trajopt_common::getHash(this, dof_vals0, dof_vals1);
  auto* it = collision_cache_->get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    return *it;
  }

  auto data = std::make_shared<trajopt_common::CollisionCacheData>();
  CalcCollisionsHelper(data->contact_results_map, dof_vals0, dof_vals1, vars0_fixed, vars1_fixed);

  for (const auto& pair : data->contact_results_map)
  {
    using ShapeGrsType = std::map<std::pair<std::size_t, std::size_t>, trajopt_common::GradientResultsSet>;
    ShapeGrsType shape_grs;
    const double coeff = coeff_data_.getCollisionCoeff(pair.first.first, pair.first.second);
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
        grs.is_continuous = true;
        grs.results.reserve(pair.second.size());
        grs.add(CalcGradientData(dof_vals0, dof_vals1, dist_result));
        shape_grs[shape_key] = grs;
      }
      else
      {
        it->second.add(CalcGradientData(dof_vals0, dof_vals1, dist_result));
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
    if (!vars0_fixed && !vars1_fixed)
    {
      std::sort(data->gradient_results_sets.begin(),
                data->gradient_results_sets.end(),
                [](const trajopt_common::GradientResultsSet& a, const trajopt_common::GradientResultsSet& b) {
                  return a.getMaxErrorWithBuffer() > b.getMaxErrorWithBuffer();
                });
    }
    else if (!vars0_fixed)
    {
      std::sort(data->gradient_results_sets.begin(),
                data->gradient_results_sets.end(),
                [](const trajopt_common::GradientResultsSet& a, const trajopt_common::GradientResultsSet& b) {
                  return a.getMaxErrorWithBufferT0() > b.getMaxErrorWithBufferT0();
                });
    }
    else
    {
      std::sort(data->gradient_results_sets.begin(),
                data->gradient_results_sets.end(),
                [](const trajopt_common::GradientResultsSet& a, const trajopt_common::GradientResultsSet& b) {
                  return a.getMaxErrorWithBufferT1() > b.getMaxErrorWithBufferT1();
                });
    }
  }

  collision_cache_->put(key, data);
  return data;
}

void LVSContinuousCollisionEvaluator::CalcCollisionsHelper(tesseract_collision::ContactResultMap& dist_results,
                                                           const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                           const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                           bool vars0_fixed,
                                                           bool vars1_fixed)
{
  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
  // the collision checking is broken up into multiple casted collision checks such that each check is less then
  // the longest valid segment length.
  const double dist = (dof_vals1 - dof_vals0).norm();

  // If not empty then there are links that are not part of the kinematics object that can move (dynamic environment)
  if (!diff_active_link_names_.empty())
  {
    tesseract_common::TransformMap state = get_state_fn_(dof_vals0);
    for (const auto& link_name : diff_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);
  }

  // Create filter
  // Don't include contacts at the fixed state
  // Don't include contacts with zero coeffs
  const auto& zero_coeff_pairs = coeff_data_.getPairsWithZeroCoeff();
  auto filter =
      [this, &zero_coeff_pairs, vars0_fixed, vars1_fixed](tesseract_collision::ContactResultMap::PairType& pair) {
        // Remove pairs with zero coeffs
        if (zero_coeff_pairs.find(pair.first) != zero_coeff_pairs.end())
        {
          pair.second.clear();
          return;
        }

        // Contains the contact distance threshold and coefficient for the given link pair
        const double margin = margin_data_.getCollisionMargin(pair.first.first, pair.first.second);
        trajopt_common::removeInvalidContactResults(pair.second, margin, margin_buffer_, vars0_fixed, vars1_fixed);
      };

  if (!single_timestep_ && dist > collision_check_config_.longest_valid_segment_length)
  {
    // Calculate the number state to interpolate
    const long cnt = static_cast<long>(std::ceil(dist / collision_check_config_.longest_valid_segment_length)) + 1;

    // Create interpolated trajectory between two states that satisfies the longest valid segment length.
    tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
    for (long i = 0; i < dof_vals0.size(); ++i)
      subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

    // Perform casted collision checking for sub trajectory and store results in contacts_vector
    tesseract_collision::ContactResultMap contacts{ dist_results };
    const int last_state_idx{ static_cast<int>(subtraj.rows()) - 1 };
    const double dt = 1.0 / double(last_state_idx);
    for (int i = 0; i < subtraj.rows() - 1; ++i)
    {
      tesseract_common::TransformMap state0 = get_state_fn_(subtraj.row(i));
      tesseract_common::TransformMap state1 = get_state_fn_(subtraj.row(i + 1));

      for (const auto& link_name : manip_active_link_names_)
        contact_manager_->setCollisionObjectsTransform(link_name, state0[link_name], state1[link_name]);

      contact_manager_->contactTest(contacts, collision_check_config_.contact_request);
      if (!contacts.empty())
      {
        dist_results.addInterpolatedCollisionResults(
            contacts, i, last_state_idx, manip_active_link_names_, dt, false, filter);
      }
      contacts.clear();
    }
  }
  else
  {
    tesseract_common::TransformMap state0 = get_state_fn_(dof_vals0);
    tesseract_common::TransformMap state1 = get_state_fn_(dof_vals1);
    for (const auto& link_name : manip_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state0[link_name], state1[link_name]);

    contact_manager_->contactTest(dist_results, collision_check_config_.contact_request);

    dist_results.filter(filter);
  }
}

trajopt_common::GradientResults
LVSContinuousCollisionEvaluator::CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                  const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                  const tesseract_collision::ContactResult& contact_results)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  const double margin = margin_data_.getCollisionMargin(contact_results.link_names[0], contact_results.link_names[1]);
  return trajopt_common::getGradient(dof_vals0, dof_vals1, contact_results, margin, margin_buffer_, *manip_);
}

double LVSContinuousCollisionEvaluator::GetCollisionMarginBuffer() const { return margin_buffer_; }

const tesseract_common::CollisionMarginData& LVSContinuousCollisionEvaluator::GetCollisionMarginData() const
{
  return margin_data_;
}

const trajopt_common::CollisionCoeffData& LVSContinuousCollisionEvaluator::GetCollisionCoeffData() const
{
  return coeff_data_;
}

//////////////////////////////////////////

LVSDiscreteCollisionEvaluator::LVSDiscreteCollisionEvaluator(
    std::shared_ptr<CollisionCache> collision_cache,
    std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
    std::shared_ptr<const tesseract_environment::Environment> env,
    const trajopt_common::TrajOptCollisionConfig& collision_config,
    bool dynamic_environment)
  : collision_cache_(std::move(collision_cache))
  , manip_(std::move(manip))
  , env_(std::move(env))
  , collision_check_config_(collision_config.collision_check_config)
  , dynamic_environment_(dynamic_environment)
{
  if (collision_check_config_.type != tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
    throw std::runtime_error("LVSDiscreteCollisionEvaluator, should be configured with LVS_DISCRETE");

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
  contact_manager_->applyContactManagerConfig(collision_config.contact_manager_config);
  // Must make a copy after applying the config but before we increment the margin data
  margin_data_ = contact_manager_->getCollisionMarginData();
  coeff_data_ = collision_config.collision_coeff_data;
  margin_buffer_ = collision_config.collision_margin_buffer;

  // Increase the default by the buffer
  contact_manager_->incrementCollisionMargin(margin_buffer_);
}

std::shared_ptr<const trajopt_common::CollisionCacheData>
LVSDiscreteCollisionEvaluator::CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                 const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                 bool vars0_fixed,
                                                 bool vars1_fixed,
                                                 std::size_t bounds_size)
{
  const std::size_t key = trajopt_common::getHash(this, dof_vals0, dof_vals1);
  auto* it = collision_cache_->get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    return *it;
  }

  auto data = std::make_shared<trajopt_common::CollisionCacheData>();
  CalcCollisionsHelper(data->contact_results_map, dof_vals0, dof_vals1, vars0_fixed, vars1_fixed);
  for (const auto& pair : data->contact_results_map)
  {
    using ShapeGrsType = std::map<std::pair<std::size_t, std::size_t>, trajopt_common::GradientResultsSet>;
    ShapeGrsType shape_grs;
    const double coeff = coeff_data_.getCollisionCoeff(pair.first.first, pair.first.second);
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
        grs.is_continuous = true;
        grs.results.reserve(pair.second.size());
        grs.add(CalcGradientData(dof_vals0, dof_vals1, dist_result));
        shape_grs[shape_key] = grs;
      }
      else
      {
        it->second.add(CalcGradientData(dof_vals0, dof_vals1, dist_result));
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
    if (!vars0_fixed && !vars1_fixed)
    {
      std::sort(data->gradient_results_sets.begin(),
                data->gradient_results_sets.end(),
                [](const trajopt_common::GradientResultsSet& a, const trajopt_common::GradientResultsSet& b) {
                  return a.getMaxErrorWithBuffer() > b.getMaxErrorWithBuffer();
                });
    }
    else if (!vars0_fixed)
    {
      std::sort(data->gradient_results_sets.begin(),
                data->gradient_results_sets.end(),
                [](const trajopt_common::GradientResultsSet& a, const trajopt_common::GradientResultsSet& b) {
                  return a.getMaxErrorWithBufferT0() > b.getMaxErrorWithBufferT0();
                });
    }
    else
    {
      std::sort(data->gradient_results_sets.begin(),
                data->gradient_results_sets.end(),
                [](const trajopt_common::GradientResultsSet& a, const trajopt_common::GradientResultsSet& b) {
                  return a.getMaxErrorWithBufferT1() > b.getMaxErrorWithBufferT1();
                });
    }
  }

  collision_cache_->put(key, data);
  return data;
}

void LVSDiscreteCollisionEvaluator::CalcCollisionsHelper(tesseract_collision::ContactResultMap& dist_results,
                                                         const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                         const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                         bool vars0_fixed,
                                                         bool vars1_fixed)
{
  // If not empty then there are links that are not part of the kinematics object that can move (dynamic environment)
  if (!diff_active_link_names_.empty())
  {
    tesseract_common::TransformMap state = get_state_fn_(dof_vals0);
    for (const auto& link_name : diff_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);
  }

  // Create filter
  // Don't include contacts at the fixed state
  // Don't include contacts with zero coeffs
  const auto& zero_coeff_pairs = coeff_data_.getPairsWithZeroCoeff();
  auto filter =
      [this, &zero_coeff_pairs, vars0_fixed, vars1_fixed](tesseract_collision::ContactResultMap::PairType& pair) {
        // Remove pairs with zero coeffs
        if (zero_coeff_pairs.find(pair.first) != zero_coeff_pairs.end())
        {
          pair.second.clear();
          return;
        }

        // Contains the contact distance threshold and coefficient for the given link pair
        const double margin = margin_data_.getCollisionMargin(pair.first.first, pair.first.second);

        // Don't include contacts at the fixed state
        trajopt_common::removeInvalidContactResults(pair.second, margin, margin_buffer_, vars0_fixed, vars1_fixed);
      };

  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
  // the collision checking is broken up into multiple casted collision checks such that each check is less then
  // the longest valid segment length.
  const double dist = (dof_vals1 - dof_vals0).norm();
  long cnt = 2;
  if (dist > collision_check_config_.longest_valid_segment_length)
  {
    // Calculate the number state to interpolate
    cnt = static_cast<long>(std::ceil(dist / collision_check_config_.longest_valid_segment_length)) + 1;
  }

  // Create interpolated trajectory between two states that satisfies the longest valid segment length.
  tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
  for (long i = 0; i < dof_vals0.size(); ++i)
    subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

  // Perform casted collision checking for sub trajectory and store results in contacts_vector
  tesseract_collision::ContactResultMap contacts{ dist_results };
  const int last_state_idx{ static_cast<int>(subtraj.rows()) - 1 };
  const double dt = 1.0 / double(last_state_idx);
  for (int i = 0; i < subtraj.rows(); ++i)
  {
    tesseract_common::TransformMap state0 = get_state_fn_(subtraj.row(i));

    for (const auto& link_name : manip_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state0[link_name]);

    contact_manager_->contactTest(contacts, collision_check_config_.contact_request);

    if (!contacts.empty())
    {
      dist_results.addInterpolatedCollisionResults(
          contacts, i, last_state_idx, manip_active_link_names_, dt, true, filter);
    }
    contacts.clear();
  }
}

trajopt_common::GradientResults
LVSDiscreteCollisionEvaluator::CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                const tesseract_collision::ContactResult& contact_results)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  const double margin = margin_data_.getCollisionMargin(contact_results.link_names[0], contact_results.link_names[1]);
  return trajopt_common::getGradient(dof_vals0, dof_vals1, contact_results, margin, margin_buffer_, *manip_);
}

double LVSDiscreteCollisionEvaluator::GetCollisionMarginBuffer() const { return margin_buffer_; }

const tesseract_common::CollisionMarginData& LVSDiscreteCollisionEvaluator::GetCollisionMarginData() const
{
  return margin_data_;
}

const trajopt_common::CollisionCoeffData& LVSDiscreteCollisionEvaluator::GetCollisionCoeffData() const
{
  return coeff_data_;
}

}  // namespace trajopt_ifopt
