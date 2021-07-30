/**
 * @file continuous_collision_evaluators.h
 * @brief Contains continuous evaluators for the collision constraint
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

#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision_utils.h>

namespace trajopt
{
LVSContinuousCollisionEvaluator::LVSContinuousCollisionEvaluator(
    tesseract_kinematics::ForwardKinematics::ConstPtr manip,
    tesseract_environment::Environment::ConstPtr env,
    tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
    const Eigen::Isometry3d& world_to_base,
    const TrajOptCollisionConfig& collision_config,
    ContinuousCollisionEvaluatorType evaluator_type,
    bool dynamic_environment)
  : manip_(std::move(manip))
  , env_(std::move(env))
  , adjacency_map_(std::move(adjacency_map))
  , world_to_base_(world_to_base)
  , collision_config_(std::move(collision_config))
  , state_solver_(env_->getStateSolver())
  , evaluator_type_(evaluator_type)
  , dynamic_environment_(dynamic_environment)
{
  // If the environment is not expected to change, then the cloned state solver may be used each time.
  if (dynamic_environment_)
  {
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return env_->getState(joint_names, joint_values);
    };
  }
  else
  {
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return state_solver_->getState(joint_names, joint_values);
    };
  }

  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
  contact_manager_->setCollisionMarginData(collision_config_.collision_margin_data);
  // Increase the default by the buffer
  contact_manager_->setDefaultCollisionMarginData(collision_config_.collision_margin_data.getMaxCollisionMargin() +
                                                  collision_config_.collision_margin_buffer);
}

void LVSContinuousCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                     const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                     tesseract_collision::ContactResultMap& dist_results)
{
  size_t key = getHash(dof_vals0, dof_vals1);
  auto it = m_cache.get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    dist_results = it->first;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Not using cached collision check");
    CalcCollisionsHelper(dof_vals0, dof_vals1, dist_results);
    tesseract_collision::ContactResultVector dist_vector;
    tesseract_collision::flattenCopyResults(dist_results, dist_vector);
    m_cache.put(key, std::make_pair(dist_results, dist_vector));
  }
}

void LVSContinuousCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                     const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                     tesseract_collision::ContactResultVector& dist_results)
{
  size_t key = getHash(dof_vals0, dof_vals1);
  auto it = m_cache.get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    dist_results = it->second;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Not using cached collision check");
    tesseract_collision::ContactResultMap dist_map;
    CalcCollisionsHelper(dof_vals0, dof_vals1, dist_map);
    tesseract_collision::flattenCopyResults(dist_map, dist_results);
    m_cache.put(key, std::make_pair(dist_map, dist_results));
  }
}

void LVSContinuousCollisionEvaluator::CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                           const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                           tesseract_collision::ContactResultMap& dist_results)
{
  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
  // the collision checking is broken up into multiple casted collision checks such that each check is less then
  // the longest valid segment length.
  double dist = (dof_vals1 - dof_vals0).norm();
  if (dist > collision_config_.longest_valid_segment_length)
  {
    // Calculate the number state to interpolate
    long cnt = static_cast<long>(std::ceil(dist / collision_config_.longest_valid_segment_length)) + 1;

    // Create interpolated trajectory between two states that satisfies the longest valid segment length.
    tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
    for (long i = 0; i < dof_vals0.size(); ++i)
      subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

    // Perform casted collision checking for sub trajectory and store results in contacts_vector
    std::vector<tesseract_collision::ContactResultMap> contacts_vector;
    contacts_vector.reserve(static_cast<size_t>(subtraj.rows()));
    bool contact_found = false;
    for (int i = 0; i < subtraj.rows() - 1; ++i)
    {
      tesseract_collision::ContactResultMap contacts;
      tesseract_environment::EnvState::Ptr state0 = state_solver_->getState(manip_->getJointNames(), subtraj.row(i));
      tesseract_environment::EnvState::Ptr state1 =
          state_solver_->getState(manip_->getJointNames(), subtraj.row(i + 1));

      for (const auto& link_name : adjacency_map_->getActiveLinkNames())
        contact_manager_->setCollisionObjectsTransform(
            link_name, state0->link_transforms[link_name], state1->link_transforms[link_name]);

      contact_manager_->contactTest(contacts, collision_config_.contact_request);
      if (!contacts.empty())
        contact_found = true;

      contacts_vector.push_back(contacts);
    }

    if (contact_found)
      processInterpolatedCollisionResults(contacts_vector,
                                          dist_results,
                                          adjacency_map_->getActiveLinkNames(),
                                          collision_config_,
                                          evaluator_type_,
                                          1.0 / double(subtraj.rows() - 1));
  }
  else
  {
    tesseract_environment::EnvState::Ptr state0 = state_solver_->getState(manip_->getJointNames(), dof_vals0);
    tesseract_environment::EnvState::Ptr state1 = state_solver_->getState(manip_->getJointNames(), dof_vals1);
    for (const auto& link_name : adjacency_map_->getActiveLinkNames())
      contact_manager_->setCollisionObjectsTransform(
          link_name, state0->link_transforms[link_name], state1->link_transforms[link_name]);

    contact_manager_->contactTest(dist_results, collision_config_.contact_request);

    // Dont include contacts at the fixed state
    for (auto& pair : dist_results)
    {
      // Contains the contact distance threshold and coefficient for the given link pair
      double dist = collision_config_.collision_margin_data.getPairCollisionMargin(pair.first.first, pair.first.second);
      double coeff = collision_config_.collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
      const Eigen::Vector3d data = { dist, collision_config_.collision_margin_buffer, coeff };
      removeInvalidContactResults(pair.second, data, evaluator_type_);
    }
  }
}

GradientResults LVSContinuousCollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals0,
                                                             const Eigen::VectorXd& dofvals1,
                                                             const tesseract_collision::ContactResult& contact_result,
                                                             bool isTimestep1)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  double dist = collision_config_.collision_margin_data.getPairCollisionMargin(contact_result.link_names[0],
                                                                               contact_result.link_names[1]);
  double coeff = collision_config_.collision_coeff_data.getPairCollisionCoeff(contact_result.link_names[0],
                                                                              contact_result.link_names[1]);

  const Eigen::Vector3d data = { dist, collision_config_.collision_margin_buffer, coeff };

  return getGradient(dofvals0, dofvals1, contact_result, data, manip_, adjacency_map_, world_to_base_, isTimestep1);
}

TrajOptCollisionConfig& LVSContinuousCollisionEvaluator::GetCollisionConfig() { return collision_config_; }

ContinuousCollisionEvaluatorType LVSContinuousCollisionEvaluator::GetEvaluatorType() const { return evaluator_type_; }

//////////////////////////////////////////

LVSDiscreteCollisionEvaluator::LVSDiscreteCollisionEvaluator(
    tesseract_kinematics::ForwardKinematics::ConstPtr manip,
    tesseract_environment::Environment::ConstPtr env,
    tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
    const Eigen::Isometry3d& world_to_base,
    const TrajOptCollisionConfig& collision_config,
    ContinuousCollisionEvaluatorType evaluator_type,
    bool dynamic_environment)
  : manip_(std::move(manip))
  , env_(std::move(env))
  , adjacency_map_(std::move(adjacency_map))
  , world_to_base_(world_to_base)
  , collision_config_(std::move(collision_config))
  , state_solver_(env_->getStateSolver())
  , evaluator_type_(evaluator_type)
  , dynamic_environment_(dynamic_environment)
{
  // If the environment is not expected to change, then the cloned state solver may be used each time.
  if (dynamic_environment_)
  {
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return env_->getState(joint_names, joint_values);
    };
  }
  else
  {
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return state_solver_->getState(joint_names, joint_values);
    };
  }

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
  contact_manager_->setCollisionMarginData(collision_config_.collision_margin_data);
  // Increase the default by the buffer
  contact_manager_->setDefaultCollisionMarginData(collision_config_.collision_margin_data.getMaxCollisionMargin() +
                                                  collision_config_.collision_margin_buffer);
}

void LVSDiscreteCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                   tesseract_collision::ContactResultMap& dist_results)
{
  size_t key = getHash(dof_vals0, dof_vals1);
  auto it = m_cache.get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    dist_results = it->first;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Not using cached collision check");
    CalcCollisionsHelper(dof_vals0, dof_vals1, dist_results);
    tesseract_collision::ContactResultVector dist_vector;
    tesseract_collision::flattenCopyResults(dist_results, dist_vector);
    m_cache.put(key, std::make_pair(dist_results, dist_vector));
  }
}

void LVSDiscreteCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                   tesseract_collision::ContactResultVector& dist_results)
{
  size_t key = getHash(dof_vals0, dof_vals1);
  auto it = m_cache.get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    dist_results = it->second;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Not using cached collision check");
    tesseract_collision::ContactResultMap dist_map;
    CalcCollisionsHelper(dof_vals0, dof_vals1, dist_map);
    tesseract_collision::flattenCopyResults(dist_map, dist_results);
    m_cache.put(key, std::make_pair(dist_map, dist_results));
  }
}

void LVSDiscreteCollisionEvaluator::CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                         const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                         tesseract_collision::ContactResultMap& dist_results)
{
  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
  // the collision checking is broken up into multiple casted collision checks such that each check is less then
  // the longest valid segment length.
  double dist = (dof_vals1 - dof_vals0).norm();
  long cnt = 2;
  if (dist > collision_config_.longest_valid_segment_length)
  {
    // Calculate the number state to interpolate
    cnt = static_cast<long>(std::ceil(dist / collision_config_.longest_valid_segment_length)) + 1;
  }

  // Get active link names
  const std::vector<std::string>& active_links = adjacency_map_->getActiveLinkNames();

  // Create interpolated trajectory between two states that satisfies the longest valid segment length.
  tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
  for (long i = 0; i < dof_vals0.size(); ++i)
    subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

  // Perform casted collision checking for sub trajectory and store results in contacts_vector
  std::vector<tesseract_collision::ContactResultMap> contacts_vector;
  contacts_vector.reserve(static_cast<size_t>(subtraj.rows()));
  bool contact_found = false;
  for (int i = 0; i < subtraj.rows(); ++i)
  {
    tesseract_collision::ContactResultMap contacts;
    tesseract_environment::EnvState::Ptr state0 = state_solver_->getState(manip_->getJointNames(), subtraj.row(i));

    for (const auto& link_name : active_links)
      contact_manager_->setCollisionObjectsTransform(link_name, state0->link_transforms[link_name]);

    contact_manager_->contactTest(contacts, collision_config_.contact_request);
    if (!contacts.empty())
      contact_found = true;

    contacts_vector.push_back(contacts);
  }

  if (contact_found)
    processInterpolatedCollisionResults(contacts_vector,
                                        dist_results,
                                        adjacency_map_->getActiveLinkNames(),
                                        collision_config_,
                                        evaluator_type_,
                                        1.0 / double(subtraj.rows() - 1));
}

GradientResults LVSDiscreteCollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals0,
                                                           const Eigen::VectorXd& dofvals1,
                                                           const tesseract_collision::ContactResult& contact_result,
                                                           bool isTimestep1)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  double dist = collision_config_.collision_margin_data.getPairCollisionMargin(contact_result.link_names[0],
                                                                               contact_result.link_names[1]);
  double coeff = collision_config_.collision_coeff_data.getPairCollisionCoeff(contact_result.link_names[0],
                                                                              contact_result.link_names[1]);

  const Eigen::Vector3d data = { dist, collision_config_.collision_margin_buffer, coeff };

  return getGradient(dofvals0, dofvals1, contact_result, data, manip_, adjacency_map_, world_to_base_, isTimestep1);
}

TrajOptCollisionConfig& LVSDiscreteCollisionEvaluator::GetCollisionConfig() { return collision_config_; }

ContinuousCollisionEvaluatorType LVSDiscreteCollisionEvaluator::GetEvaluatorType() const { return evaluator_type_; }

}  // namespace trajopt
