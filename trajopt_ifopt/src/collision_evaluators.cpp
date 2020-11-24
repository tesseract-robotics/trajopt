/**
 * @file collision_evaluators.cpp
 * @brief Contains evaluators for the collision constraint
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

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
#include <console_bridge/console.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_evaluators.h>

namespace trajopt
{
TrajOptCollisionConfig::TrajOptCollisionConfig(double margin, double coeff)
  : CollisionCheckConfig(margin), collision_coeff_data(coeff)
{
}

void CollisionsToDistances(const tesseract_collision::ContactResultVector& dist_results, std::vector<double>& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());
  for (const auto& dist_result : dist_results)
    dists.push_back(dist_result.distance);
}

void DebugPrintInfo(const tesseract_collision::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals,
                    bool header = false)
{
  if (header)
  {
    std::printf("\n");
    std::printf("DistanceResult| %30s | %30s | %6s | %6s, %6s, %6s | %6s, %6s, "
                "%6s | %6s, %6s, %6s | %6s, %6s, %6s | %6s, %6s, %6s | %10s %10s |",
                "LINK A",
                "LINK B",
                "DIST",
                "Nx",
                "Ny",
                "Nz",
                "PAx",
                "PAy",
                "PAz",
                "PBx",
                "PBy",
                "PBz",
                "LPAx",
                "LPAy",
                "LPAz",
                "LPBx",
                "LPBy",
                "LPBz",
                "CC TIME A",
                "CC TIME B");

    for (auto i = 0; i < dist_grad_A.size(); ++i)
    {
      if (i == dist_grad_A.size() - 1)
      {
        std::printf(" %6s |", ("dA" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("dA" + std::to_string(i)).c_str());
      }
    }

    for (auto i = 0; i < dist_grad_B.size(); ++i)
    {
      if (i == dist_grad_B.size() - 1)
      {
        std::printf(" %6s |", ("dB" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("dB" + std::to_string(i)).c_str());
      }
    }

    for (auto i = 0; i < dof_vals.size(); ++i)
    {
      if (i == dof_vals.size() - 1)
      {
        std::printf(" %6s |", ("J" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("J" + std::to_string(i)).c_str());
      }
    }

    std::printf("\n");
  }

  std::printf("DistanceResult| %30s | %30s | %6.3f | %6.3f, %6.3f, %6.3f | "
              "%6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %6.3f, "
              "%6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %10.3f %10.3f |",
              res.link_names[0].c_str(),
              res.link_names[1].c_str(),
              res.distance,
              res.normal(0),
              res.normal(1),
              res.normal(2),
              res.nearest_points[0](0),
              res.nearest_points[0](1),
              res.nearest_points[0](2),
              res.nearest_points[1](0),
              res.nearest_points[1](1),
              res.nearest_points[1](2),
              res.nearest_points_local[0](0),
              res.nearest_points_local[0](1),
              res.nearest_points_local[0](2),
              res.nearest_points_local[1](0),
              res.nearest_points_local[1](1),
              res.nearest_points_local[1](2),
              res.cc_time[0],
              res.cc_time[1]);

  for (auto i = 0; i < dist_grad_A.size(); ++i)
  {
    if (i == dist_grad_A.size() - 1)
    {
      std::printf(" %6.3f |", dist_grad_A(i));
    }
    else
    {
      std::printf(" %6.3f,", dist_grad_A(i));
    }
  }

  for (auto i = 0; i < dist_grad_B.size(); ++i)
  {
    if (i == dist_grad_B.size() - 1)
    {
      std::printf(" %6.3f |", dist_grad_B(i));
    }
    else
    {
      std::printf(" %6.3f,", dist_grad_B(i));
    }
  }

  for (auto i = 0; i < dof_vals.size(); ++i)
  {
    if (i == dof_vals.size() - 1)
    {
      std::printf(" %6.3f |", dof_vals(i));
    }
    else
    {
      std::printf(" %6.3f,", dof_vals(i));
    }
  }

  std::printf("\n");
}

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals,
                                                const tesseract_collision::ContactResult& contact_result,
                                                const Eigen::Vector2d& data,
                                                bool isTimestep1)
{
  GradientResults results(data);
  for (std::size_t i = 0; i < 2; ++i)
  {
    tesseract_environment::AdjacencyMapPair::ConstPtr it = adjacency_map_->getLinkMapping(contact_result.link_names[i]);
    if (it != nullptr)
    {
      results.gradients[i].has_gradient = true;

      Eigen::MatrixXd jac;
      jac.resize(6, manip_->numJoints());

      // Calculate Jacobian
      manip_->calcJacobian(jac, dofvals, it->link_name);

      // Need to change the base and ref point of the jacobian.
      // When changing ref point you must provide a vector from the current ref
      // point to the new ref point.
      results.gradients[i].scale = 1;
      Eigen::Isometry3d link_transform = contact_result.transform[i];
      if (contact_result.cc_type[i] != tesseract_collision::ContinuousCollisionType::CCType_None)
      {
        assert(contact_result.cc_time[i] >= 0.0 && contact_result.cc_time[i] <= 1.0);
        results.gradients[i].scale = (isTimestep1) ? contact_result.cc_time[i] : (1 - contact_result.cc_time[i]);
        link_transform = (isTimestep1) ? contact_result.cc_transform[i] : contact_result.transform[i];
      }
      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
      tesseract_kinematics::jacobianChangeRefPoint(jac,
                                                   (link_transform * it->transform.inverse()).linear() *
                                                       (it->transform * contact_result.nearest_points_local[i]));

      //      Eigen::Isometry3d test_link_transform, temp1, temp2;
      //      manip_->calcFwdKin(test_link_transform, dofvals, it->link_name);
      //      temp1 = world_to_base_ * test_link_transform;
      //      temp2 = link_transform * it->transform.inverse();
      //      assert(temp1.isApprox(temp2, 0.0001));

      //      Eigen::MatrixXd jac_test;
      //      jac_test.resize(6, manip_->numJoints());
      //      tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, it->link_name,
      //      contact_result.nearest_points_local[i]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);

      results.gradients[i].gradient = ((i == 0) ? -1.0 : 1.0) * contact_result.normal.transpose() * jac.topRows(3);
    }
  }
  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));

  return results;
}

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals,
                                                const tesseract_collision::ContactResult& contact_result,
                                                bool isTimestep1)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  double dist = collision_config_.collision_margin_data.getPairCollisionMarginData(contact_result.link_names[0],
                                                                                   contact_result.link_names[1]);
  double coeff = collision_config_.collision_coeff_data.getPairCollisionCoeff(contact_result.link_names[0],
                                                                              contact_result.link_names[1]);
  const Eigen::Vector2d data = { dist, coeff };

  return GetGradient(dofvals, contact_result, data, isTimestep1);
}

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals0,
                                                const Eigen::VectorXd& dofvals1,
                                                const tesseract_collision::ContactResult& contact_result,
                                                const Eigen::Vector2d& data,
                                                bool isTimestep1)
{
  GradientResults results(data);
  Eigen::VectorXd dofvalst = Eigen::VectorXd::Zero(dofvals0.size());
  for (std::size_t i = 0; i < 2; ++i)
  {
    tesseract_environment::AdjacencyMapPair::ConstPtr it = adjacency_map_->getLinkMapping(contact_result.link_names[i]);
    if (it != nullptr)
    {
      results.gradients[i].has_gradient = true;

      Eigen::MatrixXd jac;
      jac.resize(6, manip_->numJoints());

      if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
        dofvalst = dofvals0;
      else if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
        dofvalst = dofvals1;
      else
        dofvalst = dofvals0 + (dofvals1 - dofvals0) * contact_result.cc_time[i];

      // Calculate Jacobian
      manip_->calcJacobian(jac, dofvalst, it->link_name);

      // Need to change the base and ref point of the jacobian.
      // When changing ref point you must provide a vector from the current ref
      // point to the new ref point.
      results.gradients[i].scale = 1;
      Eigen::Isometry3d link_transform = contact_result.transform[i];

      assert(contact_result.cc_time[i] >= 0.0 && contact_result.cc_time[i] <= 1.0);
      results.gradients[i].scale = (isTimestep1) ? contact_result.cc_time[i] : (1 - contact_result.cc_time[i]);
      link_transform = (isTimestep1) ? contact_result.cc_transform[i] : contact_result.transform[i];

      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
      tesseract_kinematics::jacobianChangeRefPoint(jac,
                                                   (link_transform * it->transform.inverse()).linear() *
                                                       (it->transform * contact_result.nearest_points_local[i]));

      //      Eigen::Isometry3d test_link_transform, temp1, temp2;
      //      manip_->calcFwdKin(test_link_transform, dofvalst, it->link_name);
      //      temp1 = world_to_base_ * test_link_transform;
      //      temp2 = link_transform * it->transform.inverse();
      //      assert(temp1.isApprox(temp2, 0.0001));

      //      Eigen::MatrixXd jac_test;
      //      jac_test.resize(6, manip_->numJoints());
      //      tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvalst, it->link_name,
      //      contact_result.nearest_points_local[i]);
      //      bool check = jac.isApprox(jac_test, 1e-3);
      //      assert(check == true);

      results.gradients[i].gradient = ((i == 0) ? -1.0 : 1.0) * contact_result.normal.transpose() * jac.topRows(3);
    }
  }

  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));

  return results;
}

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals0,
                                                const Eigen::VectorXd& dofvals1,
                                                const tesseract_collision::ContactResult& contact_result,
                                                bool isTimestep1)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  double dist = collision_config_.collision_margin_data.getPairCollisionMarginData(contact_result.link_names[0],
                                                                                   contact_result.link_names[1]);
  double coeff = collision_config_.collision_coeff_data.getPairCollisionCoeff(contact_result.link_names[0],
                                                                              contact_result.link_names[1]);
  const Eigen::Vector2d data = { dist, coeff };

  return GetGradient(dofvals0, dofvals1, contact_result, data, isTimestep1);
}

CollisionEvaluator::CollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                       tesseract_environment::Environment::ConstPtr env,
                                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                       const Eigen::Isometry3d& world_to_base,
                                       const TrajOptCollisionConfig& collision_config,
                                       bool dynamic_environment)
  : manip_(std::move(manip))
  , env_(std::move(env))
  , adjacency_map_(std::move(adjacency_map))
  , world_to_base_(world_to_base)
  , collision_config_(std::move(collision_config))
  , state_solver_(env_->getStateSolver())
  , dynamic_environment_(dynamic_environment)
{
  // If the environment is not expected to change, then the cloned state solver may be used each time.
  if (dynamic_environment_)
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return env_->getState(joint_names, joint_values);
    };
  else
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return state_solver_->getState(joint_names, joint_values);
    };
}

void CollisionEvaluator::CalcDists(const std::vector<double>& x, std::vector<double>& dists)
{
  tesseract_collision::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

void CollisionEvaluator::CalcCollisions(const std::vector<double>& x,
                                        tesseract_collision::ContactResultMap& dist_map,
                                        tesseract_collision::ContactResultVector& dist_vector)
{
  CalcCollisions(x, dist_map);
  tesseract_collision::flattenCopyResults(dist_map, dist_vector);
}

inline size_t hash(const std::vector<double>& x) { return boost::hash_range(x.begin(), x.end()); }
void CollisionEvaluator::GetCollisionsCached(const std::vector<double>& x,
                                             tesseract_collision::ContactResultVector& dist_results)
{
  size_t key = hash(x);
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
    CalcCollisions(x, dist_map, dist_results);
    m_cache.put(key, std::make_pair(dist_map, dist_results));
  }
}

void CollisionEvaluator::GetCollisionsCached(const std::vector<double>& x,
                                             tesseract_collision::ContactResultMap& dist_results)
{
  size_t key = hash(x);
  auto it = m_cache.get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    dist_results = it->first;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Not using cached collision check");
    tesseract_collision::ContactResultVector dist_vector;
    CalcCollisions(x, dist_results, dist_vector);
    m_cache.put(key, std::make_pair(dist_results, dist_vector));
  }
}

void CollisionEvaluator::processInterpolatedCollisionResults(
    std::vector<tesseract_collision::ContactResultMap>& contacts_vector,
    tesseract_collision::ContactResultMap& contact_results,
    double dt) const
{
  // If contact is found the actual dt between the original two state must be recalculated based on where it
  // occured in the subtrajectory. Also the cc_type must also be recalculated but does not appear to be used
  // currently by trajopt.
  const std::vector<std::string>& active_links = adjacency_map_->getActiveLinkNames();
  for (size_t i = 0; i < contacts_vector.size(); ++i)
  {
    for (auto& pair : contacts_vector[i])
    {
      auto p = contact_results.find(pair.first);

      // Contains the contact distance threshold and coefficient for the given link pair
      double dist =
          collision_config_.collision_margin_data.getPairCollisionMarginData(pair.first.first, pair.first.second);
      double coeff = collision_config_.collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
      const Eigen::Vector2d data = { dist, coeff };

      // Update cc_time and cc_type
      for (auto& r : pair.second)
      {
        // Iterate over the two time values in r.cc_time
        for (size_t j = 0; j < 2; ++j)
        {
          if (std::find(active_links.begin(), active_links.end(), r.link_names[j]) != active_links.end())
          {
            r.cc_time[j] = (r.cc_time[j] < 0) ? (static_cast<double>(i) * dt) :
                                                (static_cast<double>(i) * dt) + (r.cc_time[j] * dt);
            assert(r.cc_time[j] >= 0.0 && r.cc_time[j] <= 1.0);
            if (i == 0 && r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time0;
            else if (i == (contacts_vector.size() - 1) &&
                     r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time1;
            else
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Between;
          }
        }
      }

      // Dont include contacts at the fixed state
      removeInvalidContactResults(pair.second, data);

      // If the contact pair does not exist in contact_results add it
      if (p == contact_results.end() && !pair.second.empty())
      {
        contact_results[pair.first] = pair.second;
      }
      else
      {
        // Note: Must include all contacts throughout the trajectory so the optimizer has all the information
        //      to understand how to adjust the start and end state to move it out of collision. Originally tried
        //      keeping the worst case only but ran into edge cases where this does not work in the units tests.

        // If it exists then add addition contacts to the contact_results pair
        for (auto& r : pair.second)
          p->second.push_back(r);
      }
    }
  }
}

void CollisionEvaluator::removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results,
                                                     const Eigen::Vector2d& pair_data) const
{
  auto end = std::remove_if(
      contact_results.begin(), contact_results.end(), [=, &pair_data](const tesseract_collision::ContactResult& r) {
        switch (evaluator_type_)
        {
          case CollisionExpressionEvaluatorType::START_FREE_END_FREE:
          {
            break;
          }
          case CollisionExpressionEvaluatorType::START_FIXED_END_FREE:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            break;
          }
          case CollisionExpressionEvaluatorType::START_FREE_END_FIXED:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            break;
          }
          case CollisionExpressionEvaluatorType::START_FREE_END_FREE_WEIGHTED_SUM:
          {
            break;
          }
          case CollisionExpressionEvaluatorType::START_FIXED_END_FREE_WEIGHTED_SUM:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            break;
          }
          case CollisionExpressionEvaluatorType::START_FREE_END_FIXED_WEIGHTED_SUM:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            break;
          }
          default:
          {
            PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for "
                            "CollisionEvaluator::removeInvalidContactResults!");
          }
        };

        return (!((pair_data[0] + collision_config_.collision_margin_buffer) > r.distance));
      });

  contact_results.erase(end, contact_results.end());
}

DiscreteCollisionEvaluator::DiscreteCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                                       tesseract_environment::Environment::ConstPtr env,
                                                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                                       const Eigen::Isometry3d& world_to_base,
                                                       const TrajOptCollisionConfig& collision_config,
                                                       bool dynamic_environment)
  : CollisionEvaluator(std::move(manip),
                       std::move(env),
                       std::move(adjacency_map),
                       world_to_base,
                       std::move(collision_config),
                       dynamic_environment)
{
  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
  contact_manager_->setCollisionMarginData(collision_config.collision_margin_data);
}

void DiscreteCollisionEvaluator::CalcCollisions(const std::vector<double>& x,
                                                tesseract_collision::ContactResultMap& dist_results)
{
  const Eigen::Map<const Eigen::VectorXd> joint_vals(x.data(), static_cast<Eigen::Index>(x.size()));
  CalcCollisions(joint_vals, dist_results);
}

void DiscreteCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                                                tesseract_collision::ContactResultMap& dist_results)
{
  tesseract_environment::EnvState::Ptr state = get_state_fn_(manip_->getJointNames(), dof_vals);

  for (const auto& link_name : env_->getActiveLinkNames())
    contact_manager_->setCollisionObjectsTransform(link_name, state->link_transforms[link_name]);

  contact_manager_->contactTest(dist_results, collision_config_.contact_request);

  for (auto& pair : dist_results)
  {
    // Contains the contact distance threshold and coefficient for the given link pair
    double dist =
        collision_config_.collision_margin_data.getPairCollisionMarginData(pair.first.first, pair.first.second);
    double coeff = collision_config_.collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
    const Eigen::Vector2d data = { dist, coeff };
    auto end = std::remove_if(
        pair.second.begin(), pair.second.end(), [&data, this](const tesseract_collision::ContactResult& r) {
          return (!((data[0] + collision_config_.collision_margin_buffer) > r.distance));
        });
    pair.second.erase(end, pair.second.end());
  }
}

//////////////////////////////////////////

// DiscreteCollisionEvaluator::DiscreteCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
//                                                       tesseract_environment::Environment::ConstPtr env,
//                                                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
//                                                       const Eigen::Isometry3d& world_to_base,
//                                                       SafetyMarginData::ConstPtr safety_margin_data,
//                                                       tesseract_collision::ContactTestType contact_test_type,
//                                                       double longest_valid_segment_length,
//                                                       sco::VarVector vars0,
//                                                       sco::VarVector vars1,
//                                                       CollisionExpressionEvaluatorType type,
//                                                       double safety_margin_buffer)
//  : CollisionEvaluator(std::move(manip),
//                       std::move(env),
//                       std::move(adjacency_map),
//                       world_to_base,
//                       std::move(safety_margin_data),
//                       contact_test_type,
//                       longest_valid_segment_length,
//                       safety_margin_buffer)
//{
//  vars0_ = std::move(vars0);
//  vars1_ = std::move(vars1);
//  evaluator_type_ = type;

//  contact_manager_ = env_->getDiscreteContactManager();
//  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
//  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() + safety_margin_buffer_);

//  switch (evaluator_type_)
//  {
//    case CollisionExpressionEvaluatorType::START_FREE_END_FREE:
//    {
//      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsBothFree,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE:
//    {
//      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsEndFree,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED:
//    {
//      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsStartFree,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FREE_END_FREE_WEIGHTED_SUM:
//    {
//      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsBothFreeW,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE_WEIGHTED_SUM:
//    {
//      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsEndFreeW,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED_WEIGHTED_SUM:
//    {
//      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsStartFreeW,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    default:
//    {
//      PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for DiscreteCollisionEvaluator!");
//    }
//  };
//}

// void DiscreteCollisionEvaluator::CalcCollisions(const std::vector<double>& x, tesseract_collision::ContactResultMap&
// dist_results)
//{
//  Eigen::VectorXd s0 = sco::getVec(x, vars0_);
//  Eigen::VectorXd s1 = sco::getVec(x, vars1_);
//  CalcCollisions(s0, s1, dist_results);
//}

// void DiscreteCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
//                                                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
//                                                tesseract_collision::ContactResultMap& dist_results)
//{
//  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
//  // the collision checking is broken up into multiple casted collision checks such that each check is less then
//  // the longest valid segment length.
//  double dist = (dof_vals1 - dof_vals0).norm();
//  long cnt = 2;
//  if (dist > longest_valid_segment_length_)
//  {
//    // Calculate the number state to interpolate
//    cnt = static_cast<long>(std::ceil(dist / longest_valid_segment_length_)) + 1;
//  }

//  // Get active link names
//  const std::vector<std::string>& active_links = adjacency_map_->getActiveLinkNames();

//  // Create interpolated trajectory between two states that satisfies the longest valid segment length.
//  tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
//  for (long i = 0; i < dof_vals0.size(); ++i)
//    subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

//  // Perform casted collision checking for sub trajectory and store results in contacts_vector
//  std::vector<tesseract_collision::ContactResultMap> contacts_vector;
//  contacts_vector.reserve(static_cast<size_t>(subtraj.rows()));
//  bool contact_found = false;
//  for (int i = 0; i < subtraj.rows(); ++i)
//  {
//    tesseract_collision::ContactResultMap contacts;
//    tesseract_environment::EnvState::Ptr state0 = state_solver_->getState(manip_->getJointNames(), subtraj.row(i));

//    for (const auto& link_name : active_links)
//      contact_manager_->setCollisionObjectsTransform(link_name, state0->link_transforms[link_name]);

//    contact_manager_->contactTest(contacts, contact_test_type_);
//    if (!contacts.empty())
//      contact_found = true;

//    contacts_vector.push_back(contacts);
//  }

//  if (contact_found)
//    processInterpolatedCollisionResults(contacts_vector, dist_results, 1.0 / double(subtraj.rows() - 1));
//}

// void DiscreteCollisionEvaluator::CalcDistExpressions(const std::vector<double>& x,
//                                                     sco::AffExprVector& exprs,
//                                                     AlignedVector<Eigen::Vector2d>& exprs_data)
//{
//  fn_(x, exprs, exprs_data);
//}

// void DiscreteCollisionEvaluator::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const
// std::vector<double>& x)
//{
//  tesseract_collision::ContactResultVector dist_results;
//  GetCollisionsCached(x, dist_results);
//  Eigen::VectorXd dofvals0 = sco::getVec(x, vars0_);
//  Eigen::VectorXd dofvals1 = sco::getVec(x, vars1_);

//  Eigen::VectorXd safety_distance(dist_results.size());
//  for (auto i = 0u; i < dist_results.size(); ++i)
//  {
//    tesseract_collision::ContactResult& res = dist_results[i];
//    // Contains the contact distance threshold and coefficient for the given link pair
//    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0],
//    res.link_names[1]); safety_distance[i] = data[0];

//    tesseract_environment::AdjacencyMapPair::ConstPtr itA = adjacency_map_->getLinkMapping(res.link_names[0]);
//    if (itA != nullptr)
//    {
//      Eigen::MatrixXd jac;
//      Eigen::VectorXd dist_grad;
//      Eigen::Isometry3d pose, pose2, pose3;
//      jac.resize(6, manip_->numJoints());
//      manip_->calcFwdKin(pose, dofvals0, itA->link_name);
//      pose = world_to_base_ * pose;

//      // For descrete continuous we need to populate cc_transform for plotting to work correctly
//      manip_->calcFwdKin(pose3, dofvals1, itA->link_name);
//      res.cc_transform[0] = world_to_base_ * pose3;

//      manip_->calcJacobian(jac, dofvals0, itA->link_name);
//      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
//      tesseract_kinematics::jacobianChangeRefPoint(jac, pose.linear() * res.nearest_points_local[0]);

//      // Eigen::MatrixXd jac_test;
//      // jac_test.resize(6, manip_->numJoints());
//      // tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itA->link_name,
//      // res.nearest_points_local[0]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);

//      dist_grad = -res.normal.transpose() * jac.topRows(3);

//      manip_->calcFwdKin(pose2, dofvals0 + dist_grad, itA->link_name);
//      pose2 = world_to_base_ * pose2 * itA->transform;
//      plotter->plotArrow(
//          res.nearest_points[0], pose2 * res.nearest_points_local[0], Eigen::Vector4d(1, 1, 1, 1), 0.005);
//    }

//    tesseract_environment::AdjacencyMapPair::ConstPtr itB = adjacency_map_->getLinkMapping(res.link_names[1]);
//    if (itB != nullptr)
//    {
//      Eigen::MatrixXd jac;
//      Eigen::VectorXd dist_grad;
//      Eigen::Isometry3d pose, pose2, pose3;
//      jac.resize(6, manip_->numJoints());
//      manip_->calcFwdKin(pose, dofvals0, itB->link_name);
//      pose = world_to_base_ * pose;

//      // For descrete continuous we need to populate cc_transform for plotting to work correctly
//      manip_->calcFwdKin(pose3, dofvals1, itB->link_name);
//      res.cc_transform[1] = world_to_base_ * pose3;

//      // Calculate Jacobian
//      manip_->calcJacobian(jac, dofvals0, itB->link_name);
//      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
//      tesseract_kinematics::jacobianChangeRefPoint(jac, pose.linear() * res.nearest_points_local[1]);

//      // Eigen::MatrixXd jac_test;
//      // jac_test.resize(6, manip_->numJoints());
//      // tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itB->link_name,
//      // res.nearest_points_local[1]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true)

//      dist_grad = res.normal.transpose() * jac.topRows(3);

//      manip_->calcFwdKin(pose2, dofvals0 + dist_grad, itB->link_name);
//      pose2 = world_to_base_ * pose2 * itB->transform;
//      plotter->plotArrow(
//          res.nearest_points[1], pose2 * res.nearest_points_local[1], Eigen::Vector4d(1, 1, 1, 1), 0.005);
//    }
//  }

//  plotter->plotContactResults(adjacency_map_->getActiveLinkNames(), dist_results, safety_distance);
//}

//////////////////////////////////////////

// CastCollisionEvaluator::CastCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
//                                               tesseract_environment::Environment::ConstPtr env,
//                                               tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
//                                               const Eigen::Isometry3d& world_to_base,
//                                               SafetyMarginData::ConstPtr safety_margin_data,
//                                               tesseract_collision::ContactTestType contact_test_type,
//                                               double longest_valid_segment_length,
//                                               sco::VarVector vars0,
//                                               sco::VarVector vars1,
//                                               CollisionExpressionEvaluatorType type,
//                                               double safety_margin_buffer)
//  : CollisionEvaluator(std::move(manip),
//                       std::move(env),
//                       std::move(adjacency_map),
//                       world_to_base,
//                       std::move(safety_margin_data),
//                       contact_test_type,
//                       longest_valid_segment_length,
//                       safety_margin_buffer)
//{
//  vars0_ = std::move(vars0);
//  vars1_ = std::move(vars1);
//  evaluator_type_ = type;

//  contact_manager_ = env_->getContinuousContactManager();
//  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
//  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() + safety_margin_buffer_);

//  switch (evaluator_type_)
//  {
//    case CollisionExpressionEvaluatorType::START_FREE_END_FREE:
//    {
//      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsBothFree,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE:
//    {
//      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsEndFree,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED:
//    {
//      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsStartFree,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FREE_END_FREE_WEIGHTED_SUM:
//    {
//      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsBothFreeW,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE_WEIGHTED_SUM:
//    {
//      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsEndFreeW,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED_WEIGHTED_SUM:
//    {
//      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsStartFreeW,
//                      this,
//                      std::placeholders::_1,
//                      std::placeholders::_2,
//                      std::placeholders::_3);
//      break;
//    }
//    default:
//    {
//      PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for CastCollisionEvaluator!");
//    }
//  };
//}

// void CastCollisionEvaluator::CalcCollisions(const std::vector<double>& x, tesseract_collision::ContactResultMap&
// dist_results)
//{
//  Eigen::VectorXd s0 = sco::getVec(x, vars0_);
//  Eigen::VectorXd s1 = sco::getVec(x, vars1_);
//  CalcCollisions(s0, s1, dist_results);
//}

// void CastCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
//                                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
//                                            tesseract_collision::ContactResultMap& dist_results)
//{
//  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
//  // the collision checking is broken up into multiple casted collision checks such that each check is less then
//  // the longest valid segment length.
//  double dist = (dof_vals1 - dof_vals0).norm();
//  if (dist > longest_valid_segment_length_)
//  {
//    // Calculate the number state to interpolate
//    long cnt = static_cast<long>(std::ceil(dist / longest_valid_segment_length_)) + 1;

//    // Create interpolated trajectory between two states that satisfies the longest valid segment length.
//    tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
//    for (long i = 0; i < dof_vals0.size(); ++i)
//      subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

//    // Perform casted collision checking for sub trajectory and store results in contacts_vector
//    std::vector<tesseract_collision::ContactResultMap> contacts_vector;
//    contacts_vector.reserve(static_cast<size_t>(subtraj.rows()));
//    bool contact_found = false;
//    for (int i = 0; i < subtraj.rows() - 1; ++i)
//    {
//      tesseract_collision::ContactResultMap contacts;
//      tesseract_environment::EnvState::Ptr state0 = state_solver_->getState(manip_->getJointNames(), subtraj.row(i));
//      tesseract_environment::EnvState::Ptr state1 =
//          state_solver_->getState(manip_->getJointNames(), subtraj.row(i + 1));

//      for (const auto& link_name : adjacency_map_->getActiveLinkNames())
//        contact_manager_->setCollisionObjectsTransform(
//            link_name, state0->link_transforms[link_name], state1->link_transforms[link_name]);

//      contact_manager_->contactTest(contacts, contact_test_type_);
//      if (!contacts.empty())
//        contact_found = true;

//      contacts_vector.push_back(contacts);
//    }

//    if (contact_found)
//      processInterpolatedCollisionResults(contacts_vector, dist_results, 1.0 / double(subtraj.rows() - 1));
//  }
//  else
//  {
//    tesseract_environment::EnvState::Ptr state0 = state_solver_->getState(manip_->getJointNames(), dof_vals0);
//    tesseract_environment::EnvState::Ptr state1 = state_solver_->getState(manip_->getJointNames(), dof_vals1);
//    for (const auto& link_name : adjacency_map_->getActiveLinkNames())
//      contact_manager_->setCollisionObjectsTransform(
//          link_name, state0->link_transforms[link_name], state1->link_transforms[link_name]);

//    contact_manager_->contactTest(dist_results, contact_test_type_);

//    // Dont include contacts at the fixed state
//    for (auto& pair : dist_results)
//    {
//      // Contains the contact distance threshold and coefficient for the given link pair
//      const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(pair.first.first,
//      pair.first.second); removeInvalidContactResults(pair.second, data);
//    }
//  }
//}

// void CastCollisionEvaluator::CalcDistExpressions(const std::vector<double>& x,
//                                                 sco::AffExprVector& exprs,
//                                                 AlignedVector<Eigen::Vector2d>& exprs_data)
//{
//  fn_(x, exprs, exprs_data);
//}

// void CastCollisionEvaluator::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const
// std::vector<double>& x)
//{
//  tesseract_collision::ContactResultVector dist_results;
//  GetCollisionsCached(x, dist_results);
//  Eigen::VectorXd dofvals = sco::getVec(x, vars0_);

//  Eigen::VectorXd safety_distance(dist_results.size());
//  for (auto i = 0u; i < dist_results.size(); ++i)
//  {
//    const tesseract_collision::ContactResult& res = dist_results[i];
//    // Contains the contact distance threshold and coefficient for the given link pair
//    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0],
//    res.link_names[1]); safety_distance[i] = data[0];

//    tesseract_environment::AdjacencyMapPair::ConstPtr itA = adjacency_map_->getLinkMapping(res.link_names[0]);
//    if (itA != nullptr)
//    {
//      Eigen::MatrixXd jac;
//      Eigen::VectorXd dist_grad;
//      Eigen::Isometry3d pose, pose2;
//      jac.resize(6, manip_->numJoints());
//      manip_->calcFwdKin(pose, dofvals, itA->link_name);
//      pose = world_to_base_ * pose;

//      manip_->calcJacobian(jac, dofvals, itA->link_name);
//      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
//      tesseract_kinematics::jacobianChangeRefPoint(jac, pose.linear() * res.nearest_points_local[0]);

//      // Eigen::MatrixXd jac_test;
//      // jac_test.resize(6, manip_->numJoints());
//      // tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itA->link_name,
//      // res.nearest_points_local[0]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);

//      dist_grad = -res.normal.transpose() * jac.topRows(3);

//      manip_->calcFwdKin(pose2, dofvals + dist_grad, itA->link_name);
//      pose2 = world_to_base_ * pose2 * itA->transform;
//      plotter->plotArrow(
//          res.nearest_points[0], pose2 * res.nearest_points_local[0], Eigen::Vector4d(1, 1, 1, 1), 0.005);
//    }

//    tesseract_environment::AdjacencyMapPair::ConstPtr itB = adjacency_map_->getLinkMapping(res.link_names[1]);
//    if (itB != nullptr)
//    {
//      Eigen::MatrixXd jac;
//      Eigen::VectorXd dist_grad;
//      Eigen::Isometry3d pose, pose2;
//      jac.resize(6, manip_->numJoints());
//      manip_->calcFwdKin(pose, dofvals, itB->link_name);
//      pose = world_to_base_ * pose;

//      // Calculate Jacobian
//      manip_->calcJacobian(jac, dofvals, itB->link_name);
//      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
//      tesseract_kinematics::jacobianChangeRefPoint(jac, pose.linear() * res.nearest_points_local[1]);

//      // Eigen::MatrixXd jac_test;
//      // jac_test.resize(6, manip_->numJoints());
//      // tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itB->link_name,
//      // res.nearest_points_local[1]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true)

//      dist_grad = res.normal.transpose() * jac.topRows(3);

//      manip_->calcFwdKin(pose2, dofvals + dist_grad, itB->link_name);
//      pose2 = world_to_base_ * pose2 * itB->transform;
//      plotter->plotArrow(
//          res.nearest_points[1], pose2 * res.nearest_points_local[1], Eigen::Vector4d(1, 1, 1, 1), 0.005);
//    }
//  }

//  plotter->plotContactResults(adjacency_map_->getActiveLinkNames(), dist_results, safety_distance);
//}

////////////////////////////////////////////

// CollisionCost::CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
//                             tesseract_environment::Environment::ConstPtr env,
//                             tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
//                             const Eigen::Isometry3d& world_to_base,
//                             SafetyMarginData::ConstPtr safety_margin_data,
//                             tesseract_collision::ContactTestType contact_test_type,
//                             sco::VarVector vars,
//                             CollisionExpressionEvaluatorType type,
//                             double safety_margin_buffer)
//  : Cost("collision")
//{
//  m_calc = std::make_shared<DiscreteCollisionEvaluator>(std::move(manip),
//                                                              std::move(env),
//                                                              std::move(adjacency_map),
//                                                              world_to_base,
//                                                              std::move(safety_margin_data),
//                                                              contact_test_type,
//                                                              std::move(vars),
//                                                              type,
//                                                              safety_margin_buffer);
//}

// CollisionCost::CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
//                             tesseract_environment::Environment::ConstPtr env,
//                             tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
//                             const Eigen::Isometry3d& world_to_base,
//                             SafetyMarginData::ConstPtr safety_margin_data,
//                             tesseract_collision::ContactTestType contact_test_type,
//                             double longest_valid_segment_length,
//                             sco::VarVector vars0,
//                             sco::VarVector vars1,
//                             CollisionExpressionEvaluatorType type,
//                             bool discrete,
//                             double safety_margin_buffer)
//{
//  if (discrete)
//  {
//    name_ = "discrete_continuous_collision";
//    m_calc = std::make_shared<DiscreteCollisionEvaluator>(std::move(manip),
//                                                          std::move(env),
//                                                          std::move(adjacency_map),
//                                                          world_to_base,
//                                                          std::move(safety_margin_data),
//                                                          contact_test_type,
//                                                          longest_valid_segment_length,
//                                                          std::move(vars0),
//                                                          std::move(vars1),
//                                                          type,
//                                                          safety_margin_buffer);
//  }
//  else
//  {
//    name_ = "cast_continuous_collision";
//    m_calc = std::make_shared<CastCollisionEvaluator>(std::move(manip),
//                                                      std::move(env),
//                                                      std::move(adjacency_map),
//                                                      world_to_base,
//                                                      std::move(safety_margin_data),
//                                                      contact_test_type,
//                                                      longest_valid_segment_length,
//                                                      std::move(vars0),
//                                                      std::move(vars1),
//                                                      type,
//                                                      safety_margin_buffer);
//  }
//}

// sco::ConvexObjective::Ptr CollisionCost::convex(const sco::std::vector<double>& x, sco::Model* model)
//{
//  sco::ConvexObjective::Ptr out(new sco::ConvexObjective(model));
//  sco::AffExprVector exprs;
//  AlignedVector<Eigen::Vector2d> exprs_data;

//  m_calc->CalcDistExpressions(x, exprs, exprs_data);
//  assert(exprs.size() == exprs_data.size());

//  tesseract_collision::ContactResultVector dist_results;
//  m_calc->GetCollisionsCached(x, dist_results);
//  for (std::size_t i = 0; i < exprs.size(); ++i)
//  {
//    // Contains the contact distance threshold and coefficient for the given link pair
//    const Eigen::Vector2d& data = exprs_data[i];

//    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
//    out->addHinge(viol, data[1]);
//  }
//  return out;
//}

// double CollisionCost::value(const sco::std::vector<double>& x)
//{
//  std::vector<double> dists;
//  m_calc->CalcDists(x, dists);

//  tesseract_collision::ContactResultVector dist_results;
//  m_calc->GetCollisionsCached(x, dist_results);
//  double out = 0;
//  for (std::size_t i = 0; i < dists.size(); ++i)
//  {
//    // Contains the contact distance threshold and coefficient for the given link pair
//    const Eigen::Vector2d& data =
//    m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
//                                                                                         dist_results[i].link_names[1]);
//    out += sco::pospart(data[0] - dists[i]) * data[1];
//  }
//  return out;
//}

// void CollisionCost::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const std::vector<double>& x)
//{
//  m_calc->Plot(plotter, x);
//}
//// ALMOST EXACTLY COPIED FROM CollisionCost

// CollisionConstraint::CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
//                                         tesseract_environment::Environment::ConstPtr env,
//                                         tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
//                                         const Eigen::Isometry3d& world_to_base,
//                                         SafetyMarginData::ConstPtr safety_margin_data,
//                                         tesseract_collision::ContactTestType contact_test_type,
//                                         sco::VarVector vars,
//                                         CollisionExpressionEvaluatorType type,
//                                         double safety_margin_buffer)
//{
//  name_ = "collision";
//  m_calc = std::make_shared<DiscreteCollisionEvaluator>(std::move(manip),
//                                                              std::move(env),
//                                                              std::move(adjacency_map),
//                                                              world_to_base,
//                                                              std::move(safety_margin_data),
//                                                              contact_test_type,
//                                                              std::move(vars),
//                                                              type,
//                                                              safety_margin_buffer);
//}

// CollisionConstraint::CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
//                                         tesseract_environment::Environment::ConstPtr env,
//                                         tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
//                                         const Eigen::Isometry3d& world_to_base,
//                                         SafetyMarginData::ConstPtr safety_margin_data,
//                                         tesseract_collision::ContactTestType contact_test_type,
//                                         double longest_valid_segment_length,
//                                         sco::VarVector vars0,
//                                         sco::VarVector vars1,
//                                         CollisionExpressionEvaluatorType type,
//                                         bool discrete,
//                                         double safety_margin_buffer)
//{
//  if (discrete)
//  {
//    name_ = "discrete_continuous_collision";
//    m_calc = std::make_shared<DiscreteCollisionEvaluator>(std::move(manip),
//                                                          std::move(env),
//                                                          std::move(adjacency_map),
//                                                          world_to_base,
//                                                          std::move(safety_margin_data),
//                                                          contact_test_type,
//                                                          longest_valid_segment_length,
//                                                          std::move(vars0),
//                                                          std::move(vars1),
//                                                          type,
//                                                          safety_margin_buffer);
//  }
//  else
//  {
//    name_ = "cast_continuous_collision";
//    m_calc = std::make_shared<CastCollisionEvaluator>(std::move(manip),
//                                                      std::move(env),
//                                                      std::move(adjacency_map),
//                                                      world_to_base,
//                                                      std::move(safety_margin_data),
//                                                      contact_test_type,
//                                                      longest_valid_segment_length,
//                                                      std::move(vars0),
//                                                      std::move(vars1),
//                                                      type,
//                                                      safety_margin_buffer);
//  }
//}

// sco::ConvexConstraints::Ptr CollisionConstraint::convex(const sco::std::vector<double>& x, sco::Model* model)
//{
//  sco::ConvexConstraints::Ptr out(new sco::ConvexConstraints(model));
//  sco::AffExprVector exprs;
//  AlignedVector<Eigen::Vector2d> exprs_data;

//  m_calc->CalcDistExpressions(x, exprs, exprs_data);
//  assert(exprs.size() == exprs_data.size());

//  tesseract_collision::ContactResultVector dist_results;
//  m_calc->GetCollisionsCached(x, dist_results);
//  for (std::size_t i = 0; i < exprs.size(); ++i)
//  {
//    // Contains the contact distance threshold and coefficient for the given link pair
//    const Eigen::Vector2d& data = exprs_data[i];

//    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
//    out->addIneqCnt(sco::exprMult(viol, data[1]));
//  }
//  return out;
//}

// std::vector<double> CollisionConstraint::value(const sco::std::vector<double>& x)
//{
//  std::vector<double> dists;
//  m_calc->CalcDists(x, dists);

//  tesseract_collision::ContactResultVector dist_results;
//  m_calc->GetCollisionsCached(x, dist_results);
//  std::vector<double> out(dists.size());
//  for (std::size_t i = 0; i < dists.size(); ++i)
//  {
//    // Contains the contact distance threshold and coefficient for the given link pair
//    const Eigen::Vector2d& data =
//    m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
//                                                                                         dist_results[i].link_names[1]);

//    out[i] = sco::pospart(data[0] - dists[i]) * data[1];
//  }
//  return out;
//}
//}  // namespace trajopt
}  // namespace trajopt
