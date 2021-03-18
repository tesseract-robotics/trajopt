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

      // Calculate Jacobian
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvals, it->link_name);

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
  double dist = collision_config_.collision_margin_data.getPairCollisionMargin(contact_result.link_names[0],
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

      if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
        dofvalst = dofvals0;
      else if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
        dofvalst = dofvals1;
      else
        dofvalst = dofvals0 + (dofvals1 - dofvals0) * contact_result.cc_time[i];

      // Calculate Jacobian
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvalst, it->link_name);

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
  double dist = collision_config_.collision_margin_data.getPairCollisionMargin(contact_result.link_names[0],
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
      double dist = collision_config_.collision_margin_data.getPairCollisionMargin(pair.first.first, pair.first.second);
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
    double dist = collision_config_.collision_margin_data.getPairCollisionMargin(pair.first.first, pair.first.second);
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

LVSContinuousCollisionEvaluator::LVSContinuousCollisionEvaluator(
    tesseract_kinematics::ForwardKinematics::ConstPtr manip,
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
  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
  contact_manager_->setCollisionMarginData(collision_config.collision_margin_data);
}

void LVSContinuousCollisionEvaluator::CalcCollisions(const std::vector<double>& x,
                                                     tesseract_collision::ContactResultMap& dist_results)
{
  auto dof = static_cast<Eigen::Index>(x.size()) / 2;
  const Eigen::Map<const Eigen::VectorXd> joint_vals_1(x.data(), dof);
  const Eigen::Map<const Eigen::VectorXd> joint_vals_2(x.data() + dof, dof);
  CalcCollisions(joint_vals_1, joint_vals_2, dist_results);
}

void LVSContinuousCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
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
      processInterpolatedCollisionResults(contacts_vector, dist_results, 1.0 / double(subtraj.rows() - 1));
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
      const Eigen::Vector2d data = { dist, coeff };
      removeInvalidContactResults(pair.second, data);
    }
  }
}

//////////////////////////////////////////

LVSDiscreteCollisionEvaluator::LVSDiscreteCollisionEvaluator(
    tesseract_kinematics::ForwardKinematics::ConstPtr manip,
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

void LVSDiscreteCollisionEvaluator::CalcCollisions(const std::vector<double>& x,
                                                   tesseract_collision::ContactResultMap& dist_results)
{
  auto dof = static_cast<Eigen::Index>(x.size()) / 2;
  const Eigen::Map<const Eigen::VectorXd> joint_vals_1(x.data(), dof);
  const Eigen::Map<const Eigen::VectorXd> joint_vals_2(x.data() + dof, dof);
  CalcCollisions(joint_vals_1, joint_vals_2, dist_results);
}

void LVSDiscreteCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
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
    processInterpolatedCollisionResults(contacts_vector, dist_results, 1.0 / double(subtraj.rows() - 1));
}
}  // namespace trajopt
