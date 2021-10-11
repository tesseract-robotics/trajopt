/**
 * @file collision_utils.cpp
 * @brief Contains utility functions used by collision constraints
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

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
#include <console_bridge/console.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision/collision_utils.h>

namespace trajopt_ifopt
{
std::size_t getHash(const TrajOptCollisionConfig& collision_config, const Eigen::Ref<const Eigen::VectorXd>& dof_vals)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, &collision_config);
  for (Eigen::Index i = 0; i < dof_vals.rows(); ++i)
    boost::hash_combine(seed, dof_vals[i]);

  return seed;
}

std::size_t getHash(const TrajOptCollisionConfig& collision_config,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, &collision_config);
  for (Eigen::Index i = 0; i < dof_vals0.rows(); ++i)
  {
    boost::hash_combine(seed, dof_vals0[i]);
    boost::hash_combine(seed, dof_vals1[i]);
  }

  return seed;
}

void processInterpolatedCollisionResults(std::vector<tesseract_collision::ContactResultMap>& contacts_vector,
                                         tesseract_collision::ContactResultMap& contact_results,
                                         const std::vector<std::string>& active_links,
                                         const TrajOptCollisionConfig& collision_config,
                                         double dt,
                                         bool discrete_continuous)
{
  // If contact is found the actual dt between the original two state must be recalculated based on where it
  // occured in the subtrajectory. Also the cc_type must also be recalculated but does not appear to be used
  // currently by trajopt.
  //  const std::vector<std::string>& active_links = adjacency_map_->getActiveLinkNames();
  for (size_t i = 0; i < contacts_vector.size(); ++i)
  {
    for (auto& pair : contacts_vector[i])
    {
      auto p = contact_results.find(pair.first);

      // Contains the contact distance threshold and coefficient for the given link pair
      double dist = collision_config.collision_margin_data.getPairCollisionMargin(pair.first.first, pair.first.second);
      double coeff = collision_config.collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
      const Eigen::Vector3d data = { dist, collision_config.collision_margin_buffer, coeff };

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
            assert(r.cc_time[j] > 0.0 || tesseract_common::almostEqualRelativeAndAbs(r.cc_time[j], 0.0));
            assert(r.cc_time[j] < 1.0 || tesseract_common::almostEqualRelativeAndAbs(r.cc_time[j], 1.0));
            if (i == 0 && r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time0;
            else if (i == (contacts_vector.size() - 1) &&
                     r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time1;
            else
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Between;

            if (discrete_continuous)
              r.cc_transform = r.transform;
          }
        }
      }

      // Don't include contacts outside the buffer distance @todo Should this be done? Levi
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

void removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results, const Eigen::Vector3d& data)
{
  auto end = std::remove_if(
      contact_results.begin(), contact_results.end(), [=, &data](const tesseract_collision::ContactResult& r) {
        /** @todo Is this correct? (Levi)*/
        return (!((data[0] + data[1]) > r.distance));
      });

  contact_results.erase(end, contact_results.end());
}

void calcGradient(GradientResults& results,
                  std::size_t i,
                  const Eigen::VectorXd& dofvals,
                  const tesseract_collision::ContactResult& contact_result,
                  const tesseract_kinematics::JointGroup::ConstPtr& manip,
                  bool isTimestep1)
{
  LinkGradientResults& link_gradient = (isTimestep1) ? results.cc_gradients[i] : results.gradients[i];
  link_gradient.has_gradient = true;

  // Calculate Jacobian
  Eigen::MatrixXd jac = manip->calcJacobian(dofvals, contact_result.link_names[i]);

  // Need to change the base and ref point of the jacobian.
  // When changing ref point you must provide a vector from the current ref
  // point to the new ref point.
  link_gradient.scale = 1;
  Eigen::Isometry3d link_transform = contact_result.transform[i];
  if (contact_result.cc_type[i] != tesseract_collision::ContinuousCollisionType::CCType_None)
  {
    assert(contact_result.cc_time[i] > 0.0 ||
           tesseract_common::almostEqualRelativeAndAbs(contact_result.cc_time[i], 0.0));
    assert(contact_result.cc_time[i] < 1.0 ||
           tesseract_common::almostEqualRelativeAndAbs(contact_result.cc_time[i], 1.0));
    link_gradient.scale = (isTimestep1) ? contact_result.cc_time[i] : (1 - contact_result.cc_time[i]);
    link_gradient.cc_type = contact_result.cc_type[i];
    link_transform = (isTimestep1) ? contact_result.cc_transform[i] : contact_result.transform[i];
    /**
     * @todo Look at decoupling this from the cc_transforms so we only have one gradient for timestep0 and timestep1
     * This will simplify a lot of the data structures if you have a single gradient where you only the scale is
     * different
     */
  }
  // Since the link transform is known then do not call calcJacobian with link point
  tesseract_common::jacobianChangeRefPoint(jac, link_transform.linear() * contact_result.nearest_points_local[i]);

  link_gradient.translation_vector = ((i == 0) ? -1.0 : 1.0) * contact_result.normal;
  link_gradient.jacobian = jac.topRows(3);
  link_gradient.gradient = link_gradient.translation_vector.transpose() * link_gradient.jacobian;

  //#ifndef NDEBUG // This is good for checking discrete evaluators
  //  Eigen::Isometry3d test_link_transform = manip->calcFwdKin(dofvals, it->link_name);
  //  Eigen::Isometry3d temp1 = world_to_base * test_link_transform;
  //  Eigen::Isometry3d temp2 = link_transform * it->transform.inverse();
  //  assert(temp1.isApprox(temp2, 0.0001));

  //  Eigen::MatrixXd jac_test;
  //  jac_test.resize(6, manip->numJoints());
  //  tesseract_kinematics::numericalJacobian(jac_test, world_to_base, *manip, dofvals, it->link_name,
  //  contact_result.nearest_points_local[i]); bool check = link_gradient.jacobian.isApprox(jac_test.topRows(3), 1e-3);
  //  assert(check == true);
  //#endif
}

GradientResults getGradient(const Eigen::VectorXd& dofvals,
                            const tesseract_collision::ContactResult& contact_result,
                            double margin,
                            double margin_buffer,
                            const tesseract_kinematics::JointGroup::ConstPtr& manip)
{
  GradientResults results;
  results.error = (margin - contact_result.distance);
  results.error_with_buffer = (margin + margin_buffer - contact_result.distance);
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (manip->isActiveLinkName(contact_result.link_names[i]))
      calcGradient(results, i, dofvals, contact_result, manip, false);
  }
  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));

  return results;
}

GradientResults getGradient(const Eigen::VectorXd& dofvals0,
                            const Eigen::VectorXd& dofvals1,
                            const tesseract_collision::ContactResult& contact_result,
                            double margin,
                            double margin_buffer,
                            const tesseract_kinematics::JointGroup::ConstPtr& manip)
{
  GradientResults results;
  results.error = (margin - contact_result.distance);
  results.error_with_buffer = (margin + margin_buffer - contact_result.distance);

  Eigen::VectorXd dofvalst = Eigen::VectorXd::Zero(dofvals0.size());
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (manip->isActiveLinkName(contact_result.link_names[i]))
    {
      if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
        dofvalst = dofvals0;
      else if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
        dofvalst = dofvals1;
      else
        dofvalst = dofvals0 + (dofvals1 - dofvals0) * contact_result.cc_time[i];

      calcGradient(results, i, dofvalst, contact_result, manip, false);
      calcGradient(results, i, dofvalst, contact_result, manip, true);
    }
  }

  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));

  return results;
}

void debugPrintInfo(const tesseract_collision::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals,
                    bool header)
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
}  // namespace trajopt_ifopt
