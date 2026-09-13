/**
 * @file collision_utils.cpp
 * @brief Contains utility functions used by collision constraints
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
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

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <cmath>
#include <console_bridge/console.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/utils.h>
#include <utility>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_utils.h>
#include <trajopt_common/collision_types.h>

namespace trajopt_common
{
std::size_t getHash(const void* parent, const Eigen::Ref<const Eigen::VectorXd>& dof_vals)
{
  assert(parent != nullptr);
  std::size_t seed = 0;
  boost::hash_combine(seed, parent);
  for (Eigen::Index i = 0; i < dof_vals.rows(); ++i)
    boost::hash_combine(seed, dof_vals[i]);

  return seed;
}

std::size_t getHash(const void* parent,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1)
{
  assert(parent != nullptr);
  std::size_t seed = 0;
  boost::hash_combine(seed, parent);
  for (Eigen::Index i = 0; i < dof_vals0.rows(); ++i)
  {
    boost::hash_combine(seed, dof_vals0[i]);
    boost::hash_combine(seed, dof_vals1[i]);
  }

  return seed;
}

std::size_t cantorHash(int shape_id, int subshape_id)
{
  assert(shape_id >= 0);
  if (subshape_id < 0)
    return static_cast<std::size_t>(shape_id);

  return static_cast<std::size_t>(1 / 2.0 * (shape_id + subshape_id) * (shape_id + subshape_id + 1) + subshape_id);
}

void removeInvalidContactResults(tesseract::collision::ContactResultVector& contact_results,
                                 double margin,
                                 double margin_buffer,
                                 bool var0_fixed,
                                 bool var1_fixed)
{
  auto end =
      std::remove_if(contact_results.begin(), contact_results.end(), [=](const tesseract::collision::ContactResult& r) {
        /** @todo Is this correct? (Levi)*/
        if ((r.distance > (margin + margin_buffer)))
          return true;

        if (!var0_fixed && !var1_fixed)
          return false;

        if (var0_fixed)
        {
          if (r.cc_type[0] != tesseract::collision::ContinuousCollisionType::CCType_None &&
              r.cc_type[0] != tesseract::collision::ContinuousCollisionType::CCType_Time0)
            return false;

          if (r.cc_type[1] != tesseract::collision::ContinuousCollisionType::CCType_None &&
              r.cc_type[1] != tesseract::collision::ContinuousCollisionType::CCType_Time0)
            return false;
        }

        if (var1_fixed)
        {
          if (r.cc_type[0] != tesseract::collision::ContinuousCollisionType::CCType_None &&
              r.cc_type[0] != tesseract::collision::ContinuousCollisionType::CCType_Time1)
            return false;

          if (r.cc_type[1] != tesseract::collision::ContinuousCollisionType::CCType_None &&
              r.cc_type[1] != tesseract::collision::ContinuousCollisionType::CCType_Time1)
            return false;
        }

        return true;
      });

  contact_results.erase(end, contact_results.end());
}

long castCount(const tesseract::collision::CollisionCheckConfig& config, double segment_length)
{
  // Negated rather than written as <=, so a length that is not a number is cast once instead of reaching the
  // float-to-integer conversion below
  if (config.type != tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS ||
      !(config.longest_valid_segment_length > 0.0) || !(segment_length > config.longest_valid_segment_length))
    return 1;

  return static_cast<long>(std::ceil(segment_length / config.longest_valid_segment_length));
}

ContactInterval contactInterval(double cc_time, long cast_count)
{
  if (cast_count <= 0)
    return { cc_time, cc_time };

  const auto count = static_cast<double>(cast_count);
  const long index = std::clamp(static_cast<long>(std::floor(cc_time * count)), 0L, cast_count - 1);
  return { static_cast<double>(index) / count, static_cast<double>(index + 1) / count };
}

IntervalWeights intervalWeights(double cc_time, const ContactInterval& interval)
{
  const double width = interval.end - interval.start;
  // A fraction of the interval rather than of the segment, so the comparison holds at any cast count. A zero width is
  // a point in time, which sits at its start.
  const double raw = (width > 0.0) ? (cc_time - interval.start) / width : 0.0;
  const double tau = std::clamp(raw, 0.0, 1.0);

  // Judged on the unclamped position: a time the caller placed outside the interval is at neither end, however near
  // the clamp puts it.
  constexpr double at_end_tau = 1e-9;
  return { (1.0 - tau) * (1.0 - interval.start), tau * (1.0 - interval.end), (1.0 - tau) * interval.start,
           tau * interval.end,  std::abs(raw) <= at_end_tau,                 std::abs(raw - 1.0) <= at_end_tau };
}

Eigen::VectorXd intervalState(const tesseract::collision::ContactResult& contact_result,
                              std::size_t i,
                              const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              double s)
{
  if (contact_result.cc_type[i] == tesseract::collision::ContinuousCollisionType::CCType_Time0 || s <= 0.0)
    return dofvals0;

  if (contact_result.cc_type[i] == tesseract::collision::ContinuousCollisionType::CCType_Time1 || s >= 1.0)
    return dofvals1;

  return dofvals0 + (dofvals1 - dofvals0) * s;
}

namespace
{
/**
 * @brief The translational jacobian of a contact's reference point on one of its links
 * @param link_transform The pose of @p contact_result.link_ids[i] at @p dofvals. The reference point is rotated by it,
 * so it must belong to the configuration the jacobian is evaluated at; a pose the contact happens to carry for some
 * other configuration rotates the offset into the wrong frame.
 * @param on_link Whether the contact's witness point lies on the link at @p dofvals, which holds at the contact's own
 * time and nowhere else. The offset then follows from the witness the contact reports in world coordinates, the only
 * reading that is exact there: the stored local point is the mean of the two support points, and for a contact pinned
 * to an interval end it is expressed in the frame of the pose the contact carries rather than in this one.
 */
Eigen::MatrixXd contactJacobian(const tesseract::kinematics::JointGroup& manip,
                                const Eigen::VectorXd& dofvals,
                                const Eigen::Isometry3d& link_transform,
                                const tesseract::collision::ContactResult& contact_result,
                                std::size_t i,
                                bool on_link)
{
  /** @todo update calcJacobian to have out param overload */
  Eigen::MatrixXd jac = manip.calcJacobian(dofvals, contact_result.link_ids[i]);

  // Need to change the base and ref point of the jacobian.
  // When changing ref point you must provide a vector from the current ref
  // point to the new ref point. Since the link transform is known then do not call calcJacobian with link point.
  Eigen::Vector3d offset;
  if (on_link)
    offset = contact_result.nearest_points[i] - link_transform.translation();
  else
    offset = link_transform.linear() * contact_result.nearest_points_local[i];

  tesseract::common::jacobianChangeRefPoint(jac, offset);
  return jac.topRows(3);
}

/**
 * @brief Fill one timestep's gradient of one of a contact's links from its contact jacobian
 * @param time_weight The timestep's weight, applied only to a typed contact
 */
void setLinkGradient(LinkGradientResults& link_gradient,
                     Eigen::MatrixXd jacobian,
                     double time_weight,
                     const tesseract::collision::ContactResult& contact_result,
                     std::size_t i)
{
  link_gradient.has_gradient = true;
  link_gradient.scale = 1;
  // Gated on cc_type rather than cc_time, unlike trajopt's two-state GetGradient, which gates on a
  // non-negative cc_time instead. The two agree on every contact the checks produce, because the
  // backends' cast results and addInterpolatedCollisionResults both type every active link.
  if (contact_result.cc_type[i] != tesseract::collision::ContinuousCollisionType::CCType_None)
  {
    assert(contact_result.cc_time[i] > 0.0 ||
           tesseract::common::almostEqualRelativeAndAbs(contact_result.cc_time[i], 0.0));
    assert(contact_result.cc_time[i] < 1.0 ||
           tesseract::common::almostEqualRelativeAndAbs(contact_result.cc_time[i], 1.0));
    link_gradient.scale = time_weight;
    link_gradient.cc_type = contact_result.cc_type[i];
  }

  link_gradient.translation_vector = contact_result.normal;
  if (i == 0)
    link_gradient.translation_vector *= -1.0;

  link_gradient.jacobian = std::move(jacobian);
  link_gradient.gradient.resize(link_gradient.jacobian.cols());
  link_gradient.gradient.noalias() = link_gradient.jacobian.transpose() * link_gradient.translation_vector;
}
}  // namespace

/**
 * @brief Compute one link's gradient contribution for a contact
 * @param link_transform The pose of @p contact_result.link_ids[i] at @p dofvals. The reference point
 * is rotated by it, so it must belong to the configuration the jacobian is evaluated at; a pose the
 * contact happens to carry for some other configuration rotates the offset into the wrong frame.
 */
void calcGradient(GradientResults& results,
                  std::size_t i,
                  const Eigen::VectorXd& dofvals,
                  const Eigen::Isometry3d& link_transform,
                  const tesseract::collision::ContactResult& contact_result,
                  const tesseract::kinematics::JointGroup& manip)
{
  setLinkGradient(results.gradients[i],
                  contactJacobian(manip, dofvals, link_transform, contact_result, i, false),
                  1 - contact_result.cc_time[i],
                  contact_result,
                  i);

  // #ifndef NDEBUG // This is good for checking discrete evaluators
  //   Eigen::Isometry3d test_link_transform = manip->calcFwdKin(dofvals, it->link_name);
  //   Eigen::Isometry3d temp1 = world_to_base * test_link_transform;
  //   Eigen::Isometry3d temp2 = link_transform * it->transform.inverse();
  //   assert(temp1.isApprox(temp2, 0.0001));

  //  Eigen::MatrixXd jac_test;
  //  jac_test.resize(6, manip->numJoints());
  //  tesseract::kinematics::numericalJacobian(jac_test, world_to_base, *manip, dofvals, it->link_name,
  //  contact_result.nearest_points_local[i]); bool check = link_gradient.jacobian.isApprox(jac_test.topRows(3), 1e-3);
  //  assert(check == true);
  // #endif
}

void getGradient(GradientResults& results,
                 const Eigen::VectorXd& dofvals,
                 const tesseract::collision::ContactResult& contact_result,
                 double margin,
                 double margin_buffer,
                 const tesseract::kinematics::JointGroup& manip)
{
  results.error = (margin - contact_result.distance);
  results.error_with_buffer = (margin + margin_buffer - contact_result.distance);
  for (std::size_t i = 0; i < 2; ++i)
  {
    // A discrete contact stores the pose of the state it was found at, which is the state the
    // jacobian is evaluated at, so no forward kinematics is needed here and both readings of the
    // witness point agree.
    if (manip.isActiveLinkId(contact_result.link_ids[i]))
      calcGradient(results, i, dofvals, contact_result.transform[i], contact_result, manip);
  }
  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));
}

void getGradient(GradientResults& results,
                 const Eigen::VectorXd& dofvals0,
                 const Eigen::VectorXd& dofvals1,
                 const tesseract::collision::ContactResult& contact_result,
                 double margin,
                 double margin_buffer,
                 const tesseract::kinematics::JointGroup& manip,
                 long cast_count)
{
  results.error = (margin - contact_result.distance);
  results.error_with_buffer = (margin + margin_buffer - contact_result.distance);

  // Reused across both links and both interval ends so the map keeps its nodes; calcFwdKin overwrites every entry of
  // the group, so nothing carries over between calls.
  tesseract::common::LinkIdTransformMap link_transforms;
  /**
   * @todo Look at decoupling this from the cc_transforms so we only have one gradient for timestep0 and timestep1
   * This will simplify a lot of the data structures if you have a single gradient where you only the scale is
   * different
   */
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (!manip.isActiveLinkId(contact_result.link_ids[i]))
      continue;

    const tesseract::collision::ContinuousCollisionType cc_type = contact_result.cc_type[i];
    const bool pinned = cc_type == tesseract::collision::ContinuousCollisionType::CCType_Time0 ||
                        cc_type == tesseract::collision::ContinuousCollisionType::CCType_Time1;
    const double cc_time = contact_result.cc_time[i];
    auto jacobianAt = [&](double s, bool on_link) {
      const Eigen::VectorXd dofvals = intervalState(contact_result, i, dofvals0, dofvals1, s);
      manip.calcFwdKin(link_transforms, dofvals);
      return contactJacobian(
          manip, dofvals, link_transforms.at(contact_result.link_ids[i]), contact_result, i, on_link);
    };

    // A link the check gave no interval is linearised at the segment start for both halves rather than extrapolated
    // outside the segment: both halves come from one state, so there is no separate endpoint per timestep.
    if (!pinned && cc_time < 0.0)
    {
      Eigen::MatrixXd jacobian = jacobianAt(0.0, false);
      setLinkGradient(results.gradients[i], jacobian, 1 - cc_time, contact_result, i);
      setLinkGradient(results.cc_gradients[i], std::move(jacobian), cc_time, contact_result, i);
      continue;
    }

    // The link's contact point moves with it at both ends of the interval it was found in, so each timestep's gradient
    // blends the contact jacobians at those two states. A link pinned to a segment endpoint is a point in time there,
    // weighted by its own time.
    const ContactInterval interval =
        pinned ? ContactInterval{ cc_time, cc_time } : contactInterval(cc_time, cast_count);
    const IntervalWeights w = intervalWeights(cc_time, interval);

    // An end neither timestep weights is not evaluated, except that a timestep weighting neither end takes the
    // interval start's
    Eigen::MatrixXd at_start;
    Eigen::MatrixXd at_end;
    if (w.start_a > 0.0 || w.end_a > 0.0 || w.start_b == 0.0 || w.end_b == 0.0)
      at_start = jacobianAt(interval.start, w.start_at_contact);
    if (w.start_b > 0.0 || w.end_b > 0.0)
      at_end = jacobianAt(interval.end, w.end_at_contact);

    setLinkGradient(results.gradients[i],
                    blendIntervalEnds(at_start, w.start_a, at_end, w.start_b),
                    w.start_a + w.start_b,
                    contact_result,
                    i);
    setLinkGradient(results.cc_gradients[i],
                    blendIntervalEnds(at_start, w.end_a, at_end, w.end_b),
                    w.end_a + w.end_b,
                    contact_result,
                    i);
  }

  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));
}

void debugPrintInfo(const tesseract::collision::ContactResult& res,
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
              res.link_ids[0].name().c_str(),
              res.link_ids[1].name().c_str(),
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
}  // namespace trajopt_common
