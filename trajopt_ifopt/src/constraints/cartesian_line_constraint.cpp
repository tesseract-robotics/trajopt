/**
 * @file cartesian_position_constraint.h
 * @brief The cartesian position constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @author Colin Lewis
 * @date December 27, 2020
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

#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt_common/utils.hpp>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/common/utils.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
namespace
{
/**
 * @brief Calculate the fraction of a line at its point nearest a position
 * @param position The position
 * @param start The start of the line
 * @param line The vector from the start of the line to its end
 * @return The fraction, not kept between the ends of the line; zero for a line of zero length
 */
double lineFraction(const Eigen::Vector3d& position, const Eigen::Vector3d& start, const Eigen::Vector3d& line)
{
  const double length_squared = line.squaredNorm();
  return (length_squared > 0.0) ? ((position - start).dot(line) / length_squared) : 0.0;
}

/**
 * @brief Get the orientations of the ends of a line as the quaternions the line is interpolated between
 * @details Of the two quaternions of the orientation of the end, the one nearest that of the start is returned, so
 * the line turns the shorter way. This also settles which way a line turns whose ends are half a turn apart.
 * @param start The pose of the start of the line
 * @param end The pose of the end of the line
 * @return The quaternion of the start and that of the end
 */
std::pair<Eigen::Quaterniond, Eigen::Quaterniond> lineQuaternions(const Eigen::Isometry3d& start,
                                                                  const Eigen::Isometry3d& end)
{
  const Eigen::Quaterniond quat_a(start.rotation());
  Eigen::Quaterniond quat_b(end.rotation());
  if (quat_a.dot(quat_b) < 0.0)
    quat_b.coeffs() = -quat_b.coeffs();

  return { quat_a, quat_b };
}

/**
 * @brief Calculate the turn of a line from its start to its end
 * @param start The pose of the start of the line
 * @param end The pose of the end of the line
 * @return The turn as an angle axis vector, which is the same in the frame of every point of the line
 */
Eigen::Vector3d lineTurn(const Eigen::Isometry3d& start, const Eigen::Isometry3d& end)
{
  const auto [quat_a, quat_b] = lineQuaternions(start, end);
  const Eigen::Quaterniond turn = quat_a.conjugate() * quat_b;

  // The angle is not brought back to the shorter way here: lineQuaternions settled the way the line turns
  const double sine = turn.vec().norm();
  if (sine > 0.0)
    return turn.vec() * (2.0 * std::atan2(sine, turn.w()) / sine);

  return Eigen::Vector3d::Zero();
}

/**
 * @brief Get the nearest point on the line to the source, found in the frame of the link of the line
 * @details The ends of the line are the same in that frame at every joint position, so a line whose ends are half a
 * turn apart turns the same way at each of them.
 * @param info The line
 * @param target_link_tf The pose of the link of the line
 * @param source_tf The pose of the source
 * @return The pose of the nearest point, in the frame the two poses are given in
 */
Eigen::Isometry3d nearestLinePoint(const CartLineInfo& info,
                                   const Eigen::Isometry3d& target_link_tf,
                                   const Eigen::Isometry3d& source_tf)
{
  return target_link_tf * CartLineConstraint::getLinePoint(target_link_tf.inverse() * source_tf,
                                                           info.target_frame_offset1,
                                                           info.target_frame_offset2);
}
}  // namespace

CartLineInfo::CartLineInfo(std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                           tesseract::common::LinkId source_frame,
                           tesseract::common::LinkId target_frame,
                           const Eigen::Isometry3d& target_frame_offset1,  // NOLINT(modernize-pass-by-value)
                           const Eigen::Isometry3d& target_frame_offset2,  // NOLINT(modernize-pass-by-value)
                           const Eigen::Isometry3d& source_frame_offset,   // NOLINT(modernize-pass-by-value)
                           const Eigen::VectorXi& indices)                 // NOLINT(modernize-pass-by-value)
  : manip(std::move(manip))
  , source_frame(std::move(source_frame))
  , target_frame(std::move(target_frame))
  , source_frame_offset(source_frame_offset)
  , target_frame_offset1(target_frame_offset1)
  , target_frame_offset2(target_frame_offset2)
  , indices(indices)
{
  if (!this->manip->hasLinkId(this->source_frame))
    throw std::runtime_error("CartLineInfo: Source Link name '" + this->source_frame.name() +
                             "' provided does not exist.");

  if (!this->manip->hasLinkId(this->target_frame))
    throw std::runtime_error("CartLineInfo: Target Link name '" + this->target_frame.name() +
                             "' provided does not exist.");

  if (this->target_frame_offset1.isApprox(target_frame_offset2))
    throw std::runtime_error("CartLineInfo: The start and end point are the same!");

  if (this->indices.size() > 6)
    throw std::runtime_error("CartLineInfo: The indices list length cannot be larger than six.");

  if (this->indices.size() == 0)
    throw std::runtime_error("CartLineInfo: The indices list length is zero.");
}

thread_local tesseract::common::LinkIdTransformMap CartLineConstraint::transforms_cache_;  // NOLINT

CartLineConstraint::CartLineConstraint(CartLineInfo info,
                                       std::shared_ptr<const Var> position_var,
                                       const Eigen::VectorXd& coeffs,  // NOLINT
                                       std::string name)
  : ConstraintSet(std::move(name), static_cast<int>(info.indices.rows()))
  , coeffs_(coeffs)
  , position_var_(std::move(position_var))
  , info_(std::move(info))
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = info_.manip->numJoints();
  assert(n_dof_ > 0);

  non_zeros_ = n_dof_ * info_.indices.rows();
  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(info_.indices.rows()), BoundZero);

  if (coeffs_.rows() != info_.indices.rows())
    throw std::runtime_error("The number of coeffs does not match the number of constraints.");

  if (!coeffs_.allFinite() || (coeffs_.array() < 0).any())
    throw std::runtime_error("The coeffs must be finite and non-negative.");

  source_active_ = info_.manip->isActiveLinkId(info_.source_frame);
  target_active_ = info_.manip->isActiveLinkId(info_.target_frame);

  link_line_ = info_.target_frame_offset2.translation() - info_.target_frame_offset1.translation();
  line_turn_ = lineTurn(info_.target_frame_offset1, info_.target_frame_offset2);
  rotation_rows_ = (info_.indices.array() >= 3).any();

  error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                const Eigen::Isometry3d& target_tf,
                                const Eigen::Isometry3d& source_tf,
                                tesseract::common::LinkIdTransformMap& transforms_cache) -> Eigen::VectorXd {
    info_.manip->calcFwdKin(transforms_cache, vals);
    const Eigen::Isometry3d perturbed_source_tf = transforms_cache.at(info_.source_frame) * info_.source_frame_offset;

    // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
    const Eigen::Isometry3d perturbed_target_tf =
        nearestLinePoint(info_, transforms_cache.at(info_.target_frame), perturbed_source_tf);

    Eigen::VectorXd error_diff = tesseract::common::calcJacobianTransformErrorDiff(
        target_tf, perturbed_target_tf, source_tf, perturbed_source_tf);

    Eigen::VectorXd reduced_error_diff(info_.indices.size());
    for (int i = 0; i < info_.indices.size(); ++i)
      reduced_error_diff[i] = error_diff[info_.indices[i]];

    return reduced_error_diff;
  };
}

Eigen::VectorXd CartLineConstraint::calcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  transforms_cache_.clear();
  info_.manip->calcFwdKin(transforms_cache_, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache_.at(info_.source_frame) * info_.source_frame_offset;

  // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
  const Eigen::Isometry3d target_tf = nearestLinePoint(info_, transforms_cache_.at(info_.target_frame), source_tf);

  // pose error is the vector from the new_pose to nearest point on line AB, line_point
  // the below method is equivalent to the position constraint; using the line point as the target point
  Eigen::VectorXd err = tesseract::common::calcTransformError(target_tf, source_tf);

  Eigen::VectorXd reduced_err(info_.indices.size());
  for (int i = 0; i < info_.indices.size(); ++i)
    reduced_err[i] = err[info_.indices[i]];

  return reduced_err;  // This is available in 3.4 err(indices_, Eigen::all);
}

Eigen::VectorXd CartLineConstraint::getValues() const { return calcValues(position_var_->value()); }

Eigen::VectorXd CartLineConstraint::getCoefficients() const { return coeffs_; }

// Set the limits on the constraint values
std::vector<Bounds> CartLineConstraint::getBounds() const { return bounds_; }

void CartLineConstraint::setBounds(const std::vector<Bounds>& bounds)
{
  if (bounds.size() != static_cast<std::size_t>(rows_))
    throw std::runtime_error("CartLineConstraint: The number of bounds does not match the number of constraints.");

  bounds_ = bounds;
}

void CartLineConstraint::calcJacobianBlock(Jacobian& jac_block,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  transforms_cache_.clear();
  info_.manip->calcFwdKin(transforms_cache_, joint_vals);
  const Eigen::Isometry3d source_link_tf = transforms_cache_.at(info_.source_frame);
  const Eigen::Isometry3d source_tf = source_link_tf * info_.source_frame_offset;
  const Eigen::Isometry3d target_link_tf = transforms_cache_.at(info_.target_frame);

  // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
  const Eigen::Isometry3d target_tf = nearestLinePoint(info_, target_link_tf, source_tf);

  constexpr double eps{ 1e-5 };
  if (use_numeric_differentiation)
  {
    Eigen::MatrixXd jac0(info_.indices.size(), joint_vals.size());
    Eigen::VectorXd dof_vals_pert = joint_vals;
    for (int i = 0; i < joint_vals.size(); ++i)
    {
      dof_vals_pert(i) = joint_vals(i) + eps;
      const Eigen::VectorXd error_diff = error_diff_function_(dof_vals_pert, target_tf, source_tf, transforms_cache_);
      jac0.col(i) = error_diff / eps;
      dof_vals_pert(i) = joint_vals(i);
    }

    // The rows of jac0 already follow the indices
    for (int i = 0; i < info_.indices.size(); i++)
    {
      jac_block.startVec(i);
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.insertBack(i, position_var_->getIndex() + j) = jac0(i, j);
      }
    }
  }
  else
  {
    // The geometric jacobian of the kinematics maps joint velocities to the twist of a point fixed to a link. The
    // error follows the twist of the source relative to the link of the line: that of the source minus that of the
    // point of the link of the line that coincides with the source. A link the joints do not move contributes none.
    // The kinematics give the twist of the origin of a link, which is moved to the source. The result is rotated into
    // the frame of the nearest point on the line.
    const Eigen::Isometry3d target_tf_inv = target_tf.inverse();
    Eigen::MatrixXd twists;
    if (source_active_)
    {
      twists = info_.manip->calcJacobian(joint_vals, info_.source_frame);
      tesseract::common::jacobianChangeRefPoint(twists, source_tf.translation() - source_link_tf.translation());
    }
    else
    {
      twists.setZero(6, n_dof_);
    }
    if (target_active_)
    {
      Eigen::MatrixXd target_twists = info_.manip->calcJacobian(joint_vals, info_.target_frame);
      tesseract::common::jacobianChangeRefPoint(target_twists, source_tf.translation() - target_link_tf.translation());
      twists -= target_twists;
    }
    tesseract::common::jacobianChangeBase(twists, target_tf_inv);

    // Beside the line the nearest point follows the source along the line and turns as the line does from its start
    // to its end. Beyond an end it stays at that end. A source within this fraction of the length of the line of an
    // end is beside the line.
    constexpr double end_tolerance{ 1e-9 };
    const double length_squared = link_line_.squaredNorm();
    const double fraction = lineFraction(
        target_link_tf.inverse() * source_tf.translation(), info_.target_frame_offset1.translation(), link_line_);
    const bool slides = length_squared > 0.0 && fraction >= -end_tolerance && fraction <= 1.0 + end_tolerance;
    const Eigen::Vector3d line = target_tf.linear().transpose() * (target_link_tf.linear() * link_line_);

    // The error is the pose of the source in the frame of the nearest point. With v and w the linear and the angular
    // rows of the twists, d the position error, l the line, t its turn and s the slide, its rate is
    //   v + (d x t - l) s   in translation
    //   A (w - t s)         in rotation
    // where A maps an angular velocity to the rate of the angle axis vector of the rotation error.
    const Eigen::Isometry3d error_tf = target_tf_inv * source_tf;
    const Eigen::Vector3d translation_per_slide = error_tf.translation().cross(line_turn_) - line;

    // Without a rotation row among those the indices name, the rotation error is not needed
    Eigen::Matrix3d rate_map = Eigen::Matrix3d::Identity();
    if (rotation_rows_)
      rate_map = trajopt_common::calcAngleAxisRateMap(tesseract::common::calcRotationalError(error_tf.linear()));

    // The rows of the jacobian the indices name
    Eigen::MatrixXd jac0(info_.indices.size(), n_dof_);
    for (Eigen::Index j = 0; j < n_dof_; ++j)
    {
      const Eigen::Vector3d linear = twists.col(j).head<3>();
      const Eigen::Vector3d angular = twists.col(j).tail<3>();

      // The fraction of the line the nearest point moves per unit velocity of the joint
      const double slide = slides ? (line.dot(linear) / length_squared) : 0.0;

      Eigen::Matrix<double, 6, 1> rate;
      rate.head<3>() = linear + translation_per_slide * slide;
      rate.tail<3>() = rate_map * (angular - line_turn_ * slide);
      for (int i = 0; i < info_.indices.size(); ++i)
        jac0(i, j) = rate[info_.indices[i]];
    }

    // Convert to a sparse matrix and set the jacobian
    // TODO: Make this more efficient. This does not work.
    //    Jacobian jac_block = jac0.sparseView();

    for (int i = 0; i < info_.indices.size(); i++)
    {
      jac_block.startVec(i);
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.insertBack(i, position_var_->getIndex() + j) = jac0(i, j);
      }
    }
  }
  jac_block.finalize();  // NOLINT
}

Jacobian CartLineConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  // Get current joint values and calculate jacobian
  calcJacobianBlock(jac, position_var_->value());  // NOLINT

  return jac;
}

std::pair<Eigen::Isometry3d, Eigen::Isometry3d> CartLineConstraint::getLine() const
{
  return std::make_pair(info_.target_frame_offset1, info_.target_frame_offset2);
}

const CartLineInfo& CartLineConstraint::getInfo() const { return info_; }

Eigen::Isometry3d CartLineConstraint::getLinePoint(const Eigen::Isometry3d& source_tf,
                                                   const Eigen::Isometry3d& target_tf1,
                                                   const Eigen::Isometry3d& target_tf2)
{
  const Eigen::Vector3d line = target_tf2.translation() - target_tf1.translation();

  // The fraction of the line at the point nearest the source; a line of zero length has only its start
  const double fraction = std::clamp(lineFraction(source_tf.translation(), target_tf1.translation(), line), 0.0, 1.0);

  Eigen::Isometry3d line_point = Eigen::Isometry3d::Identity();
  line_point.translation() = target_tf1.translation() + fraction * line;

  // The orientation of the line_point is found using quaternion SLERP
  const auto [quat_a, quat_b] = lineQuaternions(target_tf1, target_tf2);
  line_point.linear() = quat_a.slerp(fraction, quat_b).toRotationMatrix();

  return line_point;
}

}  // namespace trajopt_ifopt
