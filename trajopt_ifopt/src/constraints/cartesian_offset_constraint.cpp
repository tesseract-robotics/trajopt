/**
 * @file cartesian_offset_constraint.cpp
 * @brief Holds a frame of one state at an offset along an axis of the pose it reaches in another state
 *
 * @author Roelof Oomen
 * @date October 13, 2026
 *
 * @copyright Copyright (c) 2026
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
#include <trajopt_ifopt/constraints/cartesian_offset_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_axis_kinematics.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_common/utils.hpp>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/common/utils.h>
#include <cmath>
#include <stdexcept>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
thread_local tesseract::common::LinkIdTransformMap CartOffsetConstraint::transforms_cache_;  // NOLINT

CartOffsetConstraint::CartOffsetConstraint(std::shared_ptr<const Var> reference_var,
                                           std::shared_ptr<const Var> follower_var,
                                           std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                                           tesseract::common::LinkId frame,
                                           const Eigen::Isometry3d& frame_offset,  // NOLINT(modernize-pass-by-value)
                                           const Eigen::Vector3d& axis,
                                           double distance,
                                           const Eigen::VectorXd& coeffs,
                                           const std::vector<Bounds>& bounds,
                                           std::string name,
                                           RangeBoundHandling range_bound_handling)
  : ConstraintSet(std::move(name), 6)
  , reference_var_(std::move(reference_var))
  , follower_var_(std::move(follower_var))
  , manip_(std::move(manip))
  , frame_(std::move(frame))
  , frame_offset_(frame_offset)
  , axis_(CartAxisKinematics::toUnitAxis(axis, "CartOffsetConstraint: offset axis"))
  , distance_(distance)
  , n_dof_(manip_->numJoints())
{
  if (reference_var_ == follower_var_)
    throw std::runtime_error("CartOffsetConstraint: the reference and the follower must be two variables.");

  if (reference_var_->size() != n_dof_ || follower_var_->size() != n_dof_)
    throw std::runtime_error("CartOffsetConstraint: each variable must hold the joint values of the group.");

  if (!manip_->hasLinkId(frame_))
    throw std::runtime_error("CartOffsetConstraint: link '" + frame_.name() + "' does not exist.");

  if (!manip_->isActiveLinkId(frame_))
    throw std::runtime_error("CartOffsetConstraint: link '" + frame_.name() + "' is a static link.");

  if (!std::isfinite(distance_))
    throw std::runtime_error("CartOffsetConstraint: the distance must be finite.");

  if (bounds.size() != 6)
    throw std::runtime_error("CartOffsetConstraint: the number of bounds should be six.");

  if (coeffs.rows() != 6)
    throw std::runtime_error("CartOffsetConstraint: the number of coeffs should be six.");

  if (!coeffs.allFinite() || (coeffs.array() < 0).any())
    throw std::runtime_error("CartOffsetConstraint: the coeffs must be finite and non-negative.");

  RowSelection selection = selectRows(coeffs, bounds, range_bound_handling);
  indices_ = std::move(selection.indices);
  coeffs_ = std::move(selection.coeffs);
  bounds_ = std::move(selection.bounds);
  rotation_rows_ = (indices_.array() >= 3).any();
  rows_ = static_cast<int>(indices_.rows());
  non_zeros_ = 2 * n_dof_ * indices_.rows();

  reference_offset_ = frame_offset_ * Eigen::Translation3d(distance_ * axis_);
}

CartOffsetConstraint::CartOffsetConstraint(std::shared_ptr<const Var> reference_var,
                                           std::shared_ptr<const Var> follower_var,
                                           std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                                           tesseract::common::LinkId frame,
                                           const Eigen::Isometry3d& frame_offset,
                                           const Eigen::Vector3d& axis,
                                           double distance,
                                           std::string name,
                                           RangeBoundHandling range_bound_handling)
  : CartOffsetConstraint(std::move(reference_var),
                         std::move(follower_var),
                         std::move(manip),
                         std::move(frame),
                         frame_offset,
                         axis,
                         distance,
                         Eigen::VectorXd::Ones(6),
                         std::vector<Bounds>(6, BoundZero),
                         std::move(name),
                         range_bound_handling)
{
}

void CartOffsetConstraint::calcLinkPoses(Eigen::Isometry3d& reference_link_tf,
                                         Eigen::Isometry3d& follower_link_tf,
                                         const Eigen::Ref<const Eigen::VectorXd>& reference_vals,
                                         const Eigen::Ref<const Eigen::VectorXd>& follower_vals) const
{
  // One clear bounds the map to the links of this group. The second pass writes the same keys, so it needs none.
  transforms_cache_.clear();
  manip_->calcFwdKin(transforms_cache_, reference_vals);
  reference_link_tf = transforms_cache_.at(frame_);

  manip_->calcFwdKin(transforms_cache_, follower_vals);
  follower_link_tf = transforms_cache_.at(frame_);
}

Eigen::VectorXd CartOffsetConstraint::calcValues(const Eigen::Ref<const Eigen::VectorXd>& reference_vals,
                                                 const Eigen::Ref<const Eigen::VectorXd>& follower_vals) const
{
  Eigen::Isometry3d reference_link_tf;
  Eigen::Isometry3d follower_link_tf;
  calcLinkPoses(reference_link_tf, follower_link_tf, reference_vals, follower_vals);

  const Eigen::Isometry3d err_tf =
      (reference_link_tf * reference_offset_).inverse() * (follower_link_tf * frame_offset_);
  Eigen::Matrix<double, 6, 1> err = Eigen::Matrix<double, 6, 1>::Zero();
  err.head<3>() = err_tf.translation();
  if (rotation_rows_)
    err.tail<3>() = tesseract::common::calcRotationalError(err_tf.linear());

  Eigen::VectorXd values(indices_.size());
  for (Eigen::Index i = 0; i < indices_.size(); ++i)
    values[i] = err[indices_[i]];

  return values;
}

Eigen::VectorXd CartOffsetConstraint::getValues() const
{
  return calcValues(reference_var_->value(), follower_var_->value());
}

Eigen::VectorXd CartOffsetConstraint::getCoefficients() const { return coeffs_; }

std::vector<Bounds> CartOffsetConstraint::getBounds() const { return bounds_; }

Jacobian CartOffsetConstraint::getJacobian() const
{
  const Eigen::VectorXd& reference_vals = reference_var_->value();
  const Eigen::VectorXd& follower_vals = follower_var_->value();

  Eigen::Isometry3d reference_link_tf;
  Eigen::Isometry3d follower_link_tf;
  calcLinkPoses(reference_link_tf, follower_link_tf, reference_vals, follower_vals);
  const Eigen::Isometry3d reference_tf = reference_link_tf * reference_offset_;
  const Eigen::Isometry3d follower_tf = follower_link_tf * frame_offset_;

  // The kinematics give the twist of the origin of the link. For the follower it is moved to the follower frame. For
  // the reference it is moved to the same point: the values change with the twist of the follower frame relative to
  // the point of the reference link that coincides with it.
  Eigen::MatrixXd follower_rates = manip_->calcJacobian(follower_vals, frame_);
  tesseract::common::jacobianChangeRefPoint(follower_rates, follower_tf.translation() - follower_link_tf.translation());
  Eigen::MatrixXd reference_rates = manip_->calcJacobian(reference_vals, frame_);
  tesseract::common::jacobianChangeRefPoint(reference_rates,
                                            follower_tf.translation() - reference_link_tf.translation());
  reference_rates = -reference_rates;

  // In the coordinates of the reference frame the translation rows are the rate of the translation values
  const Eigen::Isometry3d to_reference = reference_tf.inverse();
  tesseract::common::jacobianChangeBase(follower_rates, to_reference);
  tesseract::common::jacobianChangeBase(reference_rates, to_reference);

  // The rotation rows are angular velocities, which are mapped to the rate of the rotation vector
  if (rotation_rows_)
  {
    const Eigen::Matrix3d rate_map = trajopt_common::calcAngleAxisRateMap(
        tesseract::common::calcRotationalError(reference_tf.linear().transpose() * follower_tf.linear()));
    for (Eigen::Index j = 0; j < n_dof_; ++j)
    {
      follower_rates.col(j).tail<3>() = rate_map * follower_rates.col(j).tail<3>();
      reference_rates.col(j).tail<3>() = rate_map * reference_rates.col(j).tail<3>();
    }
  }

  // The entries of a row are written in ascending column order, whichever state comes first in the variable set
  const bool reference_first = reference_var_->getIndex() < follower_var_->getIndex();
  const Eigen::MatrixXd& first_rates = reference_first ? reference_rates : follower_rates;
  const Eigen::MatrixXd& second_rates = reference_first ? follower_rates : reference_rates;
  const Eigen::Index first_index = reference_first ? reference_var_->getIndex() : follower_var_->getIndex();
  const Eigen::Index second_index = reference_first ? follower_var_->getIndex() : reference_var_->getIndex();

  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);
  for (int i = 0; i < indices_.size(); ++i)
  {
    jac.startVec(i);
    for (Eigen::Index j = 0; j < n_dof_; ++j)
      jac.insertBack(i, first_index + j) = first_rates(indices_[i], j);
    for (Eigen::Index j = 0; j < n_dof_; ++j)
      jac.insertBack(i, second_index + j) = second_rates(indices_[i], j);
  }
  jac.finalize();  // NOLINT
  return jac;
}

}  // namespace trajopt_ifopt
