/**
 * @file cartesian_axis_kinematics.cpp
 * @brief The relative axis direction shared by the cartesian axis constraints
 *
 * @author Jelle Feringa
 * @date October 1, 2026
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
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <sstream>
#include <stdexcept>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_axis_kinematics.h>
#include <tesseract/kinematics/joint_group.h>

namespace trajopt_ifopt
{
thread_local tesseract::common::LinkIdTransformMap CartAxisKinematics::transforms_cache_;  // NOLINT

namespace
{
/** @brief Cross-product matrix: skew(w) * x = w x x */
Eigen::Matrix3d skew(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d s;
  s << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
  return s;
}
}  // namespace

CartAxisKinematics::CartAxisKinematics(std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                                       tesseract::common::LinkId source_frame,
                                       const Eigen::Vector3d& source_axis,
                                       tesseract::common::LinkId target_frame)
  : manip_(std::move(manip))
  , source_frame_(std::move(source_frame))
  , target_frame_(std::move(target_frame))
  , source_axis_(toUnitAxis(source_axis, "source axis"))
{
  if (!manip_->hasLinkId(source_frame_))
    throw std::runtime_error("CartAxis: source link '" + source_frame_.name() + "' does not exist.");

  if (!manip_->hasLinkId(target_frame_))
    throw std::runtime_error("CartAxis: target link '" + target_frame_.name() + "' does not exist.");

  source_active_ = manip_->isActiveLinkId(source_frame_);
  target_active_ = manip_->isActiveLinkId(target_frame_);
  if (!source_active_ && !target_active_)
    throw std::runtime_error("CartAxis: source link '" + source_frame_.name() + "' and target link '" +
                             target_frame_.name() + "' are both static.");

  n_dof_ = manip_->numJoints();
}

Eigen::Vector3d CartAxisKinematics::toUnitAxis(const Eigen::Vector3d& axis, const std::string& what)
{
  const double norm = axis.norm();
  if (!axis.allFinite() || !(norm > 0))
  {
    std::stringstream msg;
    msg << "CartAxis: " << what << " must be finite and non-zero, got [" << axis.transpose() << "].";
    throw std::runtime_error(msg.str());
  }
  return axis / norm;
}

Eigen::Vector2d CartAxisKinematics::mostReachableDirection(const Eigen::Matrix2Xd& tangent_jacobian)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(tangent_jacobian * tangent_jacobian.transpose());
  return eig.eigenvectors().col(1);  // eigenvalues ascend; column 1 is the largest
}

void CartAxisKinematics::calcRotations(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                       Eigen::Matrix3d& source_rotation,
                                       Eigen::Matrix3d& target_rotation) const
{
  transforms_cache_.clear();
  manip_->calcFwdKin(transforms_cache_, joint_vals);
  source_rotation = transforms_cache_.at(source_frame_).linear();
  target_rotation = transforms_cache_.at(target_frame_).linear();
}

Eigen::Vector3d CartAxisKinematics::calcAxis(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  Eigen::Matrix3d source_rotation;
  Eigen::Matrix3d target_rotation;
  calcRotations(joint_vals, source_rotation, target_rotation);
  return target_rotation.transpose() * (source_rotation * source_axis_);
}

void CartAxisKinematics::calcAxisAndJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                             Eigen::Vector3d& axis,
                                             Eigen::Matrix3Xd& axis_jacobian) const
{
  Eigen::Matrix3d source_rotation;
  Eigen::Matrix3d target_rotation;
  calcRotations(joint_vals, source_rotation, target_rotation);

  const Eigen::Vector3d world_axis = source_rotation * source_axis_;
  axis = target_rotation.transpose() * world_axis;

  // Relative angular velocity of the source with respect to the target, world frame: J_w,src - J_w,tgt
  Eigen::Matrix3Xd relative_angular = Eigen::Matrix3Xd::Zero(3, n_dof_);
  if (source_active_)
    relative_angular += manip_->calcJacobian(joint_vals, source_frame_).bottomRows<3>();
  if (target_active_)
    relative_angular -= manip_->calcJacobian(joint_vals, target_frame_).bottomRows<3>();

  // d(R_tgt^T w)/dt = R_tgt^T ((w_src - w_tgt) x w) = -R_tgt^T [w]x (w_src - w_tgt)
  axis_jacobian = -target_rotation.transpose() * skew(world_axis) * relative_angular;
}
}  // namespace trajopt_ifopt
