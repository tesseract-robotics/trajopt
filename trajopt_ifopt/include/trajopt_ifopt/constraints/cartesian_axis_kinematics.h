/**
 * @file cartesian_axis_kinematics.h
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
#ifndef TRAJOPT_IFOPT_CARTESIAN_AXIS_KINEMATICS_H
#define TRAJOPT_IFOPT_CARTESIAN_AXIS_KINEMATICS_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract/common/eigen_types.h>
#include <tesseract/common/types.h>
#include <tesseract/kinematics/fwd.h>

namespace trajopt_ifopt
{
/**
 * @brief The direction of an axis fixed in a source link, expressed in the coordinates of a target link.
 *
 * Both links belong to one joint group and either or both may move with its joints. For joint values q
 *
 *   v(q) = R_tgt(q)^T R_src(q) a_src
 *
 * where a_src is the unit axis in source link coordinates and R_src, R_tgt are the world orientations of the links.
 * Expressing v in the target frame makes every quantity built on it independent of the world frame and smooth in q
 * whichever link moves.
 *
 * The derivative is exact, from the geometric jacobians of the two links (angular rows, world frame):
 *
 *   dv/dq = -R_tgt^T [w]x (J_w,src - J_w,tgt),   w = R_src a_src
 */
class CartAxisKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Below this |sin(phi)| a tilt direction computed from v is rounding noise: its components carry absolute error of a
   * few machine epsilons (~1e-16), so 1e-12 leaves four orders of margin. Only phi near 0 or pi gets here.
   */
  static constexpr double SIN_ANGLE_FLOOR = 1e-12;

  /**
   * @param manip The joint group both links belong to
   * @param source_frame Link that carries the axis
   * @param source_axis Axis in source link coordinates; normalized here
   * @param target_frame Link whose coordinates the axis is expressed in
   * @throws std::runtime_error if a link is not in @p manip, both links are static, or @p source_axis is zero or not
   * finite
   */
  CartAxisKinematics(std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                     tesseract::common::LinkId source_frame,
                     const Eigen::Vector3d& source_axis,
                     tesseract::common::LinkId target_frame);

  /** @brief The unit source axis in target link coordinates, v(q) */
  Eigen::Vector3d calcAxis(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /**
   * @brief v(q) and its exact derivative dv/dq
   * @param joint_vals Joint values of the group
   * @param axis Output v(q)
   * @param axis_jacobian Output dv/dq, 3 x n_dof
   */
  void calcAxisAndJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                           Eigen::Vector3d& axis,
                           Eigen::Matrix3Xd& axis_jacobian) const;

  /** @brief The number of joints in the group */
  Eigen::Index numJoints() const { return n_dof_; }

  /** @brief The unit source axis in source link coordinates */
  const Eigen::Vector3d& getSourceAxis() const { return source_axis_; }

  /**
   * @brief The unit direction e maximizing |e^T G|, the top left singular vector of @p tangent_jacobian.
   *
   * G maps joint velocities to velocities of v in a 2D tangent basis. Where a tilt direction is undefined (v exactly
   * at a_tgt or -a_tgt), this picks the direction the joints can move v fastest. If G is zero any unit vector is
   * returned; the gradient built from it is zero either way.
   */
  static Eigen::Vector2d mostReachableDirection(const Eigen::Matrix2Xd& tangent_jacobian);

  /**
   * @brief Normalize an axis given by a caller
   * @throws std::runtime_error if @p axis is zero or not finite; @p what names it in the message
   */
  static Eigen::Vector3d toUnitAxis(const Eigen::Vector3d& axis, const std::string& what);

private:
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip_;
  tesseract::common::LinkId source_frame_;
  tesseract::common::LinkId target_frame_;
  Eigen::Vector3d source_axis_;
  bool source_active_;
  bool target_active_;
  Eigen::Index n_dof_;

  /** @brief World orientations of the source and target links */
  void calcRotations(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                     Eigen::Matrix3d& source_rotation,
                     Eigen::Matrix3d& target_rotation) const;

  static thread_local tesseract::common::LinkIdTransformMap transforms_cache_;  // NOLINT
};
}  // namespace trajopt_ifopt
#endif
