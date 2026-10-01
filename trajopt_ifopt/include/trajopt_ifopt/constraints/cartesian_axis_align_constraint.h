/**
 * @file cartesian_axis_align_constraint.h
 * @brief Aligns an axis of a link with an axis of another link, leaving rotation about it free
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
#ifndef TRAJOPT_IFOPT_CARTESIAN_AXIS_ALIGN_CONSTRAINT_H
#define TRAJOPT_IFOPT_CARTESIAN_AXIS_ALIGN_CONSTRAINT_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/constraints/cartesian_axis_kinematics.h>

namespace trajopt_ifopt
{
class Var;

/**
 * @brief Points a source link axis along a target link axis; rotation about the axis is unconstrained.
 *
 * With v = R_tgt^T R_src a_src and P the 2 x 3 matrix whose rows p1, p2 are an orthonormal basis of the plane
 * perpendicular to a_tgt (fixed in target link coordinates), two equality rows, in radians:
 *
 *   r = phi / sin(phi) * P v = 0,   phi = atan2(|P v|, a_tgt . v)
 *
 * r is the logarithm map of the sphere at a_tgt: the tilt angle phi times the unit tilt direction P v / |P v|. Its
 * only zero is v = a_tgt, and |r|_1 lies between phi and sqrt(2) phi, so the l1 merit decreases monotonically as the
 * tilt shrinks. The simpler rows P v = 0 also vanish at v = -a_tgt; a half-space row a_tgt . v >= 0 cannot remove that
 * root for a gradient method, since within pi/4 of it the l1 merit decreases towards it.
 *
 * At the solution the jacobian is P dv/dq, with full rank 2. Near v = -a_tgt, the cut point of the logarithm map, it
 * grows like 1/sin(phi); exactly there the tilt direction is undefined and the most reachable one is chosen, so the
 * solver can leave.
 *
 * Unlike CartPosConstraint with the rotation row about the axis dropped, nothing here is built on the rotation vector,
 * so the rows stay continuous when the free rotation passes through pi.
 */
class CartAxisAlignConstraint : public ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartAxisAlignConstraint>;
  using ConstPtr = std::shared_ptr<const CartAxisAlignConstraint>;

  /**
   * @param position_var Joint position variable of the group
   * @param manip The joint group both links belong to
   * @param source_frame Link that carries @p source_axis
   * @param source_axis Axis in source link coordinates; normalized here
   * @param target_frame Link that carries @p target_axis
   * @param target_axis Axis in target link coordinates; normalized here
   * @param coeff Weight of every row, finite and non-negative
   * @param name Name of the constraint set
   * @throws std::runtime_error on an invalid link, axis or coefficient
   */
  CartAxisAlignConstraint(std::shared_ptr<const Var> position_var,
                          std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                          tesseract::common::LinkId source_frame,
                          const Eigen::Vector3d& source_axis,
                          tesseract::common::LinkId target_frame,
                          const Eigen::Vector3d& target_axis,
                          double coeff = 1.0,
                          std::string name = "CartAxisAlign");

  int update() override { return rows_; }

  /** @brief r = phi / sin(phi) * P v, two rows in radians */
  Eigen::VectorXd calcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /** @copydoc Differentiable::getValues */
  Eigen::VectorXd getValues() const override;

  /** @copydoc Differentiable::getCoefficients */
  Eigen::VectorXd getCoefficients() const override;

  /** @brief [0, 0], [0, 0] */
  std::vector<Bounds> getBounds() const override;

  /** @brief Fills the 2 x n_vars jacobian block at @p joint_vals */
  void calcJacobianBlock(Jacobian& jac_block, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /** @copydoc Differentiable::getJacobian */
  Jacobian getJacobian() const override;

  /**
   * @brief Set the axis in target link coordinates; normalized here
   * @throws std::runtime_error if @p target_axis is zero or not finite
   */
  void setTargetAxis(const Eigen::Vector3d& target_axis);

  /** @brief The unit axis in target link coordinates */
  const Eigen::Vector3d& getTargetAxis() const { return target_axis_; }

private:
  std::shared_ptr<const Var> position_var_;
  CartAxisKinematics kin_;
  Eigen::Vector3d target_axis_;
  /** @brief P: rows p1, p2, completing a_tgt to an orthonormal basis of target link coordinates */
  Eigen::Matrix<double, 2, 3> projection_;
  Eigen::VectorXd coeffs_;
  std::vector<Bounds> bounds_;
};
}  // namespace trajopt_ifopt
#endif
