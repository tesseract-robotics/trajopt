/**
 * @file cartesian_axis_cone_constraint.h
 * @brief Keeps an axis of a link inside a cone around an axis of another link
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
#ifndef TRAJOPT_IFOPT_CARTESIAN_AXIS_CONE_CONSTRAINT_H
#define TRAJOPT_IFOPT_CARTESIAN_AXIS_CONE_CONSTRAINT_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/constraints/cartesian_axis_kinematics.h>

namespace trajopt_ifopt
{
class Var;

/**
 * @brief Keeps the angle between a source link axis and a target link axis at most a half angle theta.
 *
 * One inequality row, in radians:
 *
 *   phi(q) = atan2(|a_tgt x v|, a_tgt . v) <= theta,   v = R_tgt^T R_src a_src
 *
 * The feasible set is a cone of half angle theta around a_tgt; rotation about the source axis stays free. Box bounds
 * on the rotation-vector error of CartPosConstraint can only describe the square inscribed in (or circumscribing) this
 * cone.
 *
 * atan2 keeps full precision at phi near 0 and pi, where acos(a_tgt . v) does not. The gradient,
 *
 *   dphi/dq = -u^T dv/dq,   u = (a_tgt - (a_tgt . v) v) / sin(phi),
 *
 * has unit norm in v, so the row is equally well conditioned for every theta. phi is not differentiable at phi = 0,
 * which lies strictly inside the cone since theta > 0; the jacobian there is zero. At phi = pi every direction
 * perpendicular to v decreases phi; the one the joints can move v along fastest is chosen, so the solver can leave
 * the antiparallel configuration.
 *
 * For theta = 0 the gradient vanishes on the feasible set; use CartAxisAlignConstraint instead.
 */
class CartAxisConeConstraint : public ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartAxisConeConstraint>;
  using ConstPtr = std::shared_ptr<const CartAxisConeConstraint>;

  /**
   * @param position_var Joint position variable of the group
   * @param manip The joint group both links belong to
   * @param source_frame Link that carries @p source_axis
   * @param source_axis Axis in source link coordinates; normalized here
   * @param target_frame Link that carries @p target_axis
   * @param target_axis Cone axis in target link coordinates; normalized here
   * @param half_angle Cone half angle theta in radians, 0 < theta < pi
   * @param coeff Weight of the row, finite and non-negative
   * @param name Name of the constraint set
   * @throws std::runtime_error on an invalid link, axis, half angle or coefficient
   */
  CartAxisConeConstraint(std::shared_ptr<const Var> position_var,
                         std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                         tesseract::common::LinkId source_frame,
                         const Eigen::Vector3d& source_axis,
                         tesseract::common::LinkId target_frame,
                         const Eigen::Vector3d& target_axis,
                         double half_angle,
                         double coeff = 1.0,
                         std::string name = "CartAxisCone");

  int update() override { return rows_; }

  /** @brief The angle phi between the axes in radians, one row */
  Eigen::VectorXd calcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /** @copydoc Differentiable::getValues */
  Eigen::VectorXd getValues() const override;

  /** @copydoc Differentiable::getCoefficients */
  Eigen::VectorXd getCoefficients() const override;

  /** @brief One bound, (-inf, theta] */
  std::vector<Bounds> getBounds() const override;

  /** @brief Fills the 1 x n_vars jacobian block dphi/dq at @p joint_vals */
  void calcJacobianBlock(Jacobian& jac_block, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /** @copydoc Differentiable::getJacobian */
  Jacobian getJacobian() const override;

  /**
   * @brief Set the cone axis in target link coordinates; normalized here
   * @throws std::runtime_error if @p target_axis is zero or not finite
   */
  void setTargetAxis(const Eigen::Vector3d& target_axis);

  /** @brief The unit cone axis in target link coordinates */
  const Eigen::Vector3d& getTargetAxis() const { return target_axis_; }

  /** @brief The cone half angle theta in radians */
  double getHalfAngle() const { return half_angle_; }

private:
  std::shared_ptr<const Var> position_var_;
  CartAxisKinematics kin_;
  Eigen::Vector3d target_axis_;
  double half_angle_;
  Eigen::VectorXd coeffs_;
  std::vector<Bounds> bounds_;
};
}  // namespace trajopt_ifopt
#endif
