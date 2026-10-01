/**
 * @file cartesian_axis_align_constraint.cpp
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
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
#include <stdexcept>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_axis_align_constraint.h>
#include <trajopt_ifopt/variable_sets/var.h>

namespace trajopt_ifopt
{
namespace
{
/**
 * @brief The logarithm map r = phi / sin(phi) * P v of the sphere at a_tgt, and its jacobian when @p jac is given.
 *
 * @p dv is needed for the jacobian, and for the value only at v = -a_tgt.
 *
 * With s = |P v| = sin(phi), c = a_tgt . v, e = P v / s:
 *
 *   dphi = c de - s dc = (c e^T P - s a_tgt^T) dv
 *   dr   = (phi / s) P dv + e (1 - phi c / s) dphi
 *
 * For s at the floor, phi / s equals 1 to double precision near the solution (phi / sin(phi) = 1 + phi^2 / 6), so
 * dr = P dv exactly. Near v = -a_tgt the direction e is undefined; the most reachable one is used, r = phi e, and the
 * jacobian is the one-sided derivative along e, dr = -e e^T P dv.
 */
void calcLogMap(const Eigen::Matrix<double, 2, 3>& projection,
                const Eigen::Vector3d& target_axis,
                const Eigen::Vector3d& v,
                const Eigen::Matrix3Xd* dv,
                Eigen::Vector2d& r,
                Eigen::Matrix2Xd* jac)
{
  const Eigen::Vector2d tangent = projection * v;
  const double s = tangent.norm();
  const double c = target_axis.dot(v);
  const double angle = std::atan2(s, c);

  if (s > CartAxisKinematics::SIN_ANGLE_FLOOR)
  {
    const Eigen::Vector2d e = tangent / s;
    r = angle * e;
    if (jac != nullptr)
    {
      const Eigen::Matrix2Xd projected = projection * (*dv);
      const Eigen::RowVectorXd dangle = c * e.transpose() * projected - s * target_axis.transpose() * (*dv);
      *jac = (angle / s) * projected + e * (1 - angle * c / s) * dangle;
    }
  }
  else if (c > 0)
  {
    r = tangent;
    if (jac != nullptr)
      *jac = projection * (*dv);
  }
  else
  {
    if (dv == nullptr)
      throw std::logic_error("calcLogMap: the axis jacobian is required at v = -a_tgt.");

    const Eigen::Matrix2Xd projected = projection * (*dv);
    const Eigen::Vector2d e = CartAxisKinematics::mostReachableDirection(projected);
    r = angle * e;
    if (jac != nullptr)
      *jac = -e * (e.transpose() * projected);
  }
}
}  // namespace

CartAxisAlignConstraint::CartAxisAlignConstraint(std::shared_ptr<const Var> position_var,
                                                 std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                                                 tesseract::common::LinkId source_frame,
                                                 const Eigen::Vector3d& source_axis,
                                                 tesseract::common::LinkId target_frame,
                                                 const Eigen::Vector3d& target_axis,
                                                 double coeff,
                                                 std::string name)
  : ConstraintSet(std::move(name), 2)
  , position_var_(std::move(position_var))
  , kin_(std::move(manip), std::move(source_frame), source_axis, std::move(target_frame))
{
  if (!std::isfinite(coeff) || coeff < 0)
    throw std::runtime_error("CartAxisAlignConstraint: coeff must be finite and non-negative, got " +
                             std::to_string(coeff) + ".");

  setTargetAxis(target_axis);
  coeffs_ = Eigen::VectorXd::Constant(2, coeff);
  bounds_ = { BoundZero, BoundZero };
}

Eigen::VectorXd CartAxisAlignConstraint::calcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  Eigen::Vector3d v = kin_.calcAxis(joint_vals);
  Eigen::Vector2d r;
  const bool at_cut_point = (projection_ * v).norm() <= CartAxisKinematics::SIN_ANGLE_FLOOR && target_axis_.dot(v) <= 0;
  if (!at_cut_point)
  {
    calcLogMap(projection_, target_axis_, v, nullptr, r, nullptr);
    return r;
  }

  // At v = -a_tgt the direction of r is the most reachable one, which needs the jacobian
  Eigen::Matrix3Xd dv;
  kin_.calcAxisAndJacobian(joint_vals, v, dv);
  calcLogMap(projection_, target_axis_, v, &dv, r, nullptr);
  return r;
}

Eigen::VectorXd CartAxisAlignConstraint::getValues() const { return calcValues(position_var_->value()); }

Eigen::VectorXd CartAxisAlignConstraint::getCoefficients() const { return coeffs_; }

std::vector<Bounds> CartAxisAlignConstraint::getBounds() const { return bounds_; }

void CartAxisAlignConstraint::calcJacobianBlock(Jacobian& jac_block,
                                                const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  Eigen::Vector3d v;
  Eigen::Matrix3Xd dv;
  kin_.calcAxisAndJacobian(joint_vals, v, dv);
  Eigen::Vector2d r;
  Eigen::Matrix2Xd jac;
  calcLogMap(projection_, target_axis_, v, &dv, r, &jac);

  for (Eigen::Index i = 0; i < 2; ++i)
  {
    jac_block.startVec(i);
    for (Eigen::Index j = 0; j < kin_.numJoints(); ++j)
      jac_block.insertBack(i, position_var_->getIndex() + j) = jac(i, j);
  }
  jac_block.finalize();  // NOLINT
}

Jacobian CartAxisAlignConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(2 * kin_.numJoints());
  calcJacobianBlock(jac, position_var_->value());  // NOLINT
  return jac;
}

void CartAxisAlignConstraint::setTargetAxis(const Eigen::Vector3d& target_axis)
{
  target_axis_ = CartAxisKinematics::toUnitAxis(target_axis, "target axis");

  // p1, p2 are fixed in target link coordinates, so the rows are smooth in q whichever link moves
  const Eigen::Vector3d p1 = target_axis_.unitOrthogonal();
  projection_.row(0) = p1.transpose();
  projection_.row(1) = target_axis_.cross(p1).transpose();
}
}  // namespace trajopt_ifopt
