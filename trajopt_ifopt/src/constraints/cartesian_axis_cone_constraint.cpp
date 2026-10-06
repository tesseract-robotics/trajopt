/**
 * @file cartesian_axis_cone_constraint.cpp
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
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
#include <stdexcept>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_axis_cone_constraint.h>
#include <trajopt_ifopt/variable_sets/var.h>

namespace trajopt_ifopt
{
namespace
{
/** pi as a double. EIGEN_PI is a long double literal; on x86-64 it exceeds the double M_PI, so compare in double. */
constexpr double PI = static_cast<double>(EIGEN_PI);
}  // namespace

CartAxisConeConstraint::CartAxisConeConstraint(std::shared_ptr<const Var> position_var,
                                               std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                                               tesseract::common::LinkId source_frame,
                                               const Eigen::Vector3d& source_axis,
                                               tesseract::common::LinkId target_frame,
                                               const Eigen::Vector3d& target_axis,
                                               double half_angle,
                                               double coeff,
                                               std::string name)
  : ConstraintSet(std::move(name), 1)
  , position_var_(std::move(position_var))
  , kin_(std::move(manip), std::move(source_frame), source_axis, std::move(target_frame))
  , target_axis_(CartAxisKinematics::toUnitAxis(target_axis, "target axis"))
  , half_angle_(half_angle)
{
  // At theta = 0 the gradient vanishes on the feasible set; at theta >= pi nothing is cut.
  if (std::isnan(half_angle) || half_angle <= 0 || half_angle >= PI)
    throw std::runtime_error("CartAxisConeConstraint: half angle must lie in (0, pi) rad, got " +
                             std::to_string(half_angle) + ". For theta = 0 use CartAxisAlignConstraint.");

  if (!std::isfinite(coeff) || coeff < 0)
    throw std::runtime_error("CartAxisConeConstraint: coeff must be finite and non-negative, got " +
                             std::to_string(coeff) + ".");

  coeffs_ = Eigen::VectorXd::Constant(1, coeff);
  non_zeros_ = kin_.numJoints();
  bounds_ = { Bounds(-double(INFINITY), half_angle_) };
}

Eigen::VectorXd CartAxisConeConstraint::calcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  const Eigen::Vector3d v = kin_.calcAxis(joint_vals);
  return Eigen::VectorXd::Constant(1, std::atan2(target_axis_.cross(v).norm(), target_axis_.dot(v)));
}

Eigen::VectorXd CartAxisConeConstraint::getValues() const { return calcValues(position_var_->value()); }

Eigen::VectorXd CartAxisConeConstraint::getCoefficients() const { return coeffs_; }

std::vector<Bounds> CartAxisConeConstraint::getBounds() const { return bounds_; }

void CartAxisConeConstraint::calcJacobianBlock(Jacobian& jac_block,
                                               const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  Eigen::Vector3d v;
  Eigen::Matrix3Xd dv;
  kin_.calcAxisAndJacobian(joint_vals, v, dv);

  // dphi = -u . dv with u the unit direction from v towards a_tgt, perpendicular to v; |a_tgt - (a_tgt . v) v| = sin
  const double cos_angle = target_axis_.dot(v);
  const Eigen::Vector3d towards_target = target_axis_ - cos_angle * v;
  const double sin_angle = towards_target.norm();

  Eigen::RowVectorXd gradient = Eigen::RowVectorXd::Zero(kin_.numJoints());
  if (sin_angle > CartAxisKinematics::SIN_ANGLE_FLOOR)
  {
    gradient = -(towards_target / sin_angle).transpose() * dv;
  }
  else if (cos_angle < 0)
  {
    // phi = pi: moving v along any tangent direction u decreases phi at rate u . dv; take the most reachable one
    Eigen::Matrix<double, 2, 3> tangent;
    tangent.row(0) = v.unitOrthogonal().transpose();
    tangent.row(1) = v.cross(v.unitOrthogonal()).transpose();
    const Eigen::Matrix2Xd tangent_jacobian = tangent * dv;
    gradient = -CartAxisKinematics::mostReachableDirection(tangent_jacobian).transpose() * tangent_jacobian;
  }
  // else phi = 0: strictly feasible, non-differentiable; zero is a subgradient

  jac_block.startVec(0);
  for (Eigen::Index j = 0; j < kin_.numJoints(); ++j)
    jac_block.insertBack(0, position_var_->getIndex() + j) = gradient(j);
  jac_block.finalize();  // NOLINT
}

Jacobian CartAxisConeConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);
  calcJacobianBlock(jac, position_var_->value());  // NOLINT
  return jac;
}

void CartAxisConeConstraint::setTargetAxis(const Eigen::Vector3d& target_axis)
{
  target_axis_ = CartAxisKinematics::toUnitAxis(target_axis, "target axis");
}
}  // namespace trajopt_ifopt
