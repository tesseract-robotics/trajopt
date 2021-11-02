/**
 * @file cartesian_position_constraint.h
 * @brief The cartesian position constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @author Colin Lewis
 * @date December 27, 2020
 * @version TODO
 * @bug No known bugs
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
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt_utils/utils.hpp>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
CartLineInfo::CartLineInfo(tesseract_kinematics::JointGroup::ConstPtr manip,
                           std::string source_frame,
                           std::string target_frame,
                           const Eigen::Isometry3d& target_frame_offset1,  // NOLINT
                           const Eigen::Isometry3d& target_frame_offset2,  // NOLINT
                           const Eigen::Isometry3d& source_frame_offset,   // NOLINT
                           const Eigen::VectorXi& indices)                 // NOLINT
  : manip(std::move(manip))
  , source_frame(std::move(source_frame))
  , target_frame(std::move(target_frame))
  , source_frame_offset(source_frame_offset)
  , target_frame_offset1(target_frame_offset1)
  , target_frame_offset2(target_frame_offset2)
  , indices(indices)
{
  if (!this->manip->hasLinkName(this->source_frame))
    throw std::runtime_error("CartLineInfo: Source Link name '" + this->source_frame + "' provided does not exist.");

  if (!this->manip->hasLinkName(this->target_frame))
    throw std::runtime_error("CartLineInfo: Target Link name '" + this->target_frame + "' provided does not exist.");

  if (this->target_frame_offset1.isApprox(target_frame_offset2))
    throw std::runtime_error("CartLineInfo: The start and end point are the same!");

  if (this->indices.size() > 6)
    throw std::runtime_error("CartLineInfo: The indices list length cannot be larger than six.");

  if (this->indices.size() == 0)
    throw std::runtime_error("CartLineInfo: The indices list length is zero.");
}

CartLineConstraint::CartLineConstraint(CartLineInfo info,
                                       JointPosition::ConstPtr position_var,
                                       const Eigen::VectorXd& coeffs,  // NOLINT
                                       const std::string& name)
  : ifopt::ConstraintSet(static_cast<int>(info.indices.rows()), name)
  , coeffs_(coeffs)
  , position_var_(std::move(position_var))
  , info_(std::move(info))
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = info_.manip->numJoints();
  assert(n_dof_ > 0);

  bounds_ = std::vector<ifopt::Bounds>(static_cast<std::size_t>(info_.indices.rows()), ifopt::BoundZero);

  if (coeffs_.rows() != info_.indices.rows())
    std::runtime_error("The number of coeffs does not match the number of constraints.");
}

Eigen::VectorXd CartLineConstraint::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  tesseract_common::TransformMap state = info_.manip->calcFwdKin(joint_vals);
  Eigen::Isometry3d source_tf = state[info_.source_frame] * info_.source_frame_offset;
  Eigen::Isometry3d target_tf1 = state[info_.target_frame] * info_.target_frame_offset1;
  Eigen::Isometry3d target_tf2 = state[info_.target_frame] * info_.target_frame_offset2;

  // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
  Eigen::Isometry3d target_tf = GetLinePoint(source_tf, target_tf1, target_tf2);

  // pose error is the vector from the new_pose to nearest point on line AB, line_point
  // the below method is equivalent to the position constraint; using the line point as the target point
  VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

  VectorXd reduced_err(info_.indices.size());
  for (int i = 0; i < info_.indices.size(); ++i)
    reduced_err[i] = err[info_.indices[i]];

  return coeffs_.cwiseProduct(reduced_err);  // This is available in 3.4 err(indices_, Eigen::all);
}

Eigen::VectorXd CartLineConstraint::GetValues() const
{
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  return CalcValues(joint_vals);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> CartLineConstraint::GetBounds() const { return bounds_; }

void CartLineConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 6);
  bounds_ = bounds;
}

void CartLineConstraint::CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                           Jacobian& jac_block) const
{
  auto pose_error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) {
    tesseract_common::TransformMap state = info_.manip->calcFwdKin(x);
    Eigen::Isometry3d source_tf = state[info_.source_frame] * info_.source_frame_offset;
    Eigen::Isometry3d target_tf1 = state[info_.target_frame] * info_.target_frame_offset1;
    Eigen::Isometry3d target_tf2 = state[info_.target_frame] * info_.target_frame_offset2;

    // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
    Eigen::Isometry3d target_tf = GetLinePoint(source_tf, target_tf1, target_tf2);

    return target_tf.inverse() * source_tf;
  };

  if (use_numeric_differentiation)
  {
    auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) {
      Eigen::Isometry3d pose_err = pose_error_calculator(x);
      Eigen::VectorXd err =
          tesseract_common::concat(pose_err.translation(), tesseract_common::calcRotationalError2(pose_err.rotation()));
      return err;
    };
    Jacobian jac0 = calcForwardNumJac(error_calculator, joint_vals, 1e-5);

    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.coeffRef(i, j) = coeffs_(i) * jac0.coeffRef(info_.indices[i], j);
      }
    }
  }
  else
  {
    // Reserve enough room in the sparse matrix
    tesseract_common::TransformMap state = info_.manip->calcFwdKin(joint_vals);
    Eigen::Isometry3d source_tf = state[info_.source_frame] * info_.source_frame_offset;
    Eigen::Isometry3d target_tf1 = state[info_.target_frame] * info_.target_frame_offset1;
    Eigen::Isometry3d target_tf2 = state[info_.target_frame] * info_.target_frame_offset2;

    // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
    Eigen::Isometry3d target_tf = GetLinePoint(source_tf, target_tf1, target_tf2);

    Eigen::Isometry3d pose_err = target_tf.inverse() * source_tf;

    Eigen::MatrixXd jac0 =
        info_.manip->calcJacobian(joint_vals, info_.source_frame, info_.source_frame_offset.translation());
    tesseract_common::jacobianChangeBase(jac0, target_tf.inverse());

    // Paper:
    // https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
    // The jacobian of the robot is the geometric jacobian (Je) which maps generalized velocities in
    // joint space to time derivatives of the end-effector configuration representation. It does not
    // represent the analytic jacobian (Ja) given by a partial differentiation of position and rotation
    // to generalized coordinates. Since the geometric jacobian is unique there exists a linear mapping
    // between velocities and the derivatives of the representation.
    //
    // The approach in the paper was tried but it was having issues with getting correct jacobian.
    // Must of had an error in the implementation so should revisit at another time but the approach
    // below should be sufficient and faster than numerical calculations using the err function.

    // The approach below leverages the geometric jacobian and a small step in time to approximate
    // the partial derivative of the error function. Note that the rotational portion is the only part
    // that is required to be modified per the paper.
    Eigen::Vector3d rot_err = tesseract_common::calcRotationalError2(pose_err.rotation());
    for (int c = 0; c < jac0.cols(); ++c)
    {
      auto new_pose_err = util::addTwist(pose_err, jac0.col(c), 1e-5);
      Eigen::VectorXd new_rot_err = tesseract_common::calcRotationalError2(new_pose_err.rotation());
      jac0.col(c).tail(3) = ((new_rot_err - rot_err) / 1e-5);
    }

    // Convert to a sparse matrix and set the jacobian
    // TODO: Make this more efficient. This does not work.
    //    Jacobian jac_block = jac0.sparseView();

    // This does work but could be faster
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.coeffRef(i, j) = coeffs_(i) * jac0(info_.indices[i], j);
      }
    }
  }
}

void CartLineConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var_->GetName())
  {
    // Get current joint values and calculate jacobian
    Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
    CalcJacobianBlock(joint_vals, jac_block);
  }
}

std::pair<Eigen::Isometry3d, Eigen::Isometry3d> CartLineConstraint::GetLine() const
{
  return std::make_pair(info_.target_frame_offset1, info_.target_frame_offset2);
}

const CartLineInfo& CartLineConstraint::GetInfo() const { return info_; }

// this has to be const because it is used in const functions, it would be nicer if this could store a member
Eigen::Isometry3d CartLineConstraint::GetLinePoint(const Eigen::Isometry3d& source_tf,
                                                   const Eigen::Isometry3d& target_tf1,
                                                   const Eigen::Isometry3d& target_tf2) const
{
  // distance 1; distance from new pose to first point on line
  Eigen::Vector3d d1 = (source_tf.translation() - target_tf1.translation()).array().abs();

  // Get the line
  Eigen::Vector3d line = target_tf2.translation() - target_tf1.translation();

  // Point D, the nearest point on line AB to point C, can be found with:
  // (AC - (AC * AB)) * AB
  Eigen::Isometry3d line_point;
  Eigen::Vector3d line_norm = line.normalized();
  double mag = d1.dot(line_norm);

  // If point C is not between the line endpoints, set nearest point to endpoint
  if (mag > 1.0)
  {
    line_point.translation() = info_.target_frame_offset2.translation();
  }
  else if (mag < 0)
  {
    line_point.translation() = info_.target_frame_offset1.translation();
  }
  else
  {
    line_point.translation() = info_.target_frame_offset1.translation() + mag * line_norm;
  }

  // The orientation of the line_point is found using quaternion SLERP
  Eigen::Quaterniond quat_a(target_tf1.rotation());
  Eigen::Quaterniond quat_b(target_tf2.rotation());
  Eigen::Quaterniond slerp = quat_a.slerp(mag, quat_b);
  line_point.linear() = slerp.toRotationMatrix();

  return line_point;
}
}  // namespace trajopt_ifopt
