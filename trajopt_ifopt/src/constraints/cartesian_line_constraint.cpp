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
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_common/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
CartLineInfo::CartLineInfo(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                           std::string source_frame,
                           std::string target_frame,
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

thread_local tesseract_common::TransformMap CartLineConstraint::transforms_cache_;  // NOLINT

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

  error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                const Eigen::Isometry3d& target_tf,
                                const Eigen::Isometry3d& source_tf,
                                tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
    info_.manip->calcFwdKin(transforms_cache, vals);
    const Eigen::Isometry3d perturbed_source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
    const Eigen::Isometry3d target_tf1 = transforms_cache[info_.target_frame] * info_.target_frame_offset1;
    const Eigen::Isometry3d target_tf2 = transforms_cache[info_.target_frame] * info_.target_frame_offset2;

    // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
    const Eigen::Isometry3d perturbed_target_tf = getLinePoint(perturbed_source_tf, target_tf1, target_tf2);

    Eigen::VectorXd error_diff = tesseract_common::calcJacobianTransformErrorDiff(
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
  const Eigen::Isometry3d source_tf = transforms_cache_[info_.source_frame] * info_.source_frame_offset;
  const Eigen::Isometry3d target_tf1 = transforms_cache_[info_.target_frame] * info_.target_frame_offset1;
  const Eigen::Isometry3d target_tf2 = transforms_cache_[info_.target_frame] * info_.target_frame_offset2;

  // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
  const Eigen::Isometry3d target_tf = getLinePoint(source_tf, target_tf1, target_tf2);

  // pose error is the vector from the new_pose to nearest point on line AB, line_point
  // the below method is equivalent to the position constraint; using the line point as the target point
  Eigen::VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

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
  assert(bounds.size() == 6);
  bounds_ = bounds;
}

void CartLineConstraint::calcJacobianBlock(Jacobian& jac_block,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  transforms_cache_.clear();
  info_.manip->calcFwdKin(transforms_cache_, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache_[info_.source_frame] * info_.source_frame_offset;
  const Eigen::Isometry3d target_tf1 = transforms_cache_[info_.target_frame] * info_.target_frame_offset1;
  const Eigen::Isometry3d target_tf2 = transforms_cache_[info_.target_frame] * info_.target_frame_offset2;

  // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
  const Eigen::Isometry3d target_tf = getLinePoint(source_tf, target_tf1, target_tf2);

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

    for (int i = 0; i < 6; i++)
    {
      jac_block.startVec(i);
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.insertBack(i, position_var_->getIndex() + j) = jac0.coeffRef(info_.indices[i], j);
      }
    }
  }
  else
  {
    // Reserve enough room in the sparse matrix
    info_.manip->calcFwdKin(transforms_cache_, joint_vals);
    const Eigen::Isometry3d source_tf = transforms_cache_[info_.source_frame] * info_.source_frame_offset;
    const Eigen::Isometry3d target_tf1 = transforms_cache_[info_.target_frame] * info_.target_frame_offset1;
    const Eigen::Isometry3d target_tf2 = transforms_cache_[info_.target_frame] * info_.target_frame_offset2;

    // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
    const Eigen::Isometry3d target_tf = getLinePoint(source_tf, target_tf1, target_tf2);

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
    for (int c = 0; c < jac0.cols(); ++c)
    {
      auto perturbed_source_tf = trajopt_common::addTwist(source_tf, jac0.col(c), eps);

      // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
      const Eigen::Isometry3d perturbed_target_tf = getLinePoint(perturbed_source_tf, target_tf1, target_tf2);

      const Eigen::VectorXd error_diff = tesseract_common::calcJacobianTransformErrorDiff(
          target_tf, perturbed_target_tf, source_tf, perturbed_source_tf);

      jac0.col(c).tail(3) = (error_diff / eps);
    }

    // Convert to a sparse matrix and set the jacobian
    // TODO: Make this more efficient. This does not work.
    //    Jacobian jac_block = jac0.sparseView();

    // This does work but could be faster
    for (int i = 0; i < 6; i++)
    {
      jac_block.startVec(i);
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.insertBack(i, position_var_->getIndex() + j) = jac0(info_.indices[i], j);
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

// this has to be const because it is used in const functions, it would be nicer if this could store a member
Eigen::Isometry3d CartLineConstraint::getLinePoint(const Eigen::Isometry3d& source_tf,
                                                   const Eigen::Isometry3d& target_tf1,
                                                   const Eigen::Isometry3d& target_tf2) const
{
  // distance 1; distance from new pose to first point on line
  const Eigen::Vector3d d1 = (source_tf.translation() - target_tf1.translation()).array().abs();

  // Get the line
  const Eigen::Vector3d line = target_tf2.translation() - target_tf1.translation();

  // Point D, the nearest point on line AB to point C, can be found with:
  // (AC - (AC * AB)) * AB
  Eigen::Isometry3d line_point;
  const Eigen::Vector3d line_norm = line.normalized();
  const double mag = d1.dot(line_norm);

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
  const Eigen::Quaterniond quat_a(target_tf1.rotation());
  const Eigen::Quaterniond quat_b(target_tf2.rotation());
  const Eigen::Quaterniond slerp = quat_a.slerp(mag, quat_b);
  line_point.linear() = slerp.toRotationMatrix();

  return line_point;
}

}  // namespace trajopt_ifopt
