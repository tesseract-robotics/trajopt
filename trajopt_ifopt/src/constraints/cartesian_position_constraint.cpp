/**
 * @file cartesian_position_constraint.h
 * @brief The cartesian position constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
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
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt_utils/utils.hpp>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
CartPosConstraint::CartPosConstraint(CartPosInfo info,
                                     JointPosition::ConstPtr position_var,
                                     const Eigen::VectorXd& coeffs,
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

  if (coeffs.rows() != info_.indices.rows())
    std::runtime_error("The number of coeffs does not match the number of constraints.");
}

CartPosConstraint::CartPosConstraint(CartPosInfo info, JointPosition::ConstPtr position_var, const std::string& name)
  : CartPosConstraint(std::move(info), std::move(position_var), Eigen::VectorXd::Ones(info.indices.rows()), name)
{
}

Eigen::VectorXd CartPosConstraint::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  tesseract_common::TransformMap state = info_.manip->calcFwdKin(joint_vals);
  Eigen::Isometry3d source_tf = state[info_.source_frame] * info_.source_frame_offset;
  Eigen::Isometry3d target_tf = state[info_.target_frame] * info_.target_frame_offset;

  VectorXd err;
  if (info_.is_target_active)
    err = tesseract_common::calcTransformError(source_tf, target_tf);
  else
    err = tesseract_common::calcTransformError(target_tf, source_tf);

  VectorXd reduced_err(info_.indices.size());
  for (int i = 0; i < info_.indices.size(); ++i)
    reduced_err[i] = err[info_.indices[i]];

  return coeffs_.cwiseProduct(reduced_err);  // This is available in 3.4 err(indices_, Eigen::all);
}

Eigen::VectorXd CartPosConstraint::GetValues() const
{
  VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  return CalcValues(joint_vals);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> CartPosConstraint::GetBounds() const { return bounds_; }

void CartPosConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 6);
  bounds_ = bounds;
}

void CartPosConstraint::CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                          Jacobian& jac_block) const
{
  auto pose_error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) {
    tesseract_common::TransformMap state = info_.manip->calcFwdKin(x);
    Eigen::Isometry3d source_tf = state[info_.source_frame] * info_.source_frame_offset;
    Eigen::Isometry3d target_tf = state[info_.target_frame] * info_.target_frame_offset;
    if (info_.is_target_active)
      return source_tf.inverse() * target_tf;

    return target_tf.inverse() * source_tf;
  };

  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_ * info_.indices.size());

  if (use_numeric_differentiation)
  {
    auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) {
      Eigen::Isometry3d pose_err = pose_error_calculator(x);
      Eigen::VectorXd err =
          tesseract_common::concat(pose_err.translation(), tesseract_common::calcRotationalError2(pose_err.rotation()));
      return err;
    };
    Jacobian jac0 = calcForwardNumJac(error_calculator, joint_vals, 1e-5);

    for (int i = 0; i < info_.indices.size(); ++i)
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
    // Calculate the jacobian
    tesseract_common::TransformMap state = info_.manip->calcFwdKin(joint_vals);
    Eigen::MatrixXd jac0;
    Eigen::Isometry3d pose_err;
    Eigen::Isometry3d tf0;
    if (info_.is_target_active)
    {
      tf0 = state[info_.target_frame] * info_.target_frame_offset;
      pose_err = (state[info_.source_frame] * info_.source_frame_offset).inverse() * tf0;
      jac0 = info_.manip->calcJacobian(joint_vals, info_.target_frame, info_.target_frame_offset.translation());
      tesseract_common::jacobianChangeBase(jac0, (state[info_.source_frame] * info_.source_frame_offset).inverse());
    }
    else
    {
      tf0 = state[info_.source_frame] * info_.source_frame_offset;
      pose_err = (state[info_.target_frame] * info_.target_frame_offset).inverse() * tf0;
      jac0 = info_.manip->calcJacobian(joint_vals, info_.source_frame, info_.source_frame_offset.translation());
      tesseract_common::jacobianChangeBase(jac0, (state[info_.target_frame] * info_.target_frame_offset).inverse());
    }

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
    for (int i = 0; i < info_.indices.size(); ++i)
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

void CartPosConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var_->GetName())
  {
    // Get current joint values and calculate jacobian
    VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
    CalcJacobianBlock(joint_vals, jac_block);
  }
}

Eigen::Isometry3d CartPosConstraint::GetCurrentPose() const
{
  VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  return info_.manip->calcFwdKin(joint_vals)[info_.source_frame] * info_.source_frame_offset;
}
}  // namespace trajopt_ifopt
