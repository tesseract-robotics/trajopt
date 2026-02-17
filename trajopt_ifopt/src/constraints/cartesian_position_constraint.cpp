/**
 * @file cartesian_position_constraint.h
 * @brief The cartesian position constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
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
thread_local tesseract_common::TransformMap CartPosConstraint::transforms_cache_;  // NOLINT

CartPosConstraint::CartPosConstraint(std::shared_ptr<const Var> position_var,
                                     const Eigen::VectorXd& coeffs,  // NOLINT(modernize-pass-by-value)
                                     const std::vector<Bounds>& bounds,
                                     std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                     std::string source_frame,
                                     std::string target_frame,
                                     const Eigen::Isometry3d& source_frame_offset,  // NOLINT(modernize-pass-by-value)
                                     const Eigen::Isometry3d& target_frame_offset,  // NOLINT(modernize-pass-by-value)
                                     std::string name,
                                     RangeBoundHandling range_bound_handling)
  : ConstraintSet(std::move(name), 6)
  , position_var_(std::move(position_var))
  , range_bound_handling_(range_bound_handling)
  , manip_(std::move(manip))
  , source_frame_(std::move(source_frame))
  , target_frame_(std::move(target_frame))
  , source_frame_offset_(source_frame_offset)
  , target_frame_offset_(target_frame_offset)
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = manip_->numJoints();
  assert(n_dof_ > 0);

  if (!manip_->hasLinkName(source_frame_))
    throw std::runtime_error("Source Link name '" + source_frame_ + "' provided does not exist.");

  if (!manip_->hasLinkName(target_frame_))
    throw std::runtime_error("Target Link name '" + target_frame_ + "' provided does not exist.");

  if (bounds.size() != 6)
    throw std::runtime_error("The number of bounds should be six.");

  if (coeffs.rows() != 6)
    throw std::runtime_error("The number of coeffs should be six.");

  const bool target_active = manip_->isActiveLinkName(target_frame_);
  const bool source_active = manip_->isActiveLinkName(source_frame_);
  if (target_active && source_active)
    type_ = Type::kBothActive;
  else if (target_active)
    type_ = Type::kTargetActive;
  else if (source_active)
    type_ = Type::kSourceActive;
  else
    throw std::runtime_error("CartPosInfo: Target and Source are both static links.");

  std::vector<double> local_coeffs;
  std::vector<int> local_indices;
  local_coeffs.reserve(12);
  local_indices.reserve(12);
  if (range_bound_handling_ == RangeBoundHandling::kSplitToTwoInequalities)
  {
    for (int i = 0; i < 6; ++i)
    {
      if (!tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0))
      {
        const auto& b = bounds[static_cast<std::size_t>(i)];
        if (b.getType() == BoundsType::kRangeBound)
        {
          bounds_.emplace_back(b.getLower(), double(INFINITY));
          bounds_.emplace_back(-double(INFINITY), b.getUpper());
          local_indices.push_back(i);
          local_indices.push_back(i);
          local_coeffs.emplace_back(coeffs[static_cast<Eigen::Index>(i)]);
          local_coeffs.emplace_back(coeffs[static_cast<Eigen::Index>(i)]);
        }
        else
        {
          bounds_.push_back(b);
          local_indices.push_back(i);
          local_coeffs.emplace_back(coeffs[static_cast<Eigen::Index>(i)]);
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < 6; ++i)
    {
      if (!tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0))
      {
        local_indices.push_back(i);
        local_coeffs.push_back(coeffs(i));
        bounds_.push_back(bounds[static_cast<std::size_t>(i)]);
      }
    }
  }

  indices_ = Eigen::Map<Eigen::VectorXi>(local_indices.data(), static_cast<Eigen::Index>(local_indices.size()));
  coeffs_ = Eigen::Map<Eigen::VectorXd>(local_coeffs.data(), static_cast<Eigen::Index>(local_coeffs.size()));
  rows_ = static_cast<int>(indices_.rows());
  non_zeros_ = n_dof_ * indices_.rows();

  switch (type_)
  {
    case Type::kTargetActive:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(source_tf, target_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(indices_.size());
        for (int i = 0; i < indices_.size(); ++i)
          reduced_err[i] = err[indices_[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        manip_->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_target_tf = transforms_cache[target_frame_] * target_frame_offset_;
        Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(source_tf, target_tf, perturbed_target_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(indices_.size());
        for (int i = 0; i < indices_.size(); ++i)
          reduced_error_diff[i] = error_diff[indices_[i]];

        return reduced_error_diff;
      };

      break;
    }
    case Type::kSourceActive:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(indices_.size());
        for (int i = 0; i < indices_.size(); ++i)
          reduced_err[i] = err[indices_[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        manip_->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_source_tf = transforms_cache[source_frame_] * source_frame_offset_;
        Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(target_tf, source_tf, perturbed_source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(indices_.size());
        for (int i = 0; i < indices_.size(); ++i)
          reduced_error_diff[i] = error_diff[indices_[i]];

        return reduced_error_diff;
      };
      break;
    }
    case Type::kBothActive:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(indices_.size());
        for (int i = 0; i < indices_.size(); ++i)
          reduced_err[i] = err[indices_[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        /** @todo This is different from legacy kinematic_terms */
        manip_->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_source_tf = transforms_cache[source_frame_] * source_frame_offset_;
        const Eigen::Isometry3d perturbed_target_tf = transforms_cache[target_frame_] * target_frame_offset_;
        Eigen::VectorXd error_diff = tesseract_common::calcJacobianTransformErrorDiff(
            target_tf, perturbed_target_tf, source_tf, perturbed_source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(indices_.size());
        for (int i = 0; i < indices_.size(); ++i)
          reduced_error_diff[i] = error_diff[indices_[i]];

        return reduced_error_diff;
      };
      break;
    }
  }
}

CartPosConstraint::CartPosConstraint(std::shared_ptr<const Var> position_var,
                                     std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                     std::string source_frame,
                                     std::string target_frame,
                                     const Eigen::Isometry3d& source_frame_offset,
                                     const Eigen::Isometry3d& target_frame_offset,
                                     std::string name,
                                     RangeBoundHandling range_bound_handling)
  : CartPosConstraint(std::move(position_var),
                      Eigen::VectorXd::Ones(6),
                      std::vector<Bounds>(6, BoundZero),
                      std::move(manip),
                      std::move(source_frame),
                      std::move(target_frame),
                      source_frame_offset,  // NOLINT
                      target_frame_offset,  // NOLINT
                      std::move(name),
                      range_bound_handling)
{
}

Eigen::VectorXd CartPosConstraint::calcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  transforms_cache_.clear();
  manip_->calcFwdKin(transforms_cache_, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache_[source_frame_] * source_frame_offset_;
  const Eigen::Isometry3d target_tf = transforms_cache_[target_frame_] * target_frame_offset_;

  const Eigen::VectorXd err = error_function_(target_tf, source_tf);

  return err;
}

Eigen::VectorXd CartPosConstraint::getValues() const { return calcValues(position_var_->value()); }

Eigen::VectorXd CartPosConstraint::getCoefficients() const { return coeffs_; }

std::vector<Bounds> CartPosConstraint::getBounds() const { return bounds_; }

void CartPosConstraint::calcJacobianBlock(Jacobian& jac_block,
                                          const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  transforms_cache_.clear();
  manip_->calcFwdKin(transforms_cache_, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache_[source_frame_] * source_frame_offset_;
  const Eigen::Isometry3d target_tf = transforms_cache_[target_frame_] * target_frame_offset_;

  constexpr double eps{ 1e-5 };
  if (use_numeric_differentiation || type_ == Type::kBothActive)
  {
    Eigen::MatrixXd jac0(indices_.size(), joint_vals.size());
    Eigen::VectorXd dof_vals_pert = joint_vals;
    for (int i = 0; i < joint_vals.size(); ++i)
    {
      dof_vals_pert(i) = joint_vals(i) + eps;
      const Eigen::VectorXd error_diff = error_diff_function_(dof_vals_pert, target_tf, source_tf, transforms_cache_);
      jac0.col(i) = error_diff / eps;
      dof_vals_pert(i) = joint_vals(i);
    }

    for (int i = 0; i < indices_.size(); ++i)
    {
      jac_block.startVec(i);
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.insertBack(i, position_var_->getIndex() + j) = jac0(i, j);
      }
    }
  }
  else
  {
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

    // Calculate the jacobian
    manip_->calcFwdKin(transforms_cache_, joint_vals);
    Eigen::MatrixXd jac0;
    if (type_ == Type::kTargetActive)
    {
      jac0 = manip_->calcJacobian(joint_vals, target_frame_, target_frame_offset_.translation());
      tesseract_common::jacobianChangeBase(jac0, (transforms_cache_[source_frame_] * source_frame_offset_).inverse());

      for (int c = 0; c < jac0.cols(); ++c)
      {
        auto perturbed_target_tf = trajopt_common::addTwist(target_tf, jac0.col(c), eps);
        const Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(source_tf, target_tf, perturbed_target_tf);
        jac0.col(c).tail(3) = (error_diff / eps);
      }
    }
    else
    {
      jac0 = manip_->calcJacobian(joint_vals, source_frame_, source_frame_offset_.translation());
      tesseract_common::jacobianChangeBase(jac0, (transforms_cache_[target_frame_] * target_frame_offset_).inverse());

      for (int c = 0; c < jac0.cols(); ++c)
      {
        auto perturbed_source_tf = trajopt_common::addTwist(source_tf, jac0.col(c), eps);
        const Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(target_tf, source_tf, perturbed_source_tf);
        jac0.col(c).tail(3) = (error_diff / eps);
      }
    }

    // Convert to a sparse matrix and set the jacobian
    for (int i = 0; i < indices_.size(); ++i)
    {
      jac_block.startVec(i);
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.insertBack(i, position_var_->getIndex() + j) = jac0(indices_[i], j);
      }
    }
  }
  jac_block.finalize();  // NOLINT
}

Jacobian CartPosConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  // Get current joint values and calculate jacobian
  calcJacobianBlock(jac, position_var_->value());  // NOLINT
  return jac;
}

void CartPosConstraint::setTargetPose(const Eigen::Isometry3d& target_frame_offset)
{
  target_frame_offset_ = target_frame_offset;
}

Eigen::Isometry3d CartPosConstraint::getTargetPose() const { return target_frame_offset_; }

Eigen::Isometry3d CartPosConstraint::getCurrentPose() const
{
  transforms_cache_.clear();
  manip_->calcFwdKin(transforms_cache_, position_var_->value());

  return transforms_cache_[source_frame_] * source_frame_offset_;
}

}  // namespace trajopt_ifopt
