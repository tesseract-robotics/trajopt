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
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
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
CartPosInfo::CartPosInfo(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                         std::string source_frame,
                         std::string target_frame,
                         const Eigen::Isometry3d& source_frame_offset,  // NOLINT(modernize-pass-by-value)
                         const Eigen::Isometry3d& target_frame_offset,  // NOLINT(modernize-pass-by-value)
                         const Eigen::VectorXi& indices)                // NOLINT(modernize-pass-by-value)
  : manip(std::move(manip))
  , source_frame(std::move(source_frame))
  , target_frame(std::move(target_frame))
  , source_frame_offset(source_frame_offset)
  , target_frame_offset(target_frame_offset)
  , indices(indices)
{
  if (!this->manip->hasLinkName(this->source_frame))
    throw std::runtime_error("CartPosInfo: Source Link name '" + this->source_frame + "' provided does not exist.");

  if (!this->manip->hasLinkName(this->target_frame))
    throw std::runtime_error("CartPosInfo: Target Link name '" + this->target_frame + "' provided does not exist.");

  if (this->indices.size() > 6)
    throw std::runtime_error("CartPosInfo: The indices list length cannot be larger than six.");

  if (this->indices.size() == 0)
    throw std::runtime_error("CartPosInfo: The indices list length is zero.");

  const bool target_active = this->manip->isActiveLinkName(this->target_frame);
  const bool source_active = this->manip->isActiveLinkName(this->source_frame);
  if (target_active && source_active)
    type = Type::BOTH_ACTIVE;
  else if (target_active)
    type = Type::TARGET_ACTIVE;
  else if (source_active)
    type = Type::SOURCE_ACTIVE;
  else
    throw std::runtime_error("CartPosInfo: Target and Source are both static links.");
}

thread_local tesseract_common::TransformMap CartPosConstraint::transforms_cache;  // NOLINT

CartPosConstraint::CartPosConstraint(CartPosInfo info,
                                     std::shared_ptr<const JointPosition> position_var,
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

  switch (info_.type)
  {
    case CartPosInfo::Type::TARGET_ACTIVE:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(source_tf, target_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_err[i] = err[info_.indices[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        info_.manip->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;
        Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(source_tf, target_tf, perturbed_target_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_error_diff[i] = error_diff[info_.indices[i]];

        return reduced_error_diff;
      };

      break;
    }
    case CartPosInfo::Type::SOURCE_ACTIVE:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_err[i] = err[info_.indices[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        info_.manip->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
        Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(target_tf, source_tf, perturbed_source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_error_diff[i] = error_diff[info_.indices[i]];

        return reduced_error_diff;
      };
      break;
    }
    case CartPosInfo::Type::BOTH_ACTIVE:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_err[i] = err[info_.indices[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        info_.manip->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
        const Eigen::Isometry3d perturbed_target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;
        Eigen::VectorXd error_diff = tesseract_common::calcJacobianTransformErrorDiff(
            target_tf, perturbed_target_tf, source_tf, perturbed_source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_error_diff[i] = error_diff[info_.indices[i]];

        return reduced_error_diff;
      };
      break;
    }
  }
}

CartPosConstraint::CartPosConstraint(const CartPosInfo& info,
                                     std::shared_ptr<const JointPosition> position_var,
                                     const std::string& name)
  : CartPosConstraint(info, std::move(position_var), Eigen::VectorXd::Ones(info.indices.rows()), name)
{
}

Eigen::VectorXd CartPosConstraint::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  transforms_cache.clear();
  info_.manip->calcFwdKin(transforms_cache, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
  const Eigen::Isometry3d target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;

  const Eigen::VectorXd err = error_function_(target_tf, source_tf);

  return coeffs_.cwiseProduct(err);
}

Eigen::VectorXd CartPosConstraint::GetValues() const
{
  const VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
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
  transforms_cache.clear();
  info_.manip->calcFwdKin(transforms_cache, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
  const Eigen::Isometry3d target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;

  std::vector<Eigen::Triplet<double> > triplet_list;
  triplet_list.reserve(static_cast<std::size_t>(n_dof_ * info_.indices.size()));

  constexpr double eps{ 1e-5 };
  if (use_numeric_differentiation || info_.type == CartPosInfo::Type::BOTH_ACTIVE)
  {
    Eigen::MatrixXd jac0(info_.indices.size(), joint_vals.size());
    Eigen::VectorXd dof_vals_pert = joint_vals;
    for (int i = 0; i < joint_vals.size(); ++i)
    {
      dof_vals_pert(i) = joint_vals(i) + eps;
      const VectorXd error_diff = error_diff_function_(dof_vals_pert, target_tf, source_tf, transforms_cache);
      jac0.col(i) = error_diff / eps;
      dof_vals_pert(i) = joint_vals(i);
    }

    for (int i = 0; i < info_.indices.size(); ++i)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        triplet_list.emplace_back(i, j, coeffs_(i) * jac0(i, j));
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
    info_.manip->calcFwdKin(transforms_cache, joint_vals);
    Eigen::MatrixXd jac0;
    if (info_.type == CartPosInfo::Type::TARGET_ACTIVE)
    {
      jac0 = info_.manip->calcJacobian(joint_vals, info_.target_frame, info_.target_frame_offset.translation());
      tesseract_common::jacobianChangeBase(
          jac0, (transforms_cache[info_.source_frame] * info_.source_frame_offset).inverse());

      for (int c = 0; c < jac0.cols(); ++c)
      {
        auto perturbed_target_tf = trajopt_common::addTwist(target_tf, jac0.col(c), eps);
        const VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(source_tf, target_tf, perturbed_target_tf);
        jac0.col(c).tail(3) = (error_diff / eps);
      }
    }
    else
    {
      jac0 = info_.manip->calcJacobian(joint_vals, info_.source_frame, info_.source_frame_offset.translation());
      tesseract_common::jacobianChangeBase(
          jac0, (transforms_cache[info_.target_frame] * info_.target_frame_offset).inverse());

      for (int c = 0; c < jac0.cols(); ++c)
      {
        auto perturbed_source_tf = trajopt_common::addTwist(source_tf, jac0.col(c), eps);
        const VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(target_tf, source_tf, perturbed_source_tf);
        jac0.col(c).tail(3) = (error_diff / eps);
      }
    }

    // Convert to a sparse matrix and set the jacobian
    for (int i = 0; i < info_.indices.size(); ++i)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        triplet_list.emplace_back(i, j, coeffs_(i) * jac0(info_.indices[i], j));
      }
    }
  }
  jac_block.setFromTriplets(triplet_list.begin(), triplet_list.end());  // NOLINT
}

void CartPosConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set != position_var_->GetName())  // NOLINT
    return;

  // Get current joint values and calculate jacobian
  const VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  CalcJacobianBlock(joint_vals, jac_block);  // NOLINT
}

const CartPosInfo& CartPosConstraint::GetInfo() const { return info_; }
CartPosInfo& CartPosConstraint::GetInfo() { return info_; }

void CartPosConstraint::SetTargetPose(const Eigen::Isometry3d& target_frame_offset)
{
  info_.target_frame_offset = target_frame_offset;
}

Eigen::Isometry3d CartPosConstraint::GetTargetPose() const { return info_.target_frame_offset; }

Eigen::Isometry3d CartPosConstraint::GetCurrentPose() const
{
  transforms_cache.clear();
  const VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  info_.manip->calcFwdKin(transforms_cache, joint_vals);

  return transforms_cache[info_.source_frame] * info_.source_frame_offset;
}

thread_local tesseract_common::TransformMap CartPosConstraint2::transforms_cache;  // NOLINT

CartPosConstraint2::CartPosConstraint2(CartPosInfo info,
                                       std::shared_ptr<const Var> position_var,
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

  switch (info_.type)
  {
    case CartPosInfo::Type::TARGET_ACTIVE:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(source_tf, target_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_err[i] = err[info_.indices[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        info_.manip->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;
        Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(source_tf, target_tf, perturbed_target_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_error_diff[i] = error_diff[info_.indices[i]];

        return reduced_error_diff;
      };

      break;
    }
    case CartPosInfo::Type::SOURCE_ACTIVE:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_err[i] = err[info_.indices[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        info_.manip->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
        Eigen::VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(target_tf, source_tf, perturbed_source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_error_diff[i] = error_diff[info_.indices[i]];

        return reduced_error_diff;
      };
      break;
    }
    case CartPosInfo::Type::BOTH_ACTIVE:
    {
      error_function_ = [this](const Eigen::Isometry3d& target_tf,
                               const Eigen::Isometry3d& source_tf) -> Eigen::VectorXd {
        Eigen::VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_err(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_err[i] = err[info_.indices[i]];

        return reduced_err;
      };

      error_diff_function_ = [this](const Eigen::VectorXd& vals,
                                    const Eigen::Isometry3d& target_tf,
                                    const Eigen::Isometry3d& source_tf,
                                    tesseract_common::TransformMap& transforms_cache) -> Eigen::VectorXd {
        info_.manip->calcFwdKin(transforms_cache, vals);
        const Eigen::Isometry3d perturbed_source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
        const Eigen::Isometry3d perturbed_target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;
        Eigen::VectorXd error_diff = tesseract_common::calcJacobianTransformErrorDiff(
            target_tf, perturbed_target_tf, source_tf, perturbed_source_tf);

        // This is available in 3.4 err(indices_, Eigen::all);
        Eigen::VectorXd reduced_error_diff(info_.indices.size());
        for (int i = 0; i < info_.indices.size(); ++i)
          reduced_error_diff[i] = error_diff[info_.indices[i]];

        return reduced_error_diff;
      };
      break;
    }
  }
}

CartPosConstraint2::CartPosConstraint2(const CartPosInfo& info,
                                       std::shared_ptr<const Var> position_var,
                                       const std::string& name)
  : CartPosConstraint2(info, std::move(position_var), Eigen::VectorXd::Ones(info.indices.rows()), name)
{
}

Eigen::VectorXd CartPosConstraint2::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  transforms_cache.clear();
  info_.manip->calcFwdKin(transforms_cache, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
  const Eigen::Isometry3d target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;

  const Eigen::VectorXd err = error_function_(target_tf, source_tf);

  return coeffs_.cwiseProduct(err);
}

Eigen::VectorXd CartPosConstraint2::GetValues() const { return CalcValues(position_var_->value()); }

// Set the limits on the constraint values
std::vector<ifopt::Bounds> CartPosConstraint2::GetBounds() const { return bounds_; }

void CartPosConstraint2::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 6);
  bounds_ = bounds;
}

void CartPosConstraint2::CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                           Jacobian& jac_block) const
{
  transforms_cache.clear();
  info_.manip->calcFwdKin(transforms_cache, joint_vals);
  const Eigen::Isometry3d source_tf = transforms_cache[info_.source_frame] * info_.source_frame_offset;
  const Eigen::Isometry3d target_tf = transforms_cache[info_.target_frame] * info_.target_frame_offset;

  std::vector<Eigen::Triplet<double> > triplet_list;
  triplet_list.reserve(static_cast<std::size_t>(n_dof_ * info_.indices.size()));

  constexpr double eps{ 1e-5 };
  if (use_numeric_differentiation || info_.type == CartPosInfo::Type::BOTH_ACTIVE)
  {
    Eigen::MatrixXd jac0(info_.indices.size(), joint_vals.size());
    Eigen::VectorXd dof_vals_pert = joint_vals;
    for (int i = 0; i < joint_vals.size(); ++i)
    {
      dof_vals_pert(i) = joint_vals(i) + eps;
      const VectorXd error_diff = error_diff_function_(dof_vals_pert, target_tf, source_tf, transforms_cache);
      jac0.col(i) = error_diff / eps;
      dof_vals_pert(i) = joint_vals(i);
    }

    for (int i = 0; i < info_.indices.size(); ++i)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        triplet_list.emplace_back(i, j, coeffs_(i) * jac0(i, j));
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
    info_.manip->calcFwdKin(transforms_cache, joint_vals);
    Eigen::MatrixXd jac0;
    if (info_.type == CartPosInfo::Type::TARGET_ACTIVE)
    {
      jac0 = info_.manip->calcJacobian(joint_vals, info_.target_frame, info_.target_frame_offset.translation());
      tesseract_common::jacobianChangeBase(
          jac0, (transforms_cache[info_.source_frame] * info_.source_frame_offset).inverse());

      for (int c = 0; c < jac0.cols(); ++c)
      {
        auto perturbed_target_tf = trajopt_common::addTwist(target_tf, jac0.col(c), eps);
        const VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(source_tf, target_tf, perturbed_target_tf);
        jac0.col(c).tail(3) = (error_diff / eps);
      }
    }
    else
    {
      jac0 = info_.manip->calcJacobian(joint_vals, info_.source_frame, info_.source_frame_offset.translation());
      tesseract_common::jacobianChangeBase(
          jac0, (transforms_cache[info_.target_frame] * info_.target_frame_offset).inverse());

      for (int c = 0; c < jac0.cols(); ++c)
      {
        auto perturbed_source_tf = trajopt_common::addTwist(source_tf, jac0.col(c), eps);
        const VectorXd error_diff =
            tesseract_common::calcJacobianTransformErrorDiff(target_tf, source_tf, perturbed_source_tf);
        jac0.col(c).tail(3) = (error_diff / eps);
      }
    }

    // Convert to a sparse matrix and set the jacobian
    for (int i = 0; i < info_.indices.size(); ++i)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        triplet_list.emplace_back(i, j, coeffs_(i) * jac0(info_.indices[i], j));
      }
    }
  }
  jac_block.setFromTriplets(triplet_list.begin(), triplet_list.end());  // NOLINT
}

void CartPosConstraint2::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set != position_var_->getParent()->getParent()->GetName())  // NOLINT
    return;

  // Get current joint values and calculate jacobian
  CalcJacobianBlock(position_var_->value(), jac_block);  // NOLINT
}

const CartPosInfo& CartPosConstraint2::GetInfo() const { return info_; }
CartPosInfo& CartPosConstraint2::GetInfo() { return info_; }

void CartPosConstraint2::SetTargetPose(const Eigen::Isometry3d& target_frame_offset)
{
  info_.target_frame_offset = target_frame_offset;
}

Eigen::Isometry3d CartPosConstraint2::GetTargetPose() const { return info_.target_frame_offset; }

Eigen::Isometry3d CartPosConstraint2::GetCurrentPose() const
{
  transforms_cache.clear();
  info_.manip->calcFwdKin(transforms_cache, position_var_->value());

  return transforms_cache[info_.source_frame] * info_.source_frame_offset;
}

}  // namespace trajopt_ifopt
