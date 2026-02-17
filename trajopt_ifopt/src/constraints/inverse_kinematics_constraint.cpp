/**
 * @file inverse_kinematics_constraint.cpp
 * @brief The cartesian position constraint
 *
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
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/kinematic_group.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
InverseKinematicsInfo::InverseKinematicsInfo(std::shared_ptr<const tesseract_kinematics::KinematicGroup> manip,
                                             std::string working_frame,
                                             std::string tcp_frame,
                                             const Eigen::Isometry3d& tcp_offset)  // NOLINT(modernize-pass-by-value)
  : manip(std::move(manip))
  , working_frame(std::move(working_frame))
  , tcp_frame(std::move(tcp_frame))
  , tcp_offset(tcp_offset)
{
  if (!this->manip->hasLinkName(this->tcp_frame))
    throw std::runtime_error("Link name '" + this->tcp_frame + "' provided does not exist.");
}

InverseKinematicsConstraint::InverseKinematicsConstraint(
    const Eigen::Isometry3d& target_pose,  // NOLINT(modernize-pass-by-value)
    InverseKinematicsInfo::ConstPtr kinematic_info,
    std::shared_ptr<const Var> constraint_var,
    std::shared_ptr<const Var> seed_var,
    std::string name)
  : ConstraintSet(std::move(name), static_cast<int>(constraint_var->size()))
  , constraint_var_(std::move(constraint_var))
  , seed_var_(std::move(seed_var))
  , target_pose_(target_pose)
  , kinematic_info_(std::move(kinematic_info))
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = constraint_var_->size();
  assert(n_dof_ > 0);
  if (constraint_var_->size() != kinematic_info_->manip->numJoints())
    CONSOLE_BRIDGE_logError("Inverse kinematics has a different number of joints than the given variable set");

  non_zeros_ = n_dof_;
  bounds_ = std::vector<Bounds>(static_cast<std::size_t>(n_dof_), BoundZero);
}

Eigen::VectorXd InverseKinematicsConstraint::getValues() const
{
  // Get joint position using IK and the seed variable
  tesseract_kinematics::KinGroupIKInputs inputs;
  inputs.emplace_back(target_pose_, kinematic_info_->working_frame, kinematic_info_->tcp_frame);

  auto joint_vals = constraint_var_->value();
  auto seed_values = seed_var_->value();
  const tesseract_kinematics::IKSolutions target_joint_position =
      kinematic_info_->manip->calcInvKin(inputs, seed_values);
  assert(!target_joint_position.empty());

  // Calculate joint error
  Eigen::VectorXd error = Eigen::VectorXd::Zero(n_dof_);
  double error_norm = std::numeric_limits<double>::max();
  for (const auto& sol : target_joint_position)
  {
    const Eigen::VectorXd cur_error = sol - joint_vals;
    const double cur_error_norm = cur_error.norm();
    if (cur_error_norm < error_norm)
    {
      error_norm = cur_error_norm;
      error = cur_error;
    }
  }
  return error;
}

Eigen::VectorXd InverseKinematicsConstraint::getCoefficients() const { return Eigen::VectorXd::Constant(n_dof_, 1); }

// Set the limits on the constraint values
std::vector<Bounds> InverseKinematicsConstraint::getBounds() const { return bounds_; }

void InverseKinematicsConstraint::setBounds(const std::vector<Bounds>& bounds)
{
  if (bounds.size() != static_cast<std::size_t>(n_dof_))
    CONSOLE_BRIDGE_logError("Bounds is incorrect size. It is %d when it should be %d", bounds.size(), n_dof_);

  bounds_ = bounds;
}

Jacobian InverseKinematicsConstraint::getJacobian() const
{
  Jacobian jac(rows_, variables_->getRows());
  jac.reserve(non_zeros_);

  // err = target - x =? derr/dx = -1
  for (int j = 0; j < n_dof_; j++)  // NOLINT
  {
    jac.startVec(j);
    jac.insertBack(j, constraint_var_->getIndex() + j) = -1;
  }

  jac.finalize();  // NOLINT
  return jac;
}

void InverseKinematicsConstraint::setTargetPose(const Eigen::Isometry3d& target_pose) { target_pose_ = target_pose; }
}  // namespace trajopt_ifopt
