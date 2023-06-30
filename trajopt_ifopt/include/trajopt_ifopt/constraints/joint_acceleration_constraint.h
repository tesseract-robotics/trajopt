/**
 * @file joint_acceleration_constraint.h
 * @brief The joint_acceleration constraint
 *
 * @author Ben Greenberg
 * @date April 22, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TRAJOPT_IFOPT_JOINT_JOINT_ACCELERATION_CONSTRAINT_H
#define TRAJOPT_IFOPT_JOINT_JOINT_ACCELERATION_CONSTRAINT_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/constraint_set.h>
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_ifopt
{
/**
 * @brief This creates a joint acceleration constraint and allows bounds to be set on a joint position
 *
 * Joint acceleration is calculated as a = th_2 - 2th_1 + th_0
 */
class JointAccelConstraint : public ifopt::ConstraintSet
{
public:
  using Ptr = std::shared_ptr<JointAccelConstraint>;
  using ConstPtr = std::shared_ptr<const JointAccelConstraint>;

  /**
   * @brief Constructs a acceleration contraint from these variables, setting the bounds to the target
   * @param targets Joint Acceleration targets (length should be n_dof). Upper and lower bounds are set to this value
   * @param position_vars Joint positions used to calculate acceleration. These vars are assumed to be continuous and in
   * order.
   * @param coeffs The joint coefficients to use as weights. If size of 1 then the values is replicated for each joint.
   * @param name Name of the constraint
   */
  JointAccelConstraint(const Eigen::VectorXd& targets,
                       const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& position_vars,
                       const Eigen::VectorXd& coeffs,
                       const std::string& name = "JointAccel");

  /**
   * @brief Constructs a acceleration contraint from these variables, setting the bounds to those passed in.
   * @param bounds Bounds on target joint acceleration (length should be n_dof)
   * @param position_vars Joint positions used to calculate acceleration. These vars are assumed to be continuous and in
   * order.
   * @param coeffs The joint coefficients to use as weights. If size of 1 then the values is replicated for each joint.
   * @param name Name of the constraint
   */
  JointAccelConstraint(const std::vector<ifopt::Bounds>& bounds,
                       const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& position_vars,
                       const Eigen::VectorXd& coeffs,
                       const std::string& name = "JointAccel");

  /**
   * @brief Returns the values associated with the constraint. In this case that is the approximate joint acceleration.
   * @return Returns jointAcceleration. Length is n_dof_ * n_vars
   */
  Eigen::VectorXd GetValues() const override;

  /**
   * @brief  Returns the "bounds" of this constraint. How these are enforced is up to the solver
   * @return Returns the "bounds" of this constraint
   */
  std::vector<ifopt::Bounds> GetBounds() const override;

  /**
   * @brief Fills the jacobian block associated with the given var_set.
   * @param var_set Name of the var_set to which the jac_block is associated
   * @param jac_block Block of the overall jacobian associated with these constraints and the var_set variable
   */
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;
  /** @brief The number of JointPositions passed in */
  long n_vars_;

  /** @brief The coeff to apply to error and gradient */
  Eigen::VectorXd coeffs_;

  /** @brief Bounds on the velocities of each joint */
  std::vector<ifopt::Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint.
   *
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()*/
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> position_vars_;
  std::unordered_map<std::string, Eigen::Index> index_map_;
};
}  // namespace trajopt_ifopt
#endif
