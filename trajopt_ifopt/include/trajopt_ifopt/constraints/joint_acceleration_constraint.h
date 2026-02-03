/**
 * @file joint_acceleration_constraint.h
 * @brief The joint_acceleration constraint
 *
 * @author Ben Greenberg
 * @date April 22, 2021
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
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/constraint_set.h>

namespace trajopt_ifopt
{
class Var;

/**
 * @brief This creates a joint acceleration constraint and allows bounds to be set on a joint position
 *
 * Joint acceleration is calculated as a = th_2 - 2th_1 + th_0
 */
class JointAccelConstraint : public ConstraintSet
{
public:
  using Ptr = std::shared_ptr<JointAccelConstraint>;
  using ConstPtr = std::shared_ptr<const JointAccelConstraint>;

  int update() override { return rows_; }

  /**
   * @brief Constructs a acceleration contraint from these variables, setting the bounds to the target
   * @param targets Joint Acceleration targets (length should be n_dof). Upper and lower bounds are set to this value
   * @param position_vars Joint positions used to calculate acceleration. These vars are assumed to be continuous and in
   * order.
   * @param coeffs The joint coefficients to use as weights. If size of 1 then the values is replicated for each joint.
   * @param name Name of the constraint
   */
  JointAccelConstraint(const Eigen::VectorXd& targets,
                       const std::vector<std::shared_ptr<const Var>>& position_vars,
                       const Eigen::VectorXd& coeffs,
                       std::string name = "JointAccel");

  /**
   * @brief Constructs a acceleration contraint from these variables, setting the bounds to those passed in.
   * @param bounds Bounds on target joint acceleration (length should be n_dof)
   * @param position_vars Joint positions used to calculate acceleration. These vars are assumed to be continuous and in
   * order.
   * @param coeffs The joint coefficients to use as weights. If size of 1 then the values is replicated for each joint.
   * @param name Name of the constraint
   */
  JointAccelConstraint(const std::vector<Bounds>& bounds,
                       const std::vector<std::shared_ptr<const Var>>& position_vars,
                       const Eigen::VectorXd& coeffs,
                       std::string name = "JointAccel");

  /**
   * @brief Returns the values associated with the constraint. In this case that is the approximate joint acceleration.
   * @return Returns jointAcceleration. Length is n_dof_ * n_vars
   */
  Eigen::VectorXd getValues() const override;

  /** @copydoc Differentiable::getCoefficients */
  Eigen::VectorXd getCoefficients() const override;

  /**
   * @brief  Returns the "bounds" of this constraint. How these are enforced is up to the solver
   * @return Returns the "bounds" of this constraint
   */
  std::vector<Bounds> getBounds() const override;

  /** @brief Get the jacobian */
  Jacobian getJacobian() const override;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief The number of JointPositions passed in */
  long n_vars_;

  /** @brief The coeff to apply to error and gradient */
  Eigen::VectorXd coeffs_;

  /** @brief Bounds on the velocities of each joint */
  std::vector<Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint. */
  std::vector<std::shared_ptr<const Var>> position_vars_;
};
}  // namespace trajopt_ifopt
#endif
