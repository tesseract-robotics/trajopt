/**
 * @file joint_position_constraint.h
 * @brief The joint position constraint
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
#ifndef TRAJOPT_IFOPT_JOINT_POSITION_CONSTRAINT_H
#define TRAJOPT_IFOPT_JOINT_POSITION_CONSTRAINT_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/constraint_set.h>

namespace trajopt_ifopt
{
class Var;

/**
 * @brief This creates a joint position constraint. Allows bounds to be set on a joint position
 */
class JointPosConstraint : public ConstraintSet
{
public:
  using Ptr = std::shared_ptr<JointPosConstraint>;
  using ConstPtr = std::shared_ptr<const JointPosConstraint>;

  int update() override { return rows_; }

  /**
   * @brief JointPosConstraint
   * @param targets Target joint position (length should be n_dof). Upper and lower bounds are set to this value
   * @param position_vars Variables to which this constraint is applied. Note that all variables should have the same
   * number of components (joint DOF)
   * @param coeffs The joint coefficients to use as weights. If size of 1 then the values is replicated for each joint.
   * @param name Name of the constraint
   */
  JointPosConstraint(const Eigen::VectorXd& target,
                     const std::shared_ptr<const Var>& position_var,
                     const Eigen::VectorXd& coeffs,
                     std::string name = "JointPos",
                     RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  /**
   * @brief JointPosConstraint
   * @param bounds Bounds on target joint position (length should be n_dof)
   * @param position_vars Variables to which this constraint is applied
   * @param coeffs The joint coefficients to use as weights. If size of 1 then the values is replicated for each joint.
   * @param name Name of the constraint
   */
  JointPosConstraint(const std::vector<Bounds>& bounds,
                     const std::shared_ptr<const Var>& position_vars,
                     const Eigen::VectorXd& coeffs,
                     std::string name = "JointPos",
                     RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  /**
   * @brief Returns the values associated with the constraint. In this case that is the concatenated joint values
   * associated with each of the joint positions should be n_dof_ * n_vars_ long
   * @return
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

  /** @brief The coeff to apply to error and gradient */
  Eigen::VectorXd coeffs_;

  /** @brief Bounds on the positions of each joint */
  std::vector<Bounds> bounds_;

  /** @brief This is a vector of indices to be returned Default: {0, 1, 2, ..., n_dof_ - 1} */
  std::vector<int> indices_;

  /** @brief Policy for representing range bounds when building the constraint rows. */
  RangeBoundHandling range_bound_handling_{ RangeBoundHandling::kSplitToTwoInequalities };

  /** @brief Pointers to the vars used by this constraint. */
  std::shared_ptr<const Var> position_var_;
};
}  // namespace trajopt_ifopt
#endif
