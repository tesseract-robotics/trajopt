/**
 * @file continuous_collision_constraint.h
 * @brief The continuous collision position constraint
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
#ifndef TRAJOPT_IFOPT_CONTINUOUS_COLLISION_CONSTRAINT_V2_H
#define TRAJOPT_IFOPT_CONTINUOUS_COLLISION_CONSTRAINT_V2_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <ifopt/constraint_set.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
class JointPosition;
class ContinuousCollisionEvaluator;

class ContinuousCollisionConstraint : public ifopt::ConstraintSet
{
public:
  using Ptr = std::shared_ptr<ContinuousCollisionConstraint>;
  using ConstPtr = std::shared_ptr<const ContinuousCollisionConstraint>;

  /**
   * @brief ContinuousCollisionConstraintIfopt
   * @param collision_evaluator The continuous collision evaluator to use
   * @param position_vars The position vars associated with this constraint.
   * @param position_vars_fixed Indicate if the position var is fixed
   * @param max_num_cnt The max number of constraits to include
   * @param fixed_sparsity This is mostly need for snopt which requires sparsity to not change
   * @param name
   */
  ContinuousCollisionConstraint(std::shared_ptr<ContinuousCollisionEvaluator> collision_evaluator,
                                std::array<std::shared_ptr<const JointPosition>, 2> position_vars,
                                bool vars0_fixed,
                                bool vars1_fixed,
                                int max_num_cnt = 1,
                                bool fixed_sparsity = false,
                                const std::string& name = "LVSCollision");

  /**
   * @brief Returns the values associated with the constraint.
   * @warning Make sure that the values returns are not just the violation but the constraint values.
   * Remember the values are the constant in the quadratic function, so if you only return the
   * violation then if it is not violating the constraint this would be zero which means it
   * will always appear to be on the constraint boundary which will cause issue when solving.
   * If it is not voliating the constraint then return the max negative number.
   * If no contacts are found return the negative of the collision margin buffer. This is why
   * it is important to not set the collision margin buffer to zero.
   * @return The constraint values not the violations
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

  /**
   * @brief Sets the bounds on the collision distance
   * @param bounds New bounds that will be set. Should be size 1
   */
  void SetBounds(const std::vector<ifopt::Bounds>& bounds);

  /**
   * @brief Get the collision evaluator. This exposed for plotter callbacks
   * @return The collision evaluator
   */
  std::shared_ptr<ContinuousCollisionEvaluator> GetCollisionEvaluator() const;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief Bounds on the constraint value. Default: std::vector<Bounds>(1, ifopt::BoundSmallerZero) */
  std::vector<ifopt::Bounds> bounds_;

  /**
   * @brief Pointers to the vars used by this constraint.
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()
   */
  std::array<std::shared_ptr<const JointPosition>, 2> position_vars_;
  bool vars0_fixed_{ false };
  bool vars1_fixed_{ false };

  /** @brief Used to initialize jacobian because snopt sparsity cannot change */
  std::vector<Eigen::Triplet<double>> triplet_list_;

  std::shared_ptr<ContinuousCollisionEvaluator> collision_evaluator_;
};
}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_CONTINUOUS_COLLISION_CONSTRAINT_H
