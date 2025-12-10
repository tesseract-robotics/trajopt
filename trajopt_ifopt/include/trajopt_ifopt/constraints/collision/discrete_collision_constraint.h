/**
 * @file discrete_collision_constraint.h
 * @brief The single timestep collision position constraint
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
#ifndef TRAJOPT_IFOPT_COLLISION_CONSTRAINT_V3_H
#define TRAJOPT_IFOPT_COLLISION_CONSTRAINT_V3_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <ifopt/constraint_set.h>
#include <mutex>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
class Var;
class DiscreteCollisionEvaluator;

class DiscreteCollisionConstraint : public ifopt::ConstraintSet
{
public:
  using Ptr = std::shared_ptr<DiscreteCollisionConstraint>;
  using ConstPtr = std::shared_ptr<const DiscreteCollisionConstraint>;

  DiscreteCollisionConstraint(std::shared_ptr<DiscreteCollisionEvaluator> collision_evaluator,
                              std::shared_ptr<const Var> position_var,
                              int max_num_cnt = 1,
                              bool fixed_sparsity = false,
                              const std::string& name = "DiscreteCollision");

  DiscreteCollisionConstraint(std::shared_ptr<DiscreteCollisionEvaluator> collision_evaluator,
                              std::vector<std::shared_ptr<const Var>> position_vars,
                              int max_num_cnt = 1,
                              bool fixed_sparsity = false,
                              const std::string& name = "DiscreteCollision");

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
  std::shared_ptr<DiscreteCollisionEvaluator> GetCollisionEvaluator() const;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_{ 0 };

  /** @brief The per-position_var max constraints */
  std::size_t max_num_cnt_per_var_{ 0 };

  /** @brief Bounds on the constraint value. Default: std::vector<Bounds>(1, ifopt::BoundSmallerZero) */
  std::vector<ifopt::Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint. */
  std::vector<std::shared_ptr<const Var>> position_vars_;

  std::shared_ptr<DiscreteCollisionEvaluator> collision_evaluator_;

  /** @brief Used to initialize jacobian because snopt sparsity cannot change */
  bool fixed_sparsity_{ false };
  mutable std::string var_set_name_;
  mutable std::once_flag init_flag_;
  mutable std::vector<Eigen::Triplet<double>> triplet_list_;

  void init() const;
};

}  // namespace trajopt_ifopt
#endif
