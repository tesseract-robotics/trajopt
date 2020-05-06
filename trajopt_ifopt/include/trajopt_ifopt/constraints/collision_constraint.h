#ifndef TRAJOPT_IFOPT_COLLISION_CONSTRAINT_H
#define TRAJOPT_IFOPT_COLLISION_CONSTRAINT_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/constraint_set.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/utils.hpp>
#include <trajopt/collision_terms.hpp>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt
{
class CollisionConstraintIfopt : public ifopt::ConstraintSet
{
public:
  using Ptr = std::shared_ptr<CollisionConstraintIfopt>;
  using ConstPtr = std::shared_ptr<const CollisionConstraintIfopt>;

  CollisionConstraintIfopt(SingleTimestepCollisionEvaluator::Ptr collision_evaluator,
                           JointPosition::Ptr position_var,
                           const std::string& name = "Collision");

  /** @brief Calculates the values associated with the constraint */
  Eigen::VectorXd CalcValues(const Eigen::Ref<Eigen::VectorXd>& joint_vals) const;
  /**
   * @brief Returns the values associated with the constraint.
   * @return
   */
  Eigen::VectorXd GetValues() const override;

  /**
   * @brief  Returns the "bounds" of this constraint. How these are enforced is up to the solver
   * @return Returns the "bounds" of this constraint
   */
  std::vector<ifopt::Bounds> GetBounds() const override;

  /**
   * @brief Sets the bounds on the collision distance
   * @param bounds New bounds that will be set. Should be size 1
   */
  void SetBounds(const std::vector<ifopt::Bounds>& bounds);

  /**
   * @brief Fills the jacobian block associated with the constraint
   * @param jac_block Block of the overall jacobian associated with these constraints
   */
  void CalcJacobianBlock(const Eigen::Ref<Eigen::VectorXd>& joint_vals, Jacobian& jac_block) const;
  /**
   * @brief Fills the jacobian block associated with the given var_set.
   * @param var_set Name of the var_set to which the jac_block is associated
   * @param jac_block Block of the overall jacobian associated with these constraints and the var_set variable
   */
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief Bounds on the constraint value. Default: std::vector<Bounds>(1, ifopt::BoundSmallerZero) */
  std::vector<ifopt::Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint.
   *
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()*/
  JointPosition::Ptr position_var_;

  SingleTimestepCollisionEvaluator::Ptr collision_evaluator_;
};
};  // namespace trajopt
#endif
