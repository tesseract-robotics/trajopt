#ifndef TRAJOPT_IFOPT_SQUARED_COST_H
#define TRAJOPT_IFOPT_SQUARED_COST_H

#include <ifopt/cost_term.h>

#include <Eigen/Eigen>

namespace trajopt
{
/**
 * @brief This class converts a constraint to a cost with a sum squared error
 *
 * IFOPT costs differ from constraints in 2 main ways.
 * 1) Constraints have multiple rows where a cost is simply a value (the sum of all of the costs)
 * 2) Constraints have bounds that they must be within. Costs should simply be as close to 0 as possible
 *
 * This class converts the bounds to a target and then sets the value to be the weighted squared sum of the distances
 * from that target. The result is that the value (GetCost) is a single double and the jacobian (GetJacobian) is a
 * single row (1 x n_vars)
 *
 * Given a constraint: g(x) with some bounds: upper_ and lower_
 *
 * The target of the cost is set to the average of the upper and lower bounds:
 *
 *     target = 1/2 * (upper_ - lower_)
 *
 * and
 *
 *     error(x) = g(x) - target
 *
 * Let
 *
 *     derror(x)/dx = dg(x)/dx = J(x)
 *
 * The squared cost is defined as
 *
 *     cost(x) = error.transpose() * W * error
 *
 * where W is the diagonal matrix of the cost weights
 *
 * Then
 *
 *     dcost(x)/dx = 2 * error.transpose() * W * J(x)
 *
 */
class SquaredCost : public ifopt::CostTerm
{
public:
  using Ptr = std::shared_ptr<SquaredCost>;
  using ConstPtr = std::shared_ptr<const SquaredCost>;

  /**
   * @brief Constructs a CostTerm that converts a constraint into a cost with a sum squared error
   * @param constraint Input constraint to be converted to a cost
   */
  SquaredCost(const ifopt::ConstraintSet::Ptr& constraint);
  /**
   * @brief Constructs a CostTerm that converts a constraint into a cost with a weighted sum squared error
   * @param constraint Input constraint to be converted to a cost
   * @param weights Weights applied to the constraints. Length should be n_constraints
   */
  SquaredCost(const ifopt::ConstraintSet::Ptr& constraint, const Eigen::Ref<const Eigen::VectorXd>& weights);

  double GetCost() const override;

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

private:
  /** @brief Constraint being converted to a cost */
  std::shared_ptr<const ConstraintSet> constraint_;

  /** @brief Size of the input constraint */
  long n_constraints_;
  /** @brief Vector of weights. Default: Eigen::VectorXd::Ones(n_constraints) */
  Eigen::VectorXd weights_;
  /** @brief Vector of targets. By default these are the average of the constraint bounds */
  Eigen::VectorXd targets_;
};

}  // namespace trajopt
#endif
