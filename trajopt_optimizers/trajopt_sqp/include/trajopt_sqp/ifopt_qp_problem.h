/**
 * @file ifopt_qp_problem.h
 * @brief Converts general NLP to QP for SQP routine
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
#ifndef TRAJOPT_SQP_IFOPT_QP_PROBLEM_H_
#define TRAJOPT_SQP_IFOPT_QP_PROBLEM_H_

#include <memory>
#include <trajopt_sqp/types.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_ifopt/fwd.h>

namespace trajopt_sqp
{
/** @brief Converts a general NLP into a convexified QP that can be solved by a QP solver */
class IfoptQPProblem : public QPProblem
{
public:
  using Ptr = std::shared_ptr<IfoptQPProblem>;
  using ConstPtr = std::shared_ptr<const IfoptQPProblem>;

  IfoptQPProblem(std::shared_ptr<trajopt_ifopt::Variables> variables);
  IfoptQPProblem(std::shared_ptr<trajopt_ifopt::Problem> nlp);

  void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set) override;

  void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set, CostPenaltyType penalty_type) override;

  void setup() override;

  void setVariables(const double* x) override;

  Eigen::VectorXd getVariableValues() const override;

  void convexify() override;

  double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override;

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override;

  double getTotalExactCost() const override;

  Eigen::VectorXd getExactCosts() const override;

  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override;

  Eigen::VectorXd getExactConstraintViolations() const override;

  void scaleBoxSize(double& scale) override;

  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) override;

  void setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff) override;

  /** @brief Prints all members to the terminal in a human readable form */
  void print() const override;

  Eigen::Index getNumNLPVars() const override { return num_nlp_vars_; }
  Eigen::Index getNumNLPConstraints() const override { return num_nlp_cnts_; }
  Eigen::Index getNumNLPCosts() const override { return num_nlp_costs_; }
  Eigen::Index getNumQPVars() const override { return num_qp_vars_; }
  Eigen::Index getNumQPConstraints() const override { return num_qp_cnts_; }

  const std::vector<std::string>& getNLPConstraintNames() const override { return constraint_names_; }
  const std::vector<std::string>& getNLPCostNames() const override { return cost_names_; }

  const Eigen::VectorXd& getBoxSize() const override { return box_size_; }
  const Eigen::VectorXd& getConstraintMeritCoeff() const override { return constraint_merit_coeff_; }

  const trajopt_ifopt::Jacobian& getHessian() const override { return hessian_; }
  const Eigen::VectorXd& getGradient() const override { return gradient_; }

  const trajopt_ifopt::Jacobian& getConstraintMatrix() const override { return constraint_matrix_; }
  const Eigen::VectorXd& getBoundsLower() const override { return bounds_lower_; }
  const Eigen::VectorXd& getBoundsUpper() const override { return bounds_upper_; }

protected:
  std::shared_ptr<trajopt_ifopt::Problem> nlp_;

  Eigen::Index num_nlp_vars_{ 0 };
  Eigen::Index num_nlp_cnts_{ 0 };
  Eigen::Index num_nlp_costs_{ 0 };
  Eigen::Index num_qp_vars_{ 0 };
  Eigen::Index num_qp_cnts_{ 0 };
  std::vector<std::string> constraint_names_;
  std::vector<std::string> cost_names_;

  std::vector<trajopt_ifopt::BoundsType> constraint_types_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;
  Eigen::VectorXd constraint_merit_coeff_;

  trajopt_ifopt::Jacobian hessian_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd cost_constant_;

  trajopt_ifopt::Jacobian constraint_matrix_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  // This should be the center of the bounds
  Eigen::VectorXd constraint_constant_;

  /**
   * @brief Helper that updates the cost QP hessian
   * @details Called by convexify()
   */
  void updateHessian();

  /**
   * @brief Helper that updates the cost gradient including slack variables
   * @details Called by convexify()
   */
  void updateGradient();

  /**
   * @brief Helper that linearizes the constraints about the current point, storing the
   * jacobian as the constraint matrix and adding slack variables.
   * @details Called by convexify()
   */
  void linearizeConstraints();

  /**
   * @brief Helper that updates the NLP constraint bounds (top section)
   * @details Called by convexify()
   */
  void updateNLPConstraintBounds();

  /**
   * @brief Helper that updates the NLP variable bounds (middle section)
   * @details Called by convexify()
   */
  void updateNLPVariableBounds();

  /**
   * @brief Called by convexify() - helper that updates the slack variable bounds (bottom section)
   * @details A slack variable is referred to as an additional variable that has been introduced
   * to the optimization problem to turn a inequality constraint into an equality constraint.
   * Information taken from: https://en.wikipedia.org/wiki/Slack_variable
   *
   * As with the other variables in the augmented constraints, the slack variable cannot take on
   * negative values, as the simplex algorithm requires them to be positive or zero.
   *
   *   - If a slack variable associated with a constraint is zero at a particular candidate solution,
   *     the constraint is binding there, as the constraint restricts the possible changes from that point.
   *   - If a slack variable is positive at a particular candidate solution, the constraint is non-binding
   *     there, as the constraint does not restrict the possible changes from that point.
   *   - If a slack variable is negative at some point, the point is infeasible (not allowed), as it does
   *     not satisfy the constraint.
   *
   * Terminology
   *
   *   - If an inequality constraint holds with equality at the optimal point, the constraint is said to
   *     be binding, as the point cannot be varied in the direction of the constraint even though doing
   *     so would improve the value of the objective function.
   *   - If an inequality constraint holds as a strict inequality at the optimal point (that is, does
   *     not hold with equality), the constraint is said to be non-binding, as the point could be varied
   *     in the direction of the constraint, although it would not be optimal to do so. Under certain
   *     conditions, as for example in convex optimization, if a constraint is non-binding, the optimization
   *     problem would have the same solution even in the absence of that constraint.
   *   - If a constraint is not satisfied at a given point, the point is said to be infeasible.
   *
   * @example By introducing the slack variable y >= 0, the inequality Ax <= b can be converted
   * to the equation Ax + y = b.
   */
  void updateSlackVariableBounds();

  /** @brief This calculates the constant expression in the quadratic expression for the constraints */
  void updateConstraintsConstantExpression();

  /** @brief This calculates the constant expression in the quadratic expression for the costs */
  void updateCostsConstantExpression();
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_IFOPT_QP_PROBLEM_H_
