/**
 * @file types.h
 * @brief Contains types for the trust region sqp solver
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
#ifndef TRAJOPT_SQP_TYPES_H_
#define TRAJOPT_SQP_TYPES_H_

#include <Eigen/Core>

namespace trajopt_sqp
{
/**
 * @brief Specifies how a constraint-like term is represented in the QP subproblem.
 *
 * This enum describes the penalty model used when a term is incorporated into the
 * convexified QP (objective and/or constraints). Different penalty types imply
 * different bound requirements and different auxiliary (slack) variable handling.
 */
enum class CostPenaltyType : std::uint8_t
{
  /**
   * @brief Squared penalty (least-squares style).
   *
   * Interprets the term as a squared objective contribution, typically of the form:
   * @code
   *   w ∘ (g(x) - target)^2
   * @endcode
   * where @c w are per-row coefficients.
   *
   * Commonly used for "soft equality" costs. In this formulation the term usually
   * expects equality-like bounds (lb == ub) to define the target value.
   *
   * @note This form is handled as a pure objective term (no additional QP constraint
   *       rows or slack variables are required beyond what the solver already uses).
   */
  kSquared,

  /**
   * @brief Absolute-value penalty (L1 / |·| style).
   *
   * Models an equality-like residual with an absolute value cost, conceptually:
   * @code
   *   w ∘ |g(x) - target|
   * @endcode
   *
   * This is typically implemented by introducing auxiliary variable(s) and adding
   * constraint row(s) that relate those variables to the linearized residual so the
   * QP objective can penalize the auxiliary variable(s).
   *
   * Commonly used for robust costs and sparsity-promoting penalties.
   *
   * @note In many setups this expects equality-like bounds (lb == ub) to define the
   *       target/residual reference value.
   */
  kAbsolute,

  /**
   * @brief Hinge penalty (one-sided / max(0, ·) style).
   *
   * Penalizes only violations of an inequality bound, conceptually:
   * @code
   *   w ∘ max(0, g(x) - ub)   // for upper-bound constraints
   *   w ∘ max(0, lb - g(x))   // for lower-bound constraints
   * @endcode
   *
   * This is typically implemented by introducing a nonnegative slack variable and
   * adding a constraint row that couples the slack to the linearized inequality,
   * while the QP objective penalizes the slack.
   *
   * @note This form expects inequality-like bounds (LOWER_BOUND or UPPER_BOUND). Range
   *       bounds may need special handling (e.g., split into two inequalities) prior
   *       to applying a hinge penalty.
   */
  kHinge
};

/**
 * @brief This struct defines parameters for the SQP optimization. The optimization should not change this struct
 */
struct SQPParameters
{
  /** @brief Minimum ratio exact_improve/approx_improve to accept step */
  double improve_ratio_threshold = 0.25;
  /** @brief NLP converges if trust region is smaller than this */
  double min_trust_box_size = 1e-4;
  /** @brief NLP converges if approx_merit_improves is smaller than this */
  double min_approx_improve = 1e-4;
  /** @brief NLP converges if approx_merit_improve / best_exact_merit < min_approx_improve_frac */
  double min_approx_improve_frac = std::numeric_limits<double>::lowest();
  /** @brief Max number of QP calls allowed */
  int max_iterations = 50;

  /** @brief Trust region is scaled by this when it is shrunk */
  double trust_shrink_ratio = 0.1;
  /** @brief Trust region is expanded by this when it is expanded */
  double trust_expand_ratio = 1.5;

  /** @brief Any constraint under this value is not considered a violation */
  double cnt_tolerance = 1e-4;
  /** @brief Max number of times the constraints will be inflated */
  double max_merit_coeff_increases = 5;
  /** @brief Max number of times the QP solver can fail before optimization is aborted */
  int max_qp_solver_failures = 3;
  /** @brief Constraints are scaled by this amount when inflated */
  double merit_coeff_increase_ratio = 10;
  /** @brief Max time in seconds that the optimizer will run */
  double max_time = std::numeric_limits<double>::max();
  /** @brief Initial coefficient that is used to scale the constraints. The total constaint cost is constaint_value
   * coeff * merit_coeff */
  double initial_merit_error_coeff = 10;
  /** @brief If true, only the constraints that are violated will be inflated */
  bool inflate_constraints_individually = true;
  /** @brief Initial size of the trust region */
  double initial_trust_box_size = 1e-1;
  /** @brief Unused */
  bool log_results = false;
  /** @brief Unused */
  std::string log_dir = "/tmp";

  bool operator==(const SQPParameters& rhs) const;
  bool operator!=(const SQPParameters& rhs) const;
};

/** @brief This struct contains information and results for the SQP problem */
struct SQPResults
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SQPResults() = default;
  SQPResults(Eigen::Index num_vars, Eigen::Index num_cnts, Eigen::Index num_costs);

  /** @brief The lowest cost ever achieved */
  double best_exact_merit{ std::numeric_limits<double>::max() };
  /** @brief The cost achieved this iteration */
  double new_exact_merit{ std::numeric_limits<double>::max() };
  /** @brief The lowest convexified cost ever achieved */
  double best_approx_merit{ std::numeric_limits<double>::max() };
  /** @brief The convexified cost achieved this iteration */
  double new_approx_merit{ std::numeric_limits<double>::max() };

  /** @brief Variable values associated with best_exact_merit */
  Eigen::VectorXd best_var_vals;
  /** @brief Variable values associated with this iteration */
  Eigen::VectorXd new_var_vals;

  /** @brief Amount the convexified cost improved over the best this iteration */
  double approx_merit_improve{ 0 };
  /** @brief Amount the exact cost improved over the best this iteration */
  double exact_merit_improve{ 0 };
  /** @brief The amount the cost improved as a ratio of the total cost */
  double merit_improve_ratio{ 0 };

  /** @brief Vector defing the box size. The box is var_vals +/- box_size */
  Eigen::VectorXd box_size;
  /** @brief Coefficients used to weight the constraint violations */
  Eigen::VectorXd merit_error_coeffs;

  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd best_constraint_violations;
  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd new_constraint_violations;

  /** @brief Vector of the convexified constraint violations. Positive is a violation */
  Eigen::VectorXd best_approx_constraint_violations;
  /** @brief Vector of the convexified constraint violations. Positive is a violation */
  Eigen::VectorXd new_approx_constraint_violations;

  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd best_costs;
  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd new_costs;

  /** @brief Vector of the convexified costs.*/
  Eigen::VectorXd best_approx_costs;
  /** @brief Vector of the convexified costs.*/
  Eigen::VectorXd new_approx_costs;

  /** @brief The names associated to constraint violations */
  std::vector<std::string> constraint_names;
  /** @brief The names associated to costs */
  std::vector<std::string> cost_names;

  int penalty_iteration{ 0 };
  int convexify_iteration{ 0 };
  int trust_region_iteration{ 0 };
  int overall_iteration{ 0 };

  void print() const;
};

/**
 * @brief Status codes reported by the SQP solver.
 *
 * These values describe why an SQP run is still in progress, terminated
 * successfully, or stopped early due to limits/errors.
 */
enum class SQPStatus : std::uint8_t
{
  kRunning,               /**< Optimization is currently running */
  kConverged,             /**< Optimization converged successfully */
  kIterationLimit,        /**< Reached SQP iteration limit */
  kPenaltyIterationLimit, /**< Reached penalty-outer-loop iteration limit */
  kTimeLimit,             /**< Reached optimization time limit */
  kQPSolveFailed,         /**< QP solve failed (solver error / no solution returned) */
  kStoppedByCallback      /**< Stopped because callback returned false */
};

}  // namespace trajopt_sqp

#endif
