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
 * @brief Specifies the penalty a cost set charges for each row's violation of its bounds.
 *
 * A row's violation is its distance outside its bounds, so the bounds decide which directions are charged:
 * both for an equality row (lb == ub), one for a one-sided row. A set may mix the two. A row bounded on both
 * sides with lb < ub, or on neither side, is not supported; express a range as two one-sided rows (see
 * trajopt_ifopt::RangeBoundHandling).
 *
 * @note This is the contract of TrajOptQPProblem. IfoptQPProblem accepts kSquared and kAbsolute only.
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
   * @brief Linear penalty (L1 style).
   *
   * Charges, with @c w the per-row coefficients:
   * @code
   *   w ∘ |g(x) - target|        // equality row
   *   w ∘ max(0, g(x) - ub)      // upper-bound row
   *   w ∘ max(0, lb - g(x))      // lower-bound row
   * @endcode
   *
   * Every row adds a QP constraint row and non-negative slack variables charged through the QP gradient: two
   * for an equality row, one for a one-sided row.
   */
  kAbsolute,

  /** @brief The same linear penalty as kAbsolute. */
  kHinge
};

/**
 * @brief Constraint violations in the two forms the SQP solver consumes, one entry per merit unit.
 * @details @c raw sums a merit unit's row violations, each in its row's own units, excluding rows whose weight is
 * exactly 0; that sum is the feasibility metric compared against SQPParameters::cnt_tolerance. @c weighted sums each
 * row's violation times its weight; the merit function charges it as @c weighted.dot(merit_error_coeffs). A merit
 * unit is a constraint set for TrajOptQPProblem and a constraint row for IfoptQPProblem. IfoptQPProblem applies no
 * per-row weights, so its two forms are equal. Entries are non-negative; 0 means satisfied.
 */
struct ConstraintViolations
{
  Eigen::VectorXd raw;
  Eigen::VectorXd weighted;
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
  /** @brief Max convexifications per penalty iteration; rejected trust-region steps do not count */
  int max_iter = 50;

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

  /** @brief NLP variable values associated with best_exact_merit */
  Eigen::VectorXd best_var_vals;
  /** @brief QP solution of this iteration: the NLP variables followed by the slack variables */
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

  /** @brief Exact constraint violations at best_var_vals */
  ConstraintViolations best_constraint_violations;
  /** @brief Exact constraint violations at new_var_vals */
  ConstraintViolations new_constraint_violations;

  /** @brief Convexified constraint violations at best_var_vals */
  ConstraintViolations best_approx_constraint_violations;
  /** @brief Convexified constraint violations at new_var_vals */
  ConstraintViolations new_approx_constraint_violations;

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
 * @brief Status codes reported by the solver.
 *
 * These values describe why an optimization run is still in progress, terminated
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

/**
 * @brief Return a string representation of the SQPStatus.
 */
std::string toString(SQPStatus status);

}  // namespace trajopt_sqp

#endif
