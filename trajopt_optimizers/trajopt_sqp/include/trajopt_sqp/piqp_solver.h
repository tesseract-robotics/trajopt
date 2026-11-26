/**
 * @file piqp_solver.h
 * @brief Interface to the PIQP solver
 *
 * @author Roelof Oomen
 * @date September 24, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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
#ifndef TRAJOPT_SQP_INCLUDE_PIQP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_PIQP_SOLVER_H_

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/SparseCholesky>
#include <cstdint>
#include <piqp/piqp.hpp>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/qp_solver.h>

namespace trajopt_sqp
{
/**
 * @brief An interface to the PIQP sparse proximal interior point QP solver
 *
 * Constraint rows are passed to PIQP by kind: rows with equal bounds as equalities, rows storing a single
 * coefficient, not zero, as variable bounds, and the remaining rows as two-sided inequalities. Stored zeros count as
 * coefficients. Rows unbounded on both sides are dropped. The partition is rebuilt on every solve, so any change to
 * the problem data is supported.
 *
 * A solve that follows a change of bounds alone keeps PIQP's setup and passes it only the bounds, provided the same
 * rows are equalities and the same rows are inequalities as at the setup. Any other change sets PIQP up again.
 *
 * With active_set_prediction set, a solve first assumes that the rows held at a bound by the last solution still are,
 * and solves the equality-constrained KKT system of those rows with a single sparse LDL' factorization. The candidate
 * is accepted only if it is a KKT point of the full QP to PIQP's tolerances: every other row within its bounds and
 * every held row's multiplier of the right sign. Otherwise PIQP solves the QP, and its solution defines the next
 * prediction. Between SQP iterations near convergence, and between neighbouring problems, the binding rows rarely
 * change, so most solves then cost one factorization instead of one per interior point iteration.
 */
class PIQPSolver : public QPSolver
{
public:
  using Ptr = std::shared_ptr<PIQPSolver>;
  using ConstPtr = std::shared_ptr<const PIQPSolver>;

  PIQPSolver();
  ~PIQPSolver() override;
  PIQPSolver(const PIQPSolver&) = delete;
  PIQPSolver& operator=(const PIQPSolver&) = delete;
  PIQPSolver(PIQPSolver&&) = delete;
  PIQPSolver& operator=(PIQPSolver&&) = delete;

  static void setDefaultPIQPSettings(piqp::Settings<double>& settings);

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts) override;

  bool clear() override;

  bool solve() override;

  Eigen::VectorXd getSolution() override;

  bool updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian) override;

  bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient) override;

  bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound) override;

  bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                    const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  bool updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& linearConstraintsMatrix) override;

  /** @brief Do nothing: the interior point method starts from its own initial point on every solve. */
  bool setWarmStart(const QPProblem& qp_problem) override;

  QPSolverStatus getSolverStatus() const override { return solver_status_; }

  /**
   * @brief The underlying solver, whose result info reports iterations and timings of the last solve
   * @details After a solve that kept the setup, setup_time is still that of the setup; update_time and run_time are
   * those of the last solve.
   */
  const piqp::SparseSolver<double>& solver() const { return solver_; }

  /**
   * @brief The underlying solver, for PIQP's derivatives: jvp() and vjp() after a successful solve with
   * settings.enable_derivatives set
   * @details Their data perturbations and gradients are in PIQP's form of the QP: the equality rows are
   * equalityRows(), the inequality rows inequalityRows(), and the rows that store a single coefficient are variable
   * bounds divided by that coefficient.
   */
  piqp::SparseSolver<double>& solver() { return solver_; }

  /** @brief The constraint rows passed to PIQP as equalities at its last setup, in order */
  const std::vector<Eigen::Index>& equalityRows() const { return eq_rows_; }

  /** @brief The constraint rows passed to PIQP as two-sided inequalities at its last setup, in order */
  const std::vector<Eigen::Index>& inequalityRows() const { return ineq_rows_; }

  /** @brief Whether the last solve kept PIQP's setup and passed it only the bounds; false when PIQP did not run */
  bool reusedSetup() const { return reused_setup_; }

  /** @brief Whether the last solve was the predicted active set's KKT solve, without PIQP */
  bool usedActiveSet() const { return used_active_set_; }

  /**
   * @brief Solve by active-set prediction first, falling back to PIQP (off by default)
   * @details Skipped while settings.enable_derivatives is set, as PIQP's derivatives describe PIQP's own last solve.
   */
  bool active_set_prediction{ false };

  /**
   * @brief Settings applied at every solve. The KKT solver must be a sparse one.
   * @details PIQP reads kkt_solver, preconditioner_iter and preconditioner_scale_cost only when it is set up, so a
   * change to these takes effect at the next solve that does not keep the setup. A change to enable_derivatives, which
   * PIQP also reads only at setup, sets PIQP up again at the next solve.
   */
  piqp::Settings<double> settings;

private:
  using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;

  piqp::SparseSolver<double> solver_;
  SparseMatrix hessian_;
  Eigen::VectorXd gradient_;
  trajopt_ifopt::Jacobian constraint_matrix_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  Eigen::Index num_vars_{ 0 };
  Eigen::Index num_cnts_{ 0 };

  /** @brief The rows PIQP was last set up with as equalities */
  std::vector<Eigen::Index> eq_rows_;
  /** @brief The rows PIQP was last set up with as inequalities */
  std::vector<Eigen::Index> ineq_rows_;
  /** @brief Whether the next solve must set PIQP up: anything but the bounds changed, or PIQP did not run */
  bool setup_needed_{ true };
  bool setup_enable_derivatives_{ false };
  bool reused_setup_{ false };

  /** @brief Where a constraint row was held by the last solution */
  enum class RowState : std::uint8_t
  {
    kFree,
    kLower,  ///< at its lower bound, or an equality
    kUpper,
  };

  /** @brief Whether active_rows_ holds a prediction; an empty one predicts that no row binds */
  bool has_prediction_{ false };
  /** @brief The rows held at a bound by the last solution, in row order, with their states */
  std::vector<Eigen::Index> active_rows_;
  std::vector<RowState> active_states_;
  Eigen::VectorXd active_solution_;
  bool used_active_set_{ false };

  /** @brief The factorization of the last active-set KKT system, kept to reuse its ordering */
  Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper> kkt_ldlt_;
  std::vector<int> kkt_outer_;
  std::vector<int> kkt_inner_;

  /** @brief Solve the KKT system of the predicted active set; true when its solution is a KKT point of the QP */
  bool solveActiveSet();

  /**
   * @brief Predict the next active set from PIQP's solution
   * @param lower_row, upper_row The rows that set each variable's lower and upper bound, -1 for none
   */
  void predictActiveSet(const std::vector<Eigen::Index>& lower_row, const std::vector<Eigen::Index>& upper_row);

  QPSolverStatus solver_status_{ QPSolverStatus::kUninitialized };
};

}  // namespace trajopt_sqp

#endif
