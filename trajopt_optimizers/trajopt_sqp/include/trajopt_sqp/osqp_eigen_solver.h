/**
 * @file osqp_eigen_solver.h
 * @brief Interface to the OSQPEigen solver
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
#ifndef TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <OsqpEigen/Constants.hpp>
#include <OsqpEigen/Settings.hpp>
#include <optional>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/qp_solver.h>

namespace OsqpEigen
{
class Solver;
}  // namespace OsqpEigen

namespace trajopt_sqp
{
class QPProblem;

/**
 * @brief An Interface to the OSQPEigen QP Solver
 *
 * OSQP copies the settings when it sets a solver up: settings changed through solver_->settings() afterwards take
 * effect at the next set-up only, and in-place updates keep OSQP's copy.
 *
 * Matrix values taken in place reach OSQP in one refactorization at the next bounds update, seed or solve. A
 * matrix-only in-place update keeps OSQP's classification of each row as an equality, an inequality or loose, which
 * sets the row's rho, from the last bounds given; the next bounds update classifies the rows again with the new data.
 * TrustRegionSQPSolver always updates the bounds after the matrices.
 *
 * With adaptive rho on, clear() carries the rho OSQP adapted to the next set-up of this solver, provided the last
 * solve() returned true; the configured rho in the settings is left as it was. Clearing a solver that holds no set-up
 * keeps the rho carried so far. A set-up with adaptive rho off in the settings starts from the configured rho and
 * drops the carry. A solver reused across runs therefore carries rho across them; a new solver starts from the
 * configured rho. A rho adapted during a solve() that returned false does not survive it, in place either:
 * solve() resets OSQP to the configured rho, clamped to OSQP's range. A solve() that fails before OSQP runs, on
 * rejected pending matrices or a failed set-up, resets nothing. solve() returns false even if the refactorization for
 * the reset fails, which cannot happen while P + sigma I is positive definite.
 */
class OSQPEigenSolver : public QPSolver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  OSQPEigenSolver();
  ~OSQPEigenSolver() override;
  OSQPEigenSolver(const OSQPEigenSolver&) = delete;
  OSQPEigenSolver& operator=(const OSQPEigenSolver&) = delete;
  OSQPEigenSolver(OSQPEigenSolver&&) = default;
  OSQPEigenSolver& operator=(OSQPEigenSolver&&) = default;

  static void setDefaultOSQPSettings(OsqpEigen::Settings& settings);

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts) override;

  /** @brief Drop the QP and the workspace; see the class description for the rho the next set-up starts from */
  bool clear() override;

  bool solve() override;

  Eigen::VectorXd getSolution() override;

  /**
   * @brief Load the Hessian; a set-up solver takes it in place only when the sparsity pattern of its upper triangle,
   * explicit zeros included, equals the one the solver was set up with, and refuses any other
   */
  bool updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian) override;

  bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient) override;

  /** @brief Load the lower bounds; see updateBounds() */
  bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound) override;

  /** @brief Load the upper bounds; see updateBounds() */
  bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  /**
   * @brief Load the bounds
   * @details A set-up solver takes any matrix values pending in place first, so OSQP classifies the rows as the new
   * data scales them; false if OSQP rejects those values.
   */
  bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                    const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  /**
   * @brief Load the constraint matrix; a set-up solver takes it in place only when its sparsity pattern, explicit zeros
   * included, equals the one the solver was set up with, and refuses any other
   */
  bool updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& linearConstraintsMatrix) override;

  /** @brief Seed the primal from @p qp_problem and start the duals at zero; ignored with warm starting off */
  bool setWarmStart(const QPProblem& qp_problem) override;

  QPSolverStatus getSolverStatus() const override { return solver_status_; }

  std::unique_ptr<OsqpEigen::Solver> solver_;

private:
  /** @brief Hand OSQP the matrix values taken in place, in one refactorization; false if OSQP rejects them */
  bool applyPendingMatrices();

  // Depending on what they decide to do with this issue, these could be dropped
  // https://github.com/gbionics/osqp-eigen/issues/17
  Eigen::VectorXd x0_;
  Eigen::VectorXd y0_;  // the seed's dual half, passed to OSQP with x0_
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  Eigen::VectorXd gradient_;
  // Matrices taken in place but not yet handed to OSQP, in OSQP's column-major layout; P holds its upper triangle
  Eigen::SparseMatrix<double, Eigen::ColMajor> pending_hessian_;
  Eigen::SparseMatrix<double, Eigen::ColMajor> pending_constraints_;
  bool hessian_pending_{ false };
  bool constraints_pending_{ false };
  Eigen::Index num_vars_{ 0 };
  Eigen::Index num_cnts_{ 0 };

  // The OSQP status of the last solve(); Unsolved when OSQP did not run it, or after clear()
  OsqpEigen::Status last_solve_status_{ OsqpEigen::Status::Unsolved };
  // The adapted rho the next set-up in solve() starts from
  std::optional<double> carried_rho_;

  QPSolverStatus solver_status_{ QPSolverStatus::kUninitialized };
};

}  // namespace trajopt_sqp

#endif
