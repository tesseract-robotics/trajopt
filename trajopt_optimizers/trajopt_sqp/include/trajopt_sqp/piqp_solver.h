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
#include <piqp/piqp.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/qp_solver.h>

namespace trajopt_sqp
{
/**
 * @brief An interface to the PIQP sparse proximal interior point QP solver
 *
 * Constraint rows are passed to PIQP by kind: rows with equal bounds as equalities, rows with a single nonzero
 * coefficient as variable bounds, and the remaining rows as two-sided inequalities. Rows unbounded on both sides are
 * dropped. The partition is rebuilt on every solve, so any change to the problem data is supported.
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
  PIQPSolver(PIQPSolver&&) = default;
  PIQPSolver& operator=(PIQPSolver&&) = default;

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

  /** @brief The underlying solver, whose result info reports iterations and timings of the last solve */
  const piqp::SparseSolver<double>& solver() const { return solver_; }

  /** @brief Settings applied at every solve. The KKT solver must be a sparse one. */
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

  QPSolverStatus solver_status_{ QPSolverStatus::kUninitialized };
};

}  // namespace trajopt_sqp

#endif
