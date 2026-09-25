/**
 * @file piqp_solver.cpp
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
#include <trajopt_sqp/piqp_solver.h>

#include <algorithm>
#include <cmath>
#include <console_bridge/console.h>
#include <vector>

namespace trajopt_sqp
{
namespace
{
using Triplet = Eigen::Triplet<double, int>;
using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;

/** @brief Stack the given rows of a row-major matrix into a new matrix */
SparseMatrix selectRows(const trajopt_ifopt::Jacobian& matrix, const std::vector<Eigen::Index>& rows)
{
  Eigen::Index nnz = 0;
  for (const Eigen::Index row : rows)
    nnz += matrix.innerVector(row).nonZeros();
  std::vector<Triplet> triplets;
  triplets.reserve(static_cast<std::size_t>(nnz));
  for (std::size_t j = 0; j < rows.size(); ++j)
    for (trajopt_ifopt::Jacobian::InnerIterator it(matrix, rows[j]); it; ++it)
      triplets.emplace_back(static_cast<int>(j), static_cast<int>(it.col()), it.value());

  SparseMatrix selected(static_cast<Eigen::Index>(rows.size()), matrix.cols());
  selected.setFromTriplets(triplets.begin(), triplets.end());
  return selected;
}
}  // namespace

PIQPSolver::PIQPSolver() { setDefaultPIQPSettings(settings); }

PIQPSolver::~PIQPSolver() = default;

void PIQPSolver::setDefaultPIQPSettings(piqp::Settings<double>& settings)
{
  settings = piqp::Settings<double>();
  settings.kkt_solver = piqp::KKTSolver::sparse_ldlt;
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-6;
  settings.check_duality_gap = true;
  settings.max_iter = 250;
  settings.verbose = false;
  settings.compute_timings = false;
}

bool PIQPSolver::init(Eigen::Index num_vars, Eigen::Index num_cnts)
{
  num_vars_ = num_vars;
  num_cnts_ = num_cnts;
  solver_status_ = QPSolverStatus::kInitialized;
  return true;
}

bool PIQPSolver::clear()
{
  num_vars_ = 0;
  num_cnts_ = 0;
  hessian_.resize(0, 0);
  gradient_.resize(0);
  constraint_matrix_.resize(0, 0);
  bounds_lower_.resize(0);
  bounds_upper_.resize(0);
  solver_status_ = QPSolverStatus::kUninitialized;
  return true;
}

bool PIQPSolver::solve()
{
  const double inf = PIQP_INF;
  Eigen::VectorXd x_lower = Eigen::VectorXd::Constant(num_vars_, -inf);
  Eigen::VectorXd x_upper = Eigen::VectorXd::Constant(num_vars_, inf);
  std::vector<Eigen::Index> eq_rows;
  std::vector<Eigen::Index> ineq_rows;
  for (Eigen::Index r = 0; r < num_cnts_; ++r)
  {
    const double lower = bounds_lower_[r];
    const double upper = bounds_upper_[r];
    if (lower <= -inf && upper >= inf)
      continue;

    if (lower != upper && constraint_matrix_.row(r).nonZeros() == 1)
    {
      const trajopt_ifopt::Jacobian::InnerIterator it(constraint_matrix_, r);
      const double coeff = it.value();
      const double scaled_lower = (coeff > 0) ? lower : -upper;
      const double scaled_upper = (coeff > 0) ? upper : -lower;
      const double var_lower = (scaled_lower <= -inf) ? -inf : scaled_lower / std::abs(coeff);
      const double var_upper = (scaled_upper >= inf) ? inf : scaled_upper / std::abs(coeff);
      x_lower[it.col()] = std::max(x_lower[it.col()], var_lower);
      x_upper[it.col()] = std::min(x_upper[it.col()], var_upper);
      continue;
    }

    (lower == upper ? eq_rows : ineq_rows).push_back(r);
  }

  // PIQP does not detect crossed variable bounds and runs to its iteration limit; bound rows with disjoint ranges on
  // one variable make the QP infeasible
  if ((x_lower.array() > x_upper.array()).any())
  {
    CONSOLE_BRIDGE_logDebug("PIQP not called: bound rows on one variable have disjoint ranges");
    solver_status_ = QPSolverStatus::kFailed;
    return false;
  }

  const SparseMatrix eq_matrix = selectRows(constraint_matrix_, eq_rows);
  const SparseMatrix ineq_matrix = selectRows(constraint_matrix_, ineq_rows);
  const Eigen::VectorXd eq_values = bounds_lower_(eq_rows);
  const Eigen::VectorXd ineq_lower = bounds_lower_(ineq_rows);
  const Eigen::VectorXd ineq_upper = bounds_upper_(ineq_rows);

  solver_.settings() = settings;
  solver_.settings().verbose = settings.verbose || verbosity > 0;
  solver_.setup(hessian_, gradient_, eq_matrix, eq_values, ineq_matrix, ineq_lower, ineq_upper, x_lower, x_upper);

  const piqp::Status status = solver_.solve();
  if (status == piqp::Status::PIQP_SOLVED)
  {
    solver_status_ = QPSolverStatus::kInitialized;
    return true;
  }

  // PIQP reports rejected settings, such as a KKT solver the sparse backend lacks, only on stderr
  if (status == piqp::Status::PIQP_UNSOLVED || status == piqp::Status::PIQP_INVALID_SETTINGS)
    CONSOLE_BRIDGE_logError("PIQP setup failed with status %s (kkt_solver %s)",
                            piqp::status_to_string(status),
                            piqp::kkt_solver_to_string(solver_.settings().kkt_solver));

  solver_status_ = QPSolverStatus::kFailed;
  return false;
}

Eigen::VectorXd PIQPSolver::getSolution() { return solver_.result().x; }

bool PIQPSolver::updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian)
{
  // PIQP minimizes 0.5 * x' * P * x, so P is twice the QP Hessian; it reads only the upper triangle
  hessian_ = hessian.triangularView<Eigen::Upper>();
  hessian_ *= 2.0;
  return true;
}

bool PIQPSolver::updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient)
{
  gradient_ = gradient;
  return true;
}

bool PIQPSolver::updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound)
{
  bounds_lower_ = lowerBound.cwiseMax(-PIQP_INF);
  return true;
}

bool PIQPSolver::updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  bounds_upper_ = upperBound.cwiseMin(PIQP_INF);
  return true;
}

bool PIQPSolver::updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                              const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  return updateLowerBound(lowerBound) && updateUpperBound(upperBound);
}

bool PIQPSolver::updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& linearConstraintsMatrix)
{
  assert(num_cnts_ == linearConstraintsMatrix.rows());
  assert(num_vars_ == linearConstraintsMatrix.cols());

  // Drop stored zeros so a row with a single nonzero coefficient is passed as a variable bound
  constraint_matrix_ = linearConstraintsMatrix;
  constraint_matrix_.prune([](Eigen::Index, Eigen::Index, double value) { return value != 0.0; });
  return true;
}

bool PIQPSolver::setWarmStart(const QPProblem& /*qp_problem*/) { return true; }

}  // namespace trajopt_sqp
