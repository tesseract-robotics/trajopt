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
#include <tesseract/common/logging.h>
#include <utility>
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

/** @brief Primal and dual regularization of the active-set KKT system, removed again by iterative refinement */
constexpr double kKKTRegularization = 1e-10;
constexpr int kRefinementSteps = 3;
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
  setup_needed_ = true;
  has_prediction_ = false;
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
  setup_needed_ = true;
  has_prediction_ = false;
  used_active_set_ = false;
  solver_status_ = QPSolverStatus::kUninitialized;
  return true;
}

bool PIQPSolver::solve()
{
  used_active_set_ = false;
  const bool predict = active_set_prediction && !settings.enable_derivatives;
  if (predict && has_prediction_ && solveActiveSet())
  {
    used_active_set_ = true;
    reused_setup_ = false;
    solver_status_ = QPSolverStatus::kInitialized;
    return true;
  }

  const double inf = PIQP_INF;
  Eigen::VectorXd x_lower = Eigen::VectorXd::Constant(num_vars_, -inf);
  Eigen::VectorXd x_upper = Eigen::VectorXd::Constant(num_vars_, inf);
  // The rows that set each variable's bounds, for the active-set prediction
  std::vector<Eigen::Index> lower_row(static_cast<std::size_t>(num_vars_), -1);
  std::vector<Eigen::Index> upper_row(static_cast<std::size_t>(num_vars_), -1);
  // The partition rarely differs from that of the last setup
  std::vector<Eigen::Index> eq_rows;
  std::vector<Eigen::Index> ineq_rows;
  eq_rows.reserve(eq_rows_.size());
  ineq_rows.reserve(ineq_rows_.size());
  for (Eigen::Index r = 0; r < num_cnts_; ++r)
  {
    const double lower = bounds_lower_[r];
    const double upper = bounds_upper_[r];
    if (lower <= -inf && upper >= inf)
      continue;

    // A stored zero bounds no variable, so its row is passed as an inequality
    const trajopt_ifopt::Jacobian::InnerIterator it(constraint_matrix_, r);
    if (lower != upper && constraint_matrix_.row(r).nonZeros() == 1 && it.value() != 0.0)
    {
      const double coeff = it.value();
      const double scaled_lower = (coeff > 0) ? lower : -upper;
      const double scaled_upper = (coeff > 0) ? upper : -lower;
      const double var_lower = (scaled_lower <= -inf) ? -inf : scaled_lower / std::abs(coeff);
      const double var_upper = (scaled_upper >= inf) ? inf : scaled_upper / std::abs(coeff);
      const auto col = static_cast<std::size_t>(it.col());
      if (var_lower > x_lower[it.col()])
      {
        x_lower[it.col()] = var_lower;
        lower_row[col] = r;
      }
      if (var_upper < x_upper[it.col()])
      {
        x_upper[it.col()] = var_upper;
        upper_row[col] = r;
      }
      continue;
    }

    (lower == upper ? eq_rows : ineq_rows).push_back(r);
  }

  // Bound rows with disjoint ranges on one variable make the QP infeasible, so fail without calling PIQP
  if ((x_lower.array() > x_upper.array()).any())
  {
    TESSERACT_LOG_DEBUG("PIQP not called: bound rows on one variable have disjoint ranges");
    solver_status_ = QPSolverStatus::kFailed;
    reused_setup_ = false;
    return false;
  }

  const Eigen::VectorXd eq_values = bounds_lower_(eq_rows);
  const Eigen::VectorXd ineq_lower = bounds_lower_(ineq_rows);
  const Eigen::VectorXd ineq_upper = bounds_upper_(ineq_rows);

  solver_.settings() = settings;
  solver_.settings().verbose = settings.verbose || verbosity > 0;

  // PIQP's update leaves the data unscaled when a vector has another size than at setup, which equal row lists and an
  // unchanged variable count rule out. Equal row lists also keep update() from enabling a row that was unbounded on
  // both sides, which PIQP 0.8 rejects without G. PIQP reads enable_derivatives only at setup.
  reused_setup_ = !setup_needed_ && eq_rows == eq_rows_ && ineq_rows == ineq_rows_ &&
                  settings.enable_derivatives == setup_enable_derivatives_;
  if (reused_setup_)
  {
    solver_.update(piqp::nullopt,
                   piqp::nullopt,
                   piqp::nullopt,
                   eq_values,
                   piqp::nullopt,
                   ineq_lower,
                   ineq_upper,
                   x_lower,
                   x_upper);
  }
  else
  {
    const SparseMatrix eq_matrix = selectRows(constraint_matrix_, eq_rows);
    const SparseMatrix ineq_matrix = selectRows(constraint_matrix_, ineq_rows);
    solver_.setup(hessian_, gradient_, eq_matrix, eq_values, ineq_matrix, ineq_lower, ineq_upper, x_lower, x_upper);
    setup_enable_derivatives_ = settings.enable_derivatives;
    eq_rows_ = std::move(eq_rows);
    ineq_rows_ = std::move(ineq_rows);
  }

  const piqp::Status status = solver_.solve();

  // PIQP did not run on these two; any other failure leaves its setup usable
  const bool not_run = status == piqp::Status::PIQP_UNSOLVED || status == piqp::Status::PIQP_INVALID_SETTINGS;
  setup_needed_ = not_run;
  reused_setup_ = reused_setup_ && !not_run;
  if (status == piqp::Status::PIQP_SOLVED)
  {
    if (predict)
      predictActiveSet(lower_row, upper_row);
    solver_status_ = QPSolverStatus::kInitialized;
    return true;
  }

  has_prediction_ = false;

  // PIQP reports rejected settings, such as a KKT solver the sparse backend lacks, only on stderr
  if (not_run)
    TESSERACT_LOG_ERROR("PIQP setup failed with status {} (kkt_solver {})",
                        piqp::status_to_string(status),
                        piqp::kkt_solver_to_string(solver_.settings().kkt_solver));

  solver_status_ = QPSolverStatus::kFailed;
  return false;
}

Eigen::VectorXd PIQPSolver::getSolution() { return used_active_set_ ? active_solution_ : solver_.result().x; }

void PIQPSolver::predictActiveSet(const std::vector<Eigen::Index>& lower_row,
                                  const std::vector<Eigen::Index>& upper_row)
{
  const auto& result = solver_.result();
  std::vector<RowState> state(static_cast<std::size_t>(num_cnts_), RowState::kFree);

  // Count a row as held when its multiplier is clearly positive and it is at the bound; a row at its bound with a
  // vanishing multiplier is left out, so the next solve checks it for feasibility instead
  const double z_scale = 1.0 + std::max({ result.z_l.size() > 0 ? result.z_l.maxCoeff() : 0.0,
                                          result.z_u.size() > 0 ? result.z_u.maxCoeff() : 0.0,
                                          result.z_bl.size() > 0 ? result.z_bl.maxCoeff() : 0.0,
                                          result.z_bu.size() > 0 ? result.z_bu.maxCoeff() : 0.0 });
  const double z_tol = 1e-6 * z_scale;
  const double slack_tol = 10.0 * std::max(settings.eps_abs, 1e-9);
  const Eigen::VectorXd values = constraint_matrix_ * result.x;
  auto at = [&](Eigen::Index r, double bound) {
    return std::abs(values[r] - bound) <= slack_tol * (1.0 + std::abs(bound));
  };

  for (const Eigen::Index r : eq_rows_)
    state[static_cast<std::size_t>(r)] = RowState::kLower;
  for (std::size_t i = 0; i < ineq_rows_.size(); ++i)
  {
    const Eigen::Index r = ineq_rows_[i];
    const auto k = static_cast<Eigen::Index>(i);
    if (result.z_l[k] > z_tol && at(r, bounds_lower_[r]))
      state[static_cast<std::size_t>(r)] = RowState::kLower;
    else if (result.z_u[k] > z_tol && at(r, bounds_upper_[r]))
      state[static_cast<std::size_t>(r)] = RowState::kUpper;
  }
  // A variable bound comes from a row with a single coefficient; a negative one swaps the row's bounds
  for (Eigen::Index c = 0; c < num_vars_; ++c)
  {
    for (const bool upper : { false, true })
    {
      const Eigen::Index r = (upper ? upper_row : lower_row)[static_cast<std::size_t>(c)];
      const double z = upper ? result.z_bu[c] : result.z_bl[c];
      if (r < 0 || z <= z_tol)
        continue;
      const trajopt_ifopt::Jacobian::InnerIterator it(constraint_matrix_, r);
      const bool row_upper = (it.value() > 0) == upper;
      if (at(r, row_upper ? bounds_upper_[r] : bounds_lower_[r]))
        state[static_cast<std::size_t>(r)] = row_upper ? RowState::kUpper : RowState::kLower;
    }
  }

  active_rows_.clear();
  active_states_.clear();
  for (Eigen::Index r = 0; r < num_cnts_; ++r)
  {
    if (state[static_cast<std::size_t>(r)] == RowState::kFree)
      continue;
    active_rows_.push_back(r);
    active_states_.push_back(state[static_cast<std::size_t>(r)]);
  }
  has_prediction_ = true;
}

bool PIQPSolver::solveActiveSet()
{
  if (constraint_matrix_.rows() != num_cnts_ || hessian_.rows() != num_vars_ ||
      (!active_rows_.empty() && active_rows_.back() >= num_cnts_))
    return false;

  // An equality's bounds must still be equal, and a held bound still finite
  const auto k = static_cast<Eigen::Index>(active_rows_.size());
  Eigen::VectorXd rhs(num_vars_ + k);
  rhs.head(num_vars_) = -gradient_;
  for (Eigen::Index j = 0; j < k; ++j)
  {
    const Eigen::Index r = active_rows_[static_cast<std::size_t>(j)];
    const double bound =
        active_states_[static_cast<std::size_t>(j)] == RowState::kUpper ? bounds_upper_[r] : bounds_lower_[r];
    if (std::abs(bound) >= PIQP_INF)
      return false;
    rhs[num_vars_ + j] = bound;
  }

  // Upper triangle of [P, C'; C, 0], with P = hessian_ (PIQP's 2H, upper triangle) and C the held rows
  std::vector<Triplet> triplets;
  triplets.reserve(static_cast<std::size_t>(hessian_.nonZeros() + num_vars_ + 2 * k));
  for (Eigen::Index c = 0; c < hessian_.outerSize(); ++c)
    for (SparseMatrix::InnerIterator it(hessian_, c); it; ++it)
      triplets.emplace_back(static_cast<int>(it.row()), static_cast<int>(it.col()), it.value());
  for (Eigen::Index j = 0; j < k; ++j)
  {
    const Eigen::Index r = active_rows_[static_cast<std::size_t>(j)];
    for (trajopt_ifopt::Jacobian::InnerIterator it(constraint_matrix_, r); it; ++it)
      triplets.emplace_back(static_cast<int>(it.col()), static_cast<int>(num_vars_ + j), it.value());
  }
  SparseMatrix kkt(num_vars_ + k, num_vars_ + k);
  kkt.setFromTriplets(triplets.begin(), triplets.end());

  // Quasi-definite regularization, so that LDL' exists for any ordering
  SparseMatrix regularized = kkt;
  for (Eigen::Index i = 0; i < num_vars_ + k; ++i)
    regularized.coeffRef(i, i) += (i < num_vars_) ? kKKTRegularization : -kKKTRegularization;
  regularized.makeCompressed();

  // Reuse the fill-reducing ordering while the sparsity pattern stays the same
  const bool same_pattern = kkt_outer_.size() == static_cast<std::size_t>(regularized.outerSize() + 1) &&
                            kkt_inner_.size() == static_cast<std::size_t>(regularized.nonZeros()) &&
                            std::equal(kkt_outer_.begin(), kkt_outer_.end(), regularized.outerIndexPtr()) &&
                            std::equal(kkt_inner_.begin(), kkt_inner_.end(), regularized.innerIndexPtr());
  if (!same_pattern)
  {
    kkt_ldlt_.analyzePattern(regularized);
    kkt_outer_.assign(regularized.outerIndexPtr(), regularized.outerIndexPtr() + regularized.outerSize() + 1);
    kkt_inner_.assign(regularized.innerIndexPtr(), regularized.innerIndexPtr() + regularized.nonZeros());
  }
  kkt_ldlt_.factorize(regularized);
  if (kkt_ldlt_.info() != Eigen::Success)
  {
    kkt_outer_.clear();
    return false;
  }

  Eigen::VectorXd sol = kkt_ldlt_.solve(rhs);
  for (int i = 0; i < kRefinementSteps; ++i)
  {
    const Eigen::VectorXd residual = rhs - kkt.selfadjointView<Eigen::Upper>() * sol;
    if (residual.lpNorm<Eigen::Infinity>() <= 1e-14 * (1.0 + rhs.lpNorm<Eigen::Infinity>()))
      break;
    sol += kkt_ldlt_.solve(residual);
  }
  if (!sol.allFinite())
    return false;

  // Accept only a KKT point of the full QP: P x + c + C' lambda = 0 holds by construction
  const Eigen::VectorXd x = sol.head(num_vars_);
  const Eigen::VectorXd lambda = sol.tail(k);
  const double primal_tol = std::max(settings.eps_abs, 1e-9);
  const double dual_tol = primal_tol * (1.0 + (k > 0 ? lambda.lpNorm<Eigen::Infinity>() : 0.0));
  const Eigen::VectorXd values = constraint_matrix_ * x;
  std::vector<bool> held(static_cast<std::size_t>(num_cnts_), false);
  for (Eigen::Index j = 0; j < k; ++j)
  {
    const Eigen::Index r = active_rows_[static_cast<std::size_t>(j)];
    held[static_cast<std::size_t>(r)] = true;
    if (bounds_lower_[r] == bounds_upper_[r])
      continue;
    // Held at its upper bound the row pushes with lambda >= 0, at its lower bound with lambda <= 0
    const bool upper = active_states_[static_cast<std::size_t>(j)] == RowState::kUpper;
    if ((upper && lambda[j] < -dual_tol) || (!upper && lambda[j] > dual_tol))
    {
      if (verbosity > 0)
        TESSERACT_LOG_DEBUG("Active-set prediction rejected: row {} has a multiplier of the wrong sign", r);
      return false;
    }
  }
  for (Eigen::Index r = 0; r < num_cnts_; ++r)
  {
    if (held[static_cast<std::size_t>(r)])
      continue;
    const double tol = primal_tol * (1.0 + std::abs(values[r]));
    if (values[r] < bounds_lower_[r] - tol || values[r] > bounds_upper_[r] + tol)
    {
      if (verbosity > 0)
        TESSERACT_LOG_DEBUG("Active-set prediction rejected: row {} is violated", r);
      return false;
    }
  }

  active_solution_ = x;
  return true;
}

bool PIQPSolver::updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian)
{
  // PIQP minimizes 0.5 * x' * P * x, so P is twice the QP Hessian; it reads only the upper triangle
  hessian_ = hessian.triangularView<Eigen::Upper>();
  hessian_ *= 2.0;
  setup_needed_ = true;
  return true;
}

bool PIQPSolver::updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient)
{
  gradient_ = gradient;
  setup_needed_ = true;
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

  constraint_matrix_ = linearConstraintsMatrix;
  setup_needed_ = true;
  return true;
}

bool PIQPSolver::setWarmStart(const QPProblem& /*qp_problem*/) { return true; }

}  // namespace trajopt_sqp
