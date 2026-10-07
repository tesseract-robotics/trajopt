#include <trajopt_sqp/expressions.h>
#include <cassert>
#include <cmath>

namespace trajopt_sqp
{
void AffExprs::values(Eigen::Ref<Eigen::VectorXd> out, const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // Avoid building a temporary for (linear_coeffs * x) + constants
  out = constants;
  out.noalias() += linear_coeffs * x;
}

void AffExprs::create(const Eigen::Ref<const Eigen::VectorXd>& func_error,
                      const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
                      const Eigen::Ref<const Eigen::VectorXd>& x)
{
  // constants = f(x₀) − J(x₀) x₀
  constants.resize(func_error.size());
  constants = func_error;
  constants.noalias() -= func_jacobian * x;

  // Copy Jacobian (alloc avoided only if pattern is identical)
  linear_coeffs = func_jacobian;
}

void AffExprs::square(QuadExprs& quad_expr, const Eigen::Ref<const Eigen::VectorXd>& weights) const
{
  const Eigen::Index m = constants.rows();
  const Eigen::Index n = linear_coeffs.cols();

  assert(linear_coeffs.rows() == m);
  assert(weights.size() == m);

  // A negative weight has no square root and an infinite one overflows: either would put NaN or
  // infinity into the objective Hessian and into that expression's quadratic row.
  assert(weights.allFinite() && (weights.array() >= 0.0).all());

  quad_expr.linear_coeffs = linear_coeffs;

  if (static_cast<Eigen::Index>(quad_expr.quadratic_coeffs.size()) != m)
    quad_expr.quadratic_coeffs.resize(static_cast<std::size_t>(m));

  // constants: a_i^2 * w_i
  quad_expr.constants = constants.array().square() * weights.array();

  // Scale row i of the linear coefficients by 2 * a_i * w_i, accumulating the column sums as
  // they are produced; those sums are the aggregate objective's linear coefficients.
  quad_expr.objective_linear_coeffs.setZero(n);
  for (Eigen::Index r = 0; r < quad_expr.linear_coeffs.outerSize(); ++r)
  {
    const double sr = 2.0 * (constants[r] * weights[r]);
    for (trajopt_ifopt::Jacobian::InnerIterator it(quad_expr.linear_coeffs, static_cast<int>(r)); it; ++it)
    {
      it.valueRef() *= sr;
      quad_expr.objective_linear_coeffs[it.col()] += it.value();
    }
  }

  // Q_i is kept in factored form instead of the materialized rank-one w_i * b_i b_i^T, which would
  // cost O(k^2) per expression: quadratic_coeffs[i] holds the 1×n row q_i = sqrt(w_i) * b_i, so
  // x^T (w_i b b^T) x == (q_i * x)^2. The aggregate objective quadratic comes from the same factor,
  // H = (diag(sqrt(w)) B)^T (diag(sqrt(w)) B).

  // Bw = diag(sqrt(w)) * B: copy B, then scale row r by sqrt(w_r).
  scratch_bw_ = linear_coeffs;
  for (Eigen::Index r = 0; r < scratch_bw_.outerSize(); ++r)
  {
    const double sr = std::sqrt(weights[r]);
    for (trajopt_ifopt::Jacobian::InnerIterator it(scratch_bw_, static_cast<int>(r)); it; ++it)
      it.valueRef() *= sr;
  }

  // objective_quadratic = Bw^T * Bw
  quad_expr.objective_quadratic_coeffs = scratch_bw_.transpose() * scratch_bw_;
  quad_expr.objective_quadratic_coeffs.makeCompressed();

  // Copy row i of Bw into quadratic_coeffs[i].
  for (Eigen::Index i = 0; i < m; ++i)
  {
    auto& Qi = quad_expr.quadratic_coeffs[static_cast<std::size_t>(i)];
    const Eigen::Index nnz = scratch_bw_.innerVector(i).nonZeros();

    if (nnz == 0)
    {
      // An entry that is already empty needs no reset, and Eigen 3.4 reallocates the outer index of
      // an empty matrix on every resize.
      if (Qi.rows() != 0)
        Qi.resize(0, 0);
      continue;
    }

    // resize() clears the outer index, which startVec() requires; it must run on every call
    // because Qi may still hold the row an earlier call built.
    Qi.resize(1, n);
    Qi.reserve(nnz);
    Qi.startVec(0);

    // Row i of Bw is visited in strictly increasing column order, which insertBack() requires.
    for (trajopt_ifopt::Jacobian::InnerIterator it(scratch_bw_, static_cast<int>(i)); it; ++it)
      Qi.insertBack(0, it.col()) = it.value();

    Qi.finalize();
  }
}

QuadExprs::QuadExprs(Eigen::Index num_cost, Eigen::Index num_vars)
  : constants(Eigen::VectorXd::Zero(num_cost))
  , linear_coeffs(num_cost, num_vars)
  , objective_linear_coeffs(Eigen::VectorXd::Zero(num_vars))
  , objective_quadratic_coeffs(num_vars, num_vars)
{
  quadratic_coeffs.reserve(static_cast<std::size_t>(num_cost));
}

void QuadExprs::values(Eigen::Ref<Eigen::VectorXd> out, const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // Start with affine part: c + A x
  out = constants;
  out.noalias() += linear_coeffs * x;

  if (quadratic_coeffs.empty())
    return;

  const auto n_terms = static_cast<Eigen::Index>(quadratic_coeffs.size());
  assert(out.rows() == n_terms);

  // We support two representations:
  //  1) General quadratic: Q_i is n×n -> add x^T Q_i x
  //  2) Squared-affine fast path: Q_i is 1×n row vector q_i -> add (q_i * x)^2
  //
  // Only allocate scratch_ if/when we hit the general quadratic path.
  bool scratch_ready = false;

  for (Eigen::Index i = 0; i < n_terms; ++i)
  {
    const trajopt_ifopt::Jacobian& Q = quadratic_coeffs[static_cast<std::size_t>(i)];
    if (Q.rows() == 0)
      continue;

    if (Q.rows() == 1)
    {
      // Encoded q_i (1×n): add (q_i * x)^2
      double t = 0.0;
      for (trajopt_ifopt::Jacobian::InnerIterator it(Q, 0); it; ++it)
        t += it.value() * x[it.col()];

      out(i) += t * t;
    }
    else
    {
      if (!scratch_ready)
      {
        scratch_.resize(x.size());
        scratch_ready = true;
      }

      // General n×n quadratic: add x^T Q x
      scratch_.noalias() = Q * x;
      out(i) += x.dot(scratch_);
    }
  }
}

void QuadExprs::create(const Eigen::Ref<const Eigen::VectorXd>& func_errors,
                       const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
                       const std::vector<trajopt_ifopt::Jacobian>& func_hessians,
                       const Eigen::Ref<const Eigen::VectorXd>& x)
{
  const Eigen::Index m = func_errors.rows();
  const Eigen::Index n = func_jacobian.cols();

  assert(func_jacobian.rows() == m);
  assert(static_cast<Eigen::Index>(func_hessians.size()) == m);
  assert(x.size() == n);

  linear_coeffs.resize(m, n);
  linear_coeffs.setZero();  // keep existing behavior: rows with no Hessian stay zero

  // constants = f(x₀) − J(x₀) x₀
  constants.resize(m);
  constants = func_errors;
  constants.noalias() -= func_jacobian * x;

  // Vector of per-cost quadratic terms
  if (static_cast<Eigen::Index>(quadratic_coeffs.size()) != m)
    quadratic_coeffs.resize(static_cast<std::size_t>(m));

  // Reuse scratch for H_i * x
  scratch_.resize(n);

  for (Eigen::Index i = 0; i < m; ++i)
  {
    auto& Q = quadratic_coeffs[static_cast<std::size_t>(i)];

    const auto& H = func_hessians[static_cast<std::size_t>(i)];
    if (H.nonZeros() == 0)
    {
      Q.resize(0, 0);
      continue;
    }

    // store Q_i = 1/2 H
    Q = 0.5 * H;
    Q.makeCompressed();

    // ½ xᵀ H x
    scratch_.noalias() = H * x;
    constants(i) += 0.5 * x.dot(scratch_);

    // linear row: J_i − H x
    linear_coeffs.row(i) = func_jacobian.row(i) - scratch_.transpose();
  }
}

}  // namespace trajopt_sqp
