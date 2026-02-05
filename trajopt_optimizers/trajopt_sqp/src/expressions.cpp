#include <trajopt_sqp/expressions.h>

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

  quad_expr.constants.resize(m);
  quad_expr.linear_coeffs = linear_coeffs;

  if (static_cast<Eigen::Index>(quad_expr.quadratic_coeffs.size()) != m)
    quad_expr.quadratic_coeffs.resize(static_cast<std::size_t>(m));

  // constants: a_i^2 * w_i
  quad_expr.constants = constants.array().square() * weights.array();

  // s_i = 2 * a_i * w_i (scratch)
  scratch_row_scale_.resize(m);
  scratch_row_scale_ = (2.0 * (constants.array() * weights.array())).matrix();

  // linear: scale each row by s_i
  for (Eigen::Index r = 0; r < quad_expr.linear_coeffs.outerSize(); ++r)
  {
    const double sr = scratch_row_scale_[r];
    for (trajopt_ifopt::Jacobian::InnerIterator it(quad_expr.linear_coeffs, static_cast<int>(r)); it; ++it)
      it.valueRef() *= sr;
  }

  // objective_linear = column sums of A
  quad_expr.objective_linear_coeffs.resize(n);
  quad_expr.objective_linear_coeffs.setZero();
  for (int r = 0; r < quad_expr.linear_coeffs.outerSize(); ++r)
    for (trajopt_ifopt::Jacobian::InnerIterator it(quad_expr.linear_coeffs, r); it; ++it)
      quad_expr.objective_linear_coeffs[it.col()] += it.value();

  // ----------------------------
  // NEW: Avoid forming Q_i = w_i * b_i b_i^T (O(k^2)).
  //
  // Store each Q_i as a 1×n sparse row vector q_i = sqrt(w_i) * b_i.
  // Then x^T (w_i b b^T) x == ( (sqrt(w_i) b)^T x )^2 == (q_i * x)^2.
  //
  // Also build objective_quadratic via sparse multiply:
  // H = (diag(sqrt(w)) B)^T (diag(sqrt(w)) B).
  // ----------------------------

  // Build Bw = diag(sqrt(w)) * B by copying and scaling each row
  scratch_bw_ = linear_coeffs;  // one copy, O(nnz)

  // scale rows of Bw by sqrt(w)
  scratch_sqrtw_.resize(m);  // reuse scratch_ for sqrt(w)
  scratch_sqrtw_ = weights.array().sqrt().matrix();

  for (Eigen::Index r = 0; r < scratch_bw_.outerSize(); ++r)
  {
    const double sr = scratch_sqrtw_[r];
    for (trajopt_ifopt::Jacobian::InnerIterator it(scratch_bw_, static_cast<int>(r)); it; ++it)
      it.valueRef() *= sr;
  }

  // objective_quadratic = Bw^T * Bw
  quad_expr.objective_quadratic_coeffs = scratch_bw_.transpose() * scratch_bw_;
  quad_expr.objective_quadratic_coeffs.makeCompressed();

  // Per-expression "quadratic_coeffs[i]" now stores q_i as a 1×n sparse row vector.
  // Use the already-scaled Bw row i directly.
  scratch_qi_trips_.clear();

  for (Eigen::Index i = 0; i < m; ++i)
  {
    auto& Qi = quad_expr.quadratic_coeffs[static_cast<std::size_t>(i)];

    // Build Qi as a 1×n sparse row: Qi(0, col) = Bw(i, col)
    scratch_qi_trips_.clear();

    // Iterate Bw row i (RowMajor outer index is the row)
    for (trajopt_ifopt::Jacobian::InnerIterator it(scratch_bw_, static_cast<int>(i)); it; ++it)
      scratch_qi_trips_.emplace_back(0, it.col(), it.value());

    if (scratch_qi_trips_.empty())
    {
      Qi.resize(0, 0);
      continue;
    }

    Qi.resize(1, n);
    Qi.setFromTriplets(scratch_qi_trips_.begin(), scratch_qi_trips_.end());
    Qi.makeCompressed();
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
