#include <trajopt_sqp/expressions.h>

namespace trajopt_sqp
{
Eigen::VectorXd AffExprs::values(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // Avoid building a temporary for (linear_coeffs * x) + constants
  Eigen::VectorXd result = constants;
  result.noalias() += linear_coeffs * x;
  return result;
}

QuadExprs::QuadExprs(Eigen::Index num_cost, Eigen::Index num_vars)
  : constants(Eigen::VectorXd::Zero(num_cost))
  , linear_coeffs(num_cost, num_vars)
  , objective_linear_coeffs(Eigen::VectorXd::Zero(num_vars))
  , objective_quadratic_coeffs(num_vars, num_vars)
{
  quadratic_coeffs.reserve(static_cast<std::size_t>(num_cost));
}

Eigen::VectorXd QuadExprs::values(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // Start with affine part: c + A x
  Eigen::VectorXd result = constants;
  result.noalias() += linear_coeffs * x;

  if (quadratic_coeffs.empty())
    return result;

  // Reuse a single scratch vector for Q_i * x to avoid per-iteration allocations
  Eigen::VectorXd tmp(x.size());
  const auto n = static_cast<Eigen::Index>(quadratic_coeffs.size());
  assert(result.rows() == n);

  for (Eigen::Index i = 0; i < n; ++i)
  {
    const trajopt_ifopt::Jacobian& Q = quadratic_coeffs[static_cast<std::size_t>(i)];
    if (Q.nonZeros() == 0)
      continue;

    tmp.noalias() = Q * x;
    result(i) += x.dot(tmp);  // xᵀ Q x
  }
  return result;
}

AffExprs createAffExprs(const Eigen::Ref<const Eigen::VectorXd>& func_error,
                        const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
                        const Eigen::Ref<const Eigen::VectorXd>& x)
{
  AffExprs aff_expr;

  // constants = f(x₀) − J(x₀) x₀
  aff_expr.constants = func_error;
  aff_expr.constants.noalias() -= func_jacobian * x;
  aff_expr.linear_coeffs = func_jacobian;

  return aff_expr;
}

QuadExprs createQuadExprs(const Eigen::Ref<const Eigen::VectorXd>& func_errors,
                          const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
                          const std::vector<trajopt_ifopt::Jacobian>& func_hessians,
                          const Eigen::Ref<const Eigen::VectorXd>& x)
{
  const Eigen::Index num_cost = func_errors.rows();
  const Eigen::Index num_vars = func_jacobian.cols();

  QuadExprs quad_exprs(num_cost, num_vars);

  // constants = f(x₀) − J(x₀) x₀
  quad_exprs.constants = func_errors;
  quad_exprs.constants.noalias() -= func_jacobian * x;

  quad_exprs.linear_coeffs.resize(func_jacobian.rows(), func_jacobian.cols());
  quad_exprs.linear_coeffs.setZero();  // keep existing behavior: rows with no Hessian stay zero
  quad_exprs.quadratic_coeffs.resize(static_cast<std::size_t>(num_cost));

  // Reuse scratch for H_i * x
  Eigen::VectorXd tmp(num_vars);

  for (Eigen::Index i = 0; i < num_cost; ++i)
  {
    const auto& H = func_hessians[static_cast<std::size_t>(i)];
    if (H.nonZeros() == 0)
      continue;

    auto& Q = quad_exprs.quadratic_coeffs[static_cast<std::size_t>(i)];
    Q = 0.5 * H;  // store ½ H

    // ½ xᵀ H x
    tmp.noalias() = H * x;
    quad_exprs.constants(i) += 0.5 * x.dot(tmp);

    // linear row: J_i − H x
    quad_exprs.linear_coeffs.row(i) = func_jacobian.row(i) - tmp.transpose();
  }

  return quad_exprs;
}

QuadExprs squareAffExprs(const AffExprs& aff_expr, const Eigen::Ref<const Eigen::VectorXd>& weights)
{
  QuadExprs quad_expr(aff_expr.constants.rows(), aff_expr.linear_coeffs.cols());

  // Per-equation constant term: a_i²
  quad_expr.constants = aff_expr.constants.array().square() * weights.array();

  // Per-equation linear term: 2 a_i b_iᵀ
  quad_expr.linear_coeffs =
      (2.0 * (aff_expr.constants.array() * weights.array())).matrix().asDiagonal() * aff_expr.linear_coeffs;

  quad_expr.quadratic_coeffs.resize(static_cast<std::size_t>(aff_expr.constants.rows()));

  // Objective aggregates
  quad_expr.objective_linear_coeffs =
      quad_expr.linear_coeffs.transpose() * Eigen::VectorXd::Ones(quad_expr.linear_coeffs.rows());

  quad_expr.objective_quadratic_coeffs.resize(aff_expr.linear_coeffs.cols(), aff_expr.linear_coeffs.cols());
  // Start as empty (all zeros)
  // objective_quadratic_coeffs will be the sum of per-equation Q_i
  for (Eigen::Index i = 0; i < aff_expr.constants.rows(); ++i)
  {
    auto eq_affexpr_coeffs = aff_expr.linear_coeffs.row(i);                                                // b_iᵀ
    const trajopt_ifopt::Jacobian eq_quadexpr_coeffs = eq_affexpr_coeffs.transpose() * eq_affexpr_coeffs;  // b_i b_iᵀ

    for (int j = 0; j < eq_quadexpr_coeffs.outerSize(); ++j)  // outerSize() == rows for RowMajor
    {
      for (trajopt_ifopt::Jacobian::InnerIterator it(eq_quadexpr_coeffs, j); it; ++it)
        it.valueRef() *= weights[i];  // scale each nonzero in row i
    }

    if (eq_quadexpr_coeffs.nonZeros() > 0)
    {
      quad_expr.quadratic_coeffs[static_cast<std::size_t>(i)] = eq_quadexpr_coeffs;
      quad_expr.objective_quadratic_coeffs += eq_quadexpr_coeffs;
    }
  }

  return quad_expr;
}
}  // namespace trajopt_sqp
