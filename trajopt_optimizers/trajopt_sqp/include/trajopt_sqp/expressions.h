#ifndef TRAJOPT_SQP_EXPRESSIONS_H
#define TRAJOPT_SQP_EXPRESSIONS_H

#include <vector>

#include <trajopt_ifopt/core/eigen_types.h>

namespace trajopt_sqp
{
struct QuadExprs;

/**
 * @brief Base class for a collection of scalar expressions evaluated at a common decision vector.
 *
 * The class represents a vector-valued function
 *
 * \f[
 *   f(x) = \begin{bmatrix} f_0(x) \\ f_1(x) \\ \vdots \\ f_{n-1}(x) \end{bmatrix},
 * \f]
 *
 * where each component may be affine, quadratic, or more general, depending on the concrete subclass.
 */
struct Exprs
{
  virtual ~Exprs() = default;

  /**
   * @brief Evaluate all expressions at the given decision vector.
   * @param x Decision vector at which to evaluate the expressions.
   * @return Vector of expression values \f$f(x)\f.
   */
  virtual void values(Eigen::Ref<Eigen::VectorXd> out, const Eigen::Ref<const Eigen::VectorXd>& x) const = 0;
};

/**
 * @brief Vector of affine expressions of the form \f$f(x) = c + A x\f.
 *
 * Each row of @ref linear_coeffs and the corresponding entry in @ref constants define
 * a single scalar affine expression:
 *
 * \f[
 *   f_i(x) = c_i + a_i^\top x,
 * \f]
 *
 * where \f$c_i\f$ is @ref constants(i) and \f$a_i^\top\f$ is @ref linear_coeffs.row(i).
 */
struct AffExprs : Exprs
{
  /**
   * @brief Constant term \f$c\f$ for each affine expression.
   *
   * Size: number of expressions.
   */
  Eigen::VectorXd constants;

  /**
   * @brief Linear coefficient matrix \f$A\f$ for the affine expressions.
   *
   * Each row corresponds to a single expression, and each column corresponds to a
   * decision variable. Size: (num_exprs × num_vars).
   */
  trajopt_ifopt::Jacobian linear_coeffs;

  /**
   * @brief Evaluate the affine expressions at the given decision vector.
   *
   * Computes
   * \f[
   *   f(x) = c + A x.
   * \f]
   *
   * @param x Decision vector.
   * @return Vector of affine expression values \f$f(x)\f.
   */
  void values(Eigen::Ref<Eigen::VectorXd> out, const Eigen::Ref<const Eigen::VectorXd>& x) const override final;

  /**
   * @brief Build a local affine (first-order) approximation of a vector-valued function.
   *
   * This constructs an affine model
   * \f[
   *   \hat{f}(x) = a + B x
   * \f]
   * around a fixed expansion point \f$x_0\f (here given by @p x), such that:
   * - \f$\hat{f}(x_0) = f(x_0)\f$  (matches the function value at @p x)
   * - \f$\nabla \hat{f}(x_0) = \nabla f(x_0)\f$  (matches the Jacobian at @p x)
   * Given:
   * - @p func_error    \f$\equiv f(x_0)\f$
   * - @p func_jacobian \f$\equiv \nabla f(x_0)\f$
   * the affine approximation can be written as
   * \f[
   *   \hat{f}(x) = f(x_0) + \nabla f(x_0)\,(x - x_0)
   *              = \underbrace{\big(f(x_0) - \nabla f(x_0)\,x_0\big)}_{a}
   *                + \underbrace{\nabla f(x_0)}_{B}\,x.
   * \f]
   * This function returns an @ref AffExprs where:
   * - `constants = func_error - func_jacobian * x`  (the vector @f$a@f$)
   * - `linear_coeffs = func_jacobian`               (the matrix @f$B@f$)
   * @details
   * The derivation follows the standard local linearization (tangent plane) of a multivariable function.
   * For a good conceptual reference, see:
   *    *
   * https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/tangent-planes-and-local-linearization/a/local-linearization
   *
   * @param func_error     Function value @f$f(x_0)@f$ at the linearization point.
   * @param func_jacobian  Jacobian @f$\nabla f(x_0)@f$ at the linearization point.
   * @param x              Linearization point @f$x_0@f$ used to compute @p func_error and @p func_jacobian.
   */
  void create(const Eigen::Ref<const Eigen::VectorXd>& func_error,
              const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
              const Eigen::Ref<const Eigen::VectorXd>& x);

  /**
   * @brief Construct a quadratic model for the weighted element-wise square of the affine
   * expressions.
   *
   * Given affine expressions \f$f_i(x) = a_i + b_i^\top x\f$ and per-expression weights
   * \f$w_i \ge 0\f$, this builds
   *
   * \f[
   *   g_i(x) = w_i f_i(x)^2
   *          = w_i a_i^2 + 2 a_i w_i b_i^\top x + x^\top (w_i b_i b_i^\top) x.
   * \f]
   *
   * In @p quad_expr this corresponds to:
   * - `constants(i)`              = \f$w_i a_i^2\f$
   * - `linear_coeffs.row(i)`      = \f$2 a_i w_i b_i^\top\f$
   * - `quadratic_coeffs[i]`       = \f$q_i^\top = \sqrt{w_i}\, b_i^\top\f$, a 1×n row
   *
   * The quadratic term is stored in factored form,
   * \f$x^\top (w_i b_i b_i^\top) x = (q_i^\top x)^2\f$, rather than as the rank-one matrix. An
   * expression whose row of @ref linear_coeffs is empty gets an empty (0×0) entry. See
   * @ref QuadExprs::quadratic_coeffs.
   *
   * The aggregate objective \f$J(x) = \sum_i g_i(x)\f$ is accumulated into:
   * - `objective_linear_coeffs`    = \f$\sum_i 2 a_i w_i b_i\f$
   * - `objective_quadratic_coeffs` = \f$\sum_i w_i b_i b_i^\top\f$
   *
   * Note that `objective_quadratic_coeffs` sums the rank-one matrices that the
   * `quadratic_coeffs` rows factor; it is not the sum of those rows.
   *
   * This is useful when converting a weighted sum-of-squares objective over affine residuals into
   * a single quadratic form suitable for QP solvers.
   *
   * @param quad_expr Output quadratic model of the weighted element-wise square.
   * @param weights Weights \f$w_i\f$, one per expression. Must be finite and non-negative.
   */
  void square(QuadExprs& quad_expr, const Eigen::Ref<const Eigen::VectorXd>& weights) const;

private:
  // Reusable sparse buffer for Bw = diag(sqrt(w)) * B
  mutable trajopt_ifopt::Jacobian scratch_bw_;
};

/**
 * @brief Vector of quadratic expressions and their aggregated objective contribution.
 *
 * Each scalar expression has the form
 *
 * \f[
 *   f_i(x) = c_i + a_i^\top x + x^\top Q_i x,
 * \f]
 *
 * where:
 *  - @ref constants(i) stores \f$c_i\f$
 *  - @ref linear_coeffs.row(i) stores \f$a_i^\top\f$
 *  - @ref quadratic_coeffs[i] stores the quadratic term, in one of the two forms described on
 *    that member.
 *
 * In addition, @ref objective_linear_coeffs and @ref objective_quadratic_coeffs may be
 * used to accumulate the sum of all expressions into a single quadratic objective.
 */
struct QuadExprs : Exprs
{
  QuadExprs() = default;

  /**
   * @brief Construct a quadratic expression container with given dimensions.
   *
   * Allocates and sizes the primary storage for a problem with @p num_cost scalar
   * expressions and @p num_vars decision variables:
   *
   *  - @ref constants has size @p num_cost
   *  - @ref linear_coeffs has size @p num_cost × @p num_vars
   *  - @ref objective_linear_coeffs has size @p num_vars
   *  - @ref objective_quadratic_coeffs has size @p num_vars × @p num_vars
   *  - @ref quadratic_coeffs is reserved to hold @p num_cost sparse matrices.
   *
   * @param num_cost Number of scalar expressions.
   * @param num_vars Number of decision variables.
   */
  QuadExprs(Eigen::Index num_cost, Eigen::Index num_vars);

  /**
   * @brief Constant term \f$c_i\f for each quadratic expression.
   * @details Entry @c constants(i) is the constant for expression \f$f_i(x)\f.
   */
  Eigen::VectorXd constants;

  /**
   * @brief Linear coefficient matrix \f$a_i^\top\f for each expression.
   * @details Row @c linear_coeffs.row(i) contains the linear coefficients
   *          associated with expression \f$f_i(x)\f.
   *
   * Dimensions: (num_expressions × num_vars).
   */
  trajopt_ifopt::Jacobian linear_coeffs;

  /**
   * @brief Quadratic coefficients for each expression, in one of two forms.
   * @details Entry @c quadratic_coeffs[i] carries the quadratic term of expression
   *          \f$f_i(x)\f$ in one of two representations, distinguished by its row count:
   *          - An n×n matrix \f$Q_i\f$ contributing \f$x^\top Q_i x\f$, produced by @ref create.
   *          - A 1×n row \f$q_i^\top\f$ contributing \f$(q_i^\top x)^2\f$, the factored form
   *            produced by @ref AffExprs::square.
   *
   *          An empty (0×0) entry means the expression is purely affine.
   *
   * @warning The two forms are distinguished by @c rows(), so they are ambiguous when there is
   *          exactly one decision variable: a genuine 1×1 \f$Q_i\f$ cannot be told apart from a
   *          factored 1×1 \f$q_i^\top\f$ and is decoded as the latter.
   */
  std::vector<trajopt_ifopt::Jacobian> quadratic_coeffs;

  /**
   * @brief Aggregated objective linear coefficients.
   * @details This typically holds the sum of the per-expression linear
   *          contributions, e.g. when forming a single objective
   *
   * \f[
   *   F(x) = \sum_i f_i(x) = C + g^\top x + x^\top H x,
   * \f]
   *
   * where @ref objective_linear_coeffs stores \f$g\f.
   */
  Eigen::VectorXd objective_linear_coeffs;

  /**
   * @brief Aggregated objective quadratic coefficients.
   * @details This typically holds the sum of the per-expression quadratic
   *          contributions \f$Q_i\f when forming a single objective, i.e.
   *          @ref objective_quadratic_coeffs stores \f$H\f in
   *
   * \f[
   *   F(x) = \sum_i f_i(x) = C + g^\top x + x^\top H x.
   * \f]
   *
   * Dimensions: (num_vars × num_vars).
   */
  trajopt_ifopt::Jacobian objective_quadratic_coeffs;

  /**
   * @brief Evaluate all quadratic expressions at the given decision vector.
   *
   * Computes, for each \f$i\f,
   *
   * \f[
   *   f_i(x) = c_i + a_i^\top x + x^\top Q_i x.
   * \f]
   *
   * @param x Decision vector.
   * @return Vector of expression values \f$f(x)\f.
   */
  void values(Eigen::Ref<Eigen::VectorXd> out, const Eigen::Ref<const Eigen::VectorXd>& x) const override final;

  /**
   * @brief Build a local quadratic (second-order) approximation of a vector-valued function.
   *    * This constructs, for each scalar component \f$f_i\f$ of a vector-valued function
   * \f$f : \mathbb{R}^n \to \mathbb{R}^m\f$, a quadratic model around the expansion
   * point \f$x_0\f (given by @p x):
   *    * \f[
   *   \hat{f}_i(x) =
   *     a_i
   *   + b_i^\top x
   *   + x^\top C_i x,
   * \f]
   *    * such that the model matches the function value, gradient, and Hessian at \f$x_0\f:
   *    * - \f$\hat{f}_i(x_0)    = f_i(x_0)\f$
   * - \f$\nabla \hat{f}_i(x_0) = \nabla f_i(x_0)\f$
   * - \f$\nabla^2 \hat{f}_i(x_0) = H_i\f$
   *    * where:
   *    * - @p func_errors   \f$\equiv f(x_0)\f \in \mathbb{R}^m\f$
   * - @p func_jacobian \f$\equiv \nabla f(x_0)\f \in \mathbb{R}^{m \times n}\f$
   * - @p func_hessians \f$\equiv \{H_i\}_{i=1}^m\f$, one Hessian per component
   * - @p x             \f$\equiv x_0\f$
   *    * Following the standard second-order Taylor expansion around \f$x_0\f$:
   *    * \f[
   *   f_i(x) \approx f_i(x_0)
   *            + \nabla f_i(x_0)^\top (x - x_0)
   *            + \tfrac{1}{2}(x - x_0)^\top H_i (x - x_0),
   * \f]
   *    * we collect terms in powers of \f$x\f$ and define:
   *    * \f[
   *   C_i = \tfrac{1}{2} H_i,
   * \f]
   * \f[
   *   a_i = f_i(x_0) - \nabla f_i(x_0)^\top x_0 + x_0^\top C_i x_0,
   * \f]
   * \f[
   *   b_i^\top = \nabla f_i(x_0)^\top - (2 C_i x_0)^\top.
   * \f]
   *    * In the returned @ref QuadExprs this corresponds to:
   *    * - `constants(i)            = a_i`
   * - `linear_coeffs.row(i)    = b_i^T`
   * - `quadratic_coeffs[i]     = C_i`
   *    * @details
   * The derivation follows the standard quadratic approximation of multivariable functions.
   * For a conceptual overview, see:
   *    *
   * https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/quadratic-approximations/a/quadratic-approximation
   *    * Note that this implementation differs from `CostFromFunc::convex` in the original TrajOpt code,
   * but the underlying Taylor expansion idea is the same.
   *
   * @param func_errors     Function values @f$f(x_0)@f$ at the expansion point.
   * @param func_jacobian   Jacobian @f$\nabla f(x_0)@f$ at the expansion point.
   * @param func_hessians   Per-component Hessians @f$\{H_i\}@f$ at the expansion point; size must equal @p
   * func_errors.rows().
   * @param x               Expansion point @f$x_0@f$ used to compute @p func_errors, @p func_jacobian, and @p
   * func_hessians.
   */
  void create(const Eigen::Ref<const Eigen::VectorXd>& func_errors,
              const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
              const std::vector<trajopt_ifopt::Jacobian>& func_hessians,
              const Eigen::Ref<const Eigen::VectorXd>& x);

private:
  mutable Eigen::VectorXd scratch_;
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_EXPRESSIONS_H
