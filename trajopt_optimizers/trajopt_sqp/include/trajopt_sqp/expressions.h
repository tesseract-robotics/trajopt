#ifndef TRAJOPT_SQP_EXPRESSIONS_H
#define TRAJOPT_SQP_EXPRESSIONS_H

#include <vector>

#include <trajopt_ifopt/core/eigen_types.h>

namespace trajopt_sqp
{
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
  virtual Eigen::VectorXd values(const Eigen::Ref<const Eigen::VectorXd>& x) const = 0;
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
  Eigen::VectorXd values(const Eigen::Ref<const Eigen::VectorXd>& x) const override final;
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
 *  - @ref constants(i) stores \f$c_i\f
 *  - @ref linear_coeffs.row(i) stores \f$a_i^\top\f
 *  - @ref quadratic_coeffs[i] stores the sparse matrix \f$Q_i\f.
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
   * @brief Quadratic coefficient matrices \f$Q_i\f for each expression.
   * @details Entry @c quadratic_coeffs[i] is the sparse matrix \f$Q_i\f for
   *          expression \f$f_i(x)\f. If a given expression is purely affine,
   *          the corresponding matrix may be empty (zero non-zeros).
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
  Eigen::VectorXd values(const Eigen::Ref<const Eigen::VectorXd>& x) const override final;
};

/**
 * @brief Build a local affine (first-order) approximation of a vector-valued function.
 *
 * This constructs an affine model
 *
 * \f[
 *   \hat{f}(x) = a + B x
 * \f]
 *
 * around a fixed expansion point \f$x_0\f (here given by @p x), such that:
 *
 * - \f$\hat{f}(x_0) = f(x_0)\f$  (matches the function value at @p x)
 * - \f$\nabla \hat{f}(x_0) = \nabla f(x_0)\f$  (matches the Jacobian at @p x)
 *
 * Given:
 *
 * - @p func_error    \f$\equiv f(x_0)\f$
 * - @p func_jacobian \f$\equiv \nabla f(x_0)\f$
 *
 * the affine approximation can be written as
 *
 * \f[
 *   \hat{f}(x) = f(x_0) + \nabla f(x_0)\,(x - x_0)
 *              = \underbrace{\big(f(x_0) - \nabla f(x_0)\,x_0\big)}_{a}
 *                + \underbrace{\nabla f(x_0)}_{B}\,x.
 * \f]
 *
 * This function returns an @ref AffExprs where:
 *
 * - `constants = func_error - func_jacobian * x`  (the vector @f$a@f$)
 * - `linear_coeffs = func_jacobian`               (the matrix @f$B@f$)
 *
 * @details
 * The derivation follows the standard local linearization (tangent plane) of a multivariable function.
 * For a good conceptual reference, see:
 *
 *   https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/tangent-planes-and-local-linearization/a/local-linearization
 *
 * @param func_error     Function value @f$f(x_0)@f$ at the linearization point.
 * @param func_jacobian  Jacobian @f$\nabla f(x_0)@f$ at the linearization point.
 * @param x              Linearization point @f$x_0@f$ used to compute @p func_error and @p func_jacobian.
 * @return Affine expressions representing the local linear approximation @f$\hat{f}(x)@f$.
 */
AffExprs createAffExprs(const Eigen::Ref<const Eigen::VectorXd>& func_error,
                        const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
                        const Eigen::Ref<const Eigen::VectorXd>& x);

/**
 * @brief Build a local quadratic (second-order) approximation of a vector-valued function.
 *
 * This constructs, for each scalar component \f$f_i\f$ of a vector-valued function
 * \f$f : \mathbb{R}^n \to \mathbb{R}^m\f$, a quadratic model around the expansion
 * point \f$x_0\f (given by @p x):
 *
 * \f[
 *   \hat{f}_i(x) =
 *     a_i
 *   + b_i^\top x
 *   + x^\top C_i x,
 * \f]
 *
 * such that the model matches the function value, gradient, and Hessian at \f$x_0\f:
 *
 * - \f$\hat{f}_i(x_0)    = f_i(x_0)\f$
 * - \f$\nabla \hat{f}_i(x_0) = \nabla f_i(x_0)\f$
 * - \f$\nabla^2 \hat{f}_i(x_0) = H_i\f$
 *
 * where:
 *
 * - @p func_errors   \f$\equiv f(x_0)\f \in \mathbb{R}^m\f$
 * - @p func_jacobian \f$\equiv \nabla f(x_0)\f \in \mathbb{R}^{m \times n}\f$
 * - @p func_hessians \f$\equiv \{H_i\}_{i=1}^m\f$, one Hessian per component
 * - @p x             \f$\equiv x_0\f$
 *
 * Following the standard second-order Taylor expansion around \f$x_0\f\f$:
 *
 * \f[
 *   f_i(x) \approx f_i(x_0)
 *            + \nabla f_i(x_0)^\top (x - x_0)
 *            + \tfrac{1}{2}(x - x_0)^\top H_i (x - x_0),
 * \f]
 *
 * we collect terms in powers of \f$x\f$ and define:
 *
 * \f[
 *   C_i = \tfrac{1}{2} H_i,
 * \f]
 * \f[
 *   a_i = f_i(x_0) - \nabla f_i(x_0)^\top x_0 + x_0^\top C_i x_0,
 * \f]
 * \f[
 *   b_i^\top = \nabla f_i(x_0)^\top - (2 C_i x_0)^\top.
 * \f]
 *
 * In the returned @ref QuadExprs this corresponds to:
 *
 * - `constants(i)            = a_i`
 * - `linear_coeffs.row(i)    = b_i^T`
 * - `quadratic_coeffs[i]     = C_i`
 *
 * @details
 * The derivation follows the standard quadratic approximation of multivariable functions.
 * For a conceptual overview, see:
 *
 *   https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/quadratic-approximations/a/quadratic-approximation
 *
 * Note that this implementation differs from `CostFromFunc::convex` in the original TrajOpt code,
 * but the underlying Taylor expansion idea is the same.
 *
 * @param func_errors     Function values @f$f(x_0)@f$ at the expansion point.
 * @param func_jacobian   Jacobian @f$\nabla f(x_0)@f$ at the expansion point.
 * @param func_hessians   Per-component Hessians @f$\{H_i\}@f$ at the expansion point; size must equal @p
 * func_errors.rows().
 * @param x               Expansion point @f$x_0@f$ used to compute @p func_errors, @p func_jacobian, and @p
 * func_hessians.
 * @return Quadratic expressions representing the local second-order approximation \f$\hat{f}(x)\f$.
 */
QuadExprs createQuadExprs(const Eigen::Ref<const Eigen::VectorXd>& func_errors,
                          const Eigen::Ref<const trajopt_ifopt::Jacobian>& func_jacobian,
                          const std::vector<trajopt_ifopt::Jacobian>& func_hessians,
                          const Eigen::Ref<const Eigen::VectorXd>& x);

/**
 * @brief Construct a quadratic representation for the element-wise square of affine expressions.
 *
 * Given an affine expression
 *
 *   f_i(x) = a_i + b_i^T x,
 *
 * for each row i in @p aff_expr, this function builds a quadratic model
 *
 *   g_i(x) = f_i(x)^2
 *          = (a_i + b_i^T x)^2
 *          = a_i^2 + 2 a_i b_i^T x + x^T (b_i b_i^T) x.
 *
 * In the returned @ref QuadExprs this corresponds to:
 *
 * - `constants(i)`              = a_i²
 * - `linear_coeffs.row(i)`      = 2 a_i b_i^T
 * - `quadratic_coeffs[i]`       = b_i b_i^T
 *
 * Additionally, this function accumulates the per-equation contributions into an
 * aggregate quadratic objective of the form
 *
 *   J(x) = sum_i g_i(x) = sum_i f_i(x)^2,
 *
 * by setting:
 *
 * - `objective_linear_coeffs`   = ∑_i (linear_coeffs.row(i))^T
 * - `objective_quadratic_coeffs`= ∑_i quadratic_coeffs[i]
 *
 * This is useful when you want to convert a sum-of-squares objective based on
 * affine residuals into a single quadratic form suitable for QP solvers.
 *
 * @param aff_expr Affine expressions f(x) = a + Bx to be squared element-wise.
 * @return A @ref QuadExprs encoding g(x) = f(x) ∘ f(x) (element-wise square),
 *         along with the aggregated quadratic objective coefficients.
 */
QuadExprs squareAffExprs(const AffExprs& aff_expr, const Eigen::Ref<const Eigen::VectorXd>& weights);

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_EXPRESSIONS_H
