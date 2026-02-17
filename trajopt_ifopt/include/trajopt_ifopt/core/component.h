/******************************************************************************
Copyright (c) 2017, Alexander W Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TRAJOPT_IFOPT_CORE_DYNAMIC_COMPONENT_H
#define TRAJOPT_IFOPT_CORE_DYNAMIC_COMPONENT_H
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/eigen_types.h>

namespace trajopt_ifopt
{
/**
 * @brief Common interface for problem building blocks.
 *
 * This interface provides the shared API between:
 * @li Variable sets (optimization decision variables)
 * @li Constraint sets (g(x) with bounds)
 * @li Cost terms (scalar objective contributions)
 *
 * The intent is to keep the "smallest common denominator" needed for:
 * @li Naming and sizing (rows)
 * @li Querying values
 * @li Querying bounds
 *
 * More specialized capabilities are expressed via derived interfaces:
 * @li @ref VariablesInterface for components that accept variable values from the solver
 * @li @ref DifferentiableInterface for components that provide Jacobians and may be dynamic-sized
 *
 * @note
 * "Rows" has the same meaning as in the original IFOPT Component design:
 * @li Variable set: number of variables in this set
 * @li Constraint set: number of constraints in this set
 * @li Cost term: typically 1 (scalar cost value)
 *
 * @ingroup ProblemFormulation
 */
class Component
{
public:
  using Ptr = std::shared_ptr<Component>;

  Component(std::string name);
  Component(std::string name, int num_rows);
  virtual ~Component() = default;

  /**
   * @brief Returns the component identifier.
   *
   * Names are used for bookkeeping, debugging, and (optionally) component lookup
   * in composite containers.
   */
  const std::string& getName() const;

  /**
   * @brief Returns the number of rows of this component.
   *
   * Semantics depend on the concrete type:
   * @li Variable set: number of variables
   * @li Constraint set: number of constraint rows
   * @li Cost term: usually 1 (scalar)
   *
   * @note For dynamic-sized differentiable components, the row count is expected
   *       to reflect the most recent @ref DifferentiableInterface::update() call.
   */
  int getRows() const;

  /**
   * @brief Returns the "values" of whatever this component represents.
   *
   * @li Variable set: the current optimization values for this variable block.
   * @li Constraint set: the constraint values g(x).
   * @li Cost term: the cost value (typically a 1x1 vector).
   */
  virtual Eigen::VectorXd getValues() const = 0;

  /**
   * @brief Returns the bounds of this component.
   *
   * @li Variable set: variable bounds [lower, upper] per decision variable.
   * @li Constraint set: constraint bounds [lower, upper] per constraint row.
   * @li Cost term: generally does not have bounds; implementations typically
   *               return an empty vector or +/-infinity bounds if required by
   *               downstream code.
   *
   * @note Callers should treat the returned vector size as equal to @ref getRows().
   */
  virtual std::vector<Bounds> getBounds() const = 0;

  /**
   * @brief Prints the relevant information (name, rows, values) of this component.
   * @param index  Of this specific variables-, constraint- or cost set.
   * @param tolerance  When to flag constraint/bound violation.
   */
  virtual void print(int& index, double tolerance = 0.001) const;

protected:
  std::string name_;
  int rows_{ -1 };
};

/**
 * @brief Interface for optimization variable components.
 *
 * A variables component:
 * @li has values and bounds (via @ref ComponentInterface)
 * @li can be assigned from the solver state vector (via @ref setVariables)
 *
 * Unlike constraints/costs, variables are typically fixed-sized. If you ever
 * need dynamic decision variables, add a dedicated capability interface rather
 * than forcing update/dynamic APIs onto all variables.
 *
 * @ingroup ProblemFormulation
 */
class Variables : public Component
{
public:
  using Ptr = std::shared_ptr<Variables>;

  using Component::Component;
  ~Variables() override = default;

  /**
   * @brief Sets the optimization variables for this variable set.
   *
   * The solver generally provides the full decision vector x, which is then
   * partitioned by a variables composite into per-variable-set segments.
   *
   * @param x Segment of the full decision vector corresponding to this variable set.
   *
   * @note @p x is expected to have size equal to @ref getRows().
   */
  virtual void setVariables(const Eigen::VectorXd& x) = 0;

  /**
   * @brief Get the variable hash leveraged for caching data. It should be based on the variables so when the variables
   * change so does the hash
   */
  virtual std::size_t getHash() const = 0;
};

/**
 * @brief Interface for differentiable components (constraints and costs).
 *
 * A differentiable component:
 * @li has values and bounds (via @ref ComponentInterface)
 * @li provides a sparse Jacobian (via @ref getJacobian)
 * @li may have a row count that changes as variables change (dynamic sizing)
 *
 * Typical concrete types:
 * @li Constraint set (multi-row g(x) with bounds)
 * @li Cost term (scalar objective contribution; 1-row value and 1-row Jacobian)
 *
 * @ingroup ProblemFormulation
 */
class Differentiable : public Component
{
public:
  using Ptr = std::shared_ptr<Differentiable>;

  /**
   * @brief Determines how values/Jacobians are combined.
   *
   * @li kStackRows: constraints semantics (rows are concatenated)
   * @li kSumToScalar: costs semantics (all terms accumulate into a single scalar row)
   */
  enum class Mode
  {
    kStackRows,
    kSumToScalar
  };

  Differentiable(std::string name, Mode mode, bool dynamic);
  Differentiable(std::string name, int num_rows, Mode mode, bool dynamic);
  ~Differentiable() override = default;

  /**
   * @brief Indicates whether this component can change its number of rows.
   *
   * A dynamic-sized component may change its reported row count when the
   * decision variables change (e.g. contact constraints whose active set changes).
   * A fixed-sized component always reports the same number of rows.
   */
  bool isDynamic() const;

  /**
   * @brief Indicates whether this component is scalar or stacked
   * @return True if scalar otherwise stacked
   */
  bool isScalar() const;

  /**
   * @brief Returns an estimate of the Jacobian sparsity (number of structural non-zeros).
   *
   * @details
   * This value is intended as a *capacity hint* for preallocation (e.g., calling
   * `SparseMatrix::reserve()` or reserving triplet storage) to reduce reallocations
   * when assembling the Jacobian.
   *
   * - For fixed-structure components, this should typically be set once and remain
   *   constant across iterations.
   * - For dynamic components, this should be refreshed during @ref update() to reflect
   *   the current active-set / structure (or a conservative upper bound if the exact
   *   count is not known cheaply).
   *
   * @note This is not required to equal `getJacobian().nonZeros()` exactly; it may be
   *       an upper bound. Returning a value that is too small may cause extra
   *       allocations, while a value that is too large only wastes reserved capacity.
   *
   * @return Estimated (or maximum expected) number of non-zero entries in the Jacobian.
   */
  Eigen::Index getNonZeros() const;

  /**
   * @brief Recompute sizing/state for dynamic-sized components.
   *
   * For components where @ref isDynamic() is true, this method is used to:
   * @li update internal state derived from the latest variables
   * @li recompute and return the current number of rows
   *
   * For fixed-sized components (@ref isDynamic() == false), this function should
   * be cheap and typically returns @ref getRows().
   *
   * @return The updated number of rows.
   *
   * @note Callers should assume @ref getRows() reflects the updated value after calling this method.
   */
  virtual int update() = 0;

  /**
   * @brief Return the coefficient vector for this component.
   *    * These coefficients scale this component’s contribution to the optimization
   * problem.
   *    * - For *soft/hinge-style* components (e.g., TrajOpt-style penalties), the
   *   coefficient(s) should be interpreted as weights applied to the *objective*
   *   term(s) (e.g., the cost on the hinge/slack variables), not as a scaling of
   *   the underlying constraint Jacobian/residual.
   * - The returned vector length and ordering are component-defined (e.g., one
   *   weight per term/row/time-step/etc.).
   *    * @return Coefficient vector (component-defined length/order).
   */
  virtual Eigen::VectorXd getCoefficients() const = 0;

  /**
   * @brief Returns derivatives of each row with respect to the decision variables.
   *
   * @li Constraint set: matrix with one row per constraint.
   * @li Cost term: row vector (gradient transpose), typically 1 x n_vars.
   *
   * @return Sparse Jacobian of size getRows() x n_vars.
   */
  virtual Jacobian getJacobian() const = 0;

protected:
  Mode mode_{ Mode::kStackRows };
  bool dynamic_{ false };
  Eigen::Index non_zeros_{ -1 };
};

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_CORE_DYNAMIC_COMPONENT_H
