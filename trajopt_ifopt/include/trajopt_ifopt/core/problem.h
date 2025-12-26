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

#ifndef TRAJOPT_IFOPT_CORE_PROBLEM_H
#define TRAJOPT_IFOPT_CORE_PROBLEM_H

#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/core/cost_term.h>
#include <trajopt_ifopt/core/component.h>

/**
 * @brief common namespace for all elements in this library.
 */
namespace trajopt_ifopt
{
/**
 * @defgroup ProblemFormulation
 * @brief The elements to formulate the solver independent optimization problem.
 *
 * An optimization problem usually consists of multiple sets of independent
 * variable- or constraint-sets. Each set represents a common concept, e.g. one
 * set of variables represents spline coefficients, another footstep positions.
 * Similarly, each constraint-set groups a set of similar constraints.
 *
 * The Nonlinear Optimization Problem to solve is defined as:
 *
 *     find x0, x1                              (variable-sets 0 & 1)
 *     s.t
 *       x0_lower  <= x0 <= x0_upper            (bounds on variable-set x0 \in R^2)
 *
 *       {x0,x1} = arg min c0(x0,x1)+c1(x0,x1)  (cost-terms 0 and 1)
 *
 *       g0_lower < g0(x0,x1) < g0_upper        (constraint-set 0 \in R^2)
 *       g1_lower < g1(x0,x1) < g0_upper        (constraint-set 1 \in R^1)
 *
 *
 * #### getValues()
 * This structure allows a user to define each of these sets independently in
 * separate classes and ifopt takes care of building the overall problem from
 * these sets. This is implemented by
 *  * stacking all variable-sets to build the overall variable vector
 *  * summing all cost-terms to calculate the total cost
 *  * stacking all constraint-sets to build the overall constraint vector.
 *
 * #### GetJacobian()
 * Supplying derivative information greatly increases solution speed. ifopt
 * allows to define the derivative of each cost-term/constraint-set with
 * respect to each variable-set independently. This ensures that when the
 * order of variable-sets changes in the overall vector, this derivative
 * information is still valid. These "Jacobian blocks" must be supplied through
 * ConstraintSet::fillJacobianBlock() and are then used to build the complete Jacobian for
 * the cost and constraints.
 *
 * \image html ifopt.png
 */

/**
 * @brief A generic optimization problem with variables, costs and constraints.
 *
 * This class is responsible for holding all the information of an optimization
 * problem, which includes the optimization variables, their variable bounds,
 * the cost function, the constraints and their bounds and derivatives of
 * all. With this information the problem can be solved by any specific solver.
 * All the quantities (variables, cost, constraint) are represented
 * by the same generic Component class.
 *
 * @ingroup ProblemFormulation
 * See @ref Solvers for currently implemented solvers.
 */
class Problem
{
public:
  /**
   * @brief  Creates a optimization problem with no variables, costs or constraints.
   */
  Problem();
  virtual ~Problem() = default;

  /**
   * @brief Add one individual set of variables to the optimization problem.
   * @param variable_set  The selection of optimization variables.
   *
   * This function can be called multiple times, with multiple sets, e.g.
   * one set that parameterizes a body trajectory, the other that resembles
   * the optimal timing values. This function correctly appends the
   * individual variables sets and ensures correct order of Jacobian columns.
   */
  void addVariableSet(Variables::Ptr variable_set);

  /**
   * @brief Add a set of multiple constraints to the optimization problem.
   * @param constraint_set  This can be 1 to infinity number of constraints.
   *
   * This function can be called multiple times for different sets of
   * constraints. It makes sure the overall constraint and Jacobian correctly
   * considers all individual constraint sets.
   */
  void addConstraintSet(ConstraintSet::Ptr constraint_set);

  /**
   * @brief Add a cost term to the optimization problem.
   * @param cost_set  The calculation of the cost from the variables.
   *
   * This function can be called multiple times if the cost function is
   * composed of different cost terms. It makes sure the overall value and
   * gradient is considering each individual cost.
   */
  void addCostSet(CostTerm::Ptr cost_set);

  /**
   * @brief  Updates the variables with the values of the raw pointer @c x.
   */
  void setVariables(const double* x);

  /**
   * @brief The number of optimization variables.
   */
  int getNumberOfOptimizationVariables() const;

  /**
   * @brief True if the optimization problem includes a cost, false if
   * merely a feasibility problem is defined.
   */
  bool hasCostTerms() const;

  /**
   * @brief The maximum and minimum value each optimization variable
   * is allowed to have.
   */
  std::vector<Bounds> getBoundsOnOptimizationVariables() const;

  /**
   * @brief The current value of the optimization variables.
   */
  Eigen::VectorXd getVariableValues() const;

  /**
   * @brief The scalar cost for current optimization variables @c x.
   */
  double evaluateCostFunction(const double* x);

  /**
   * @brief The column-vector of derivatives of the cost w.r.t. each variable.
   * @details ipopt uses 10e-8 for their derivative check, but setting here to more precise
   * https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_derivative_test_perturbation
   */
  Eigen::VectorXd evaluateCostFunctionGradient(const double* x,
                                               bool use_finite_difference_approximation = false,
                                               double epsilon = std::numeric_limits<double>::epsilon());

  /**
   * @brief The number of individual constraints.
   */
  int getNumberOfConstraints() const;

  /**
   * @brief The upper and lower bound of each individual constraint.
   */
  std::vector<Bounds> getBoundsOnConstraints() const;

  /**
   * @brief Each constraint value g(x) for current optimization variables @c x.
   */
  Eigen::VectorXd evaluateConstraints(const double* x);

  /**
   * @brief Extracts those entries from constraint Jacobian that are not zero.
   * @param [in]  x  The current values of the optimization variables.
   * @param [out] values  The nonzero derivatives ordered by Eigen::RowMajor.
   */
  void evalNonzerosOfJacobian(const double* x, double* values);

  /**
   * @brief The sparse-matrix representation of Jacobian of the constraints.
   *
   * Each row corresponds to a constraint and each column to an optimizaton
   * variable.
   */
  Jacobian getJacobianOfConstraints() const;

  /**
   * @brief The sparse-matrix representation of Jacobian of the costs.
   *
   * Returns one row corresponding to the costs and each column corresponding
   * to an optimizaton variable.
   */
  Jacobian getJacobianOfCosts() const;

  /**
   * @brief Saves the current values of the optimization variables in x_prev.
   *
   * This is used to keep a history of the values for each NLP iterations.
   */
  void saveCurrent();

  /**
   * @brief Read/write access to the current optimization variables.
   */
  CompositeVariables::Ptr getOptVariables() const;

  /**
   * @brief Sets the optimization variables to those at iteration iter.
   */
  void setOptVariables(int iter);

  /**
   * @brief Sets the optimization variables to those of the final iteration.
   */
  void setOptVariablesFinal();

  /**
   * @brief The number of iterations it took to solve the problem.
   */
  int getIterationCount() const { return static_cast<int>(x_prev.size()); };

  /**
   * @brief Prints the variables, costs and constraints.
   */
  void printCurrent() const;

  /**
   * @brief Read access to the constraints composite
   * @return A const reference to constraints_
   */
  const CompositeDifferentiable& getConstraints() const { return constraints_; };

  /**
   * @brief Read access to the costs composite
   * @return A const reference to costs_
   */
  const CompositeDifferentiable& getCosts() const { return costs_; };

  /**
   * @brief Read access to the history of iterations
   * @return A const reference to x_prev
   */
  const std::vector<Eigen::VectorXd>& getIterations() const { return x_prev; };

private:
  CompositeVariables::Ptr variables_;
  CompositeDifferentiable constraints_;
  CompositeDifferentiable costs_;

  std::vector<Eigen::VectorXd> x_prev;  ///< the pure variables for every iteration.

  Eigen::VectorXd convertToEigen(const double* x) const;
};

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_CORE_PROBLEM_H
