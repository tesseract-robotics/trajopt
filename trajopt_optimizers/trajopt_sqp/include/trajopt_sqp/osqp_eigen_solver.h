#ifndef TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <ifopt/problem.h>
#include <trajopt_sqp/qp_solver.h>
#include <OsqpEigen/OsqpEigen.h>

namespace trajopt
{
/**
 * @brief Provides an interface to OSQP via the osqp_eigen library
 *
 * For more details see here:
 * https://github.com/robotology/osqp-eigen
 *
 * Note: If we end up using this SQP routine longterm, we should probably take much of the code here and define an SQP
 * problem class that converts a NLP to SQP Problem. Then we can define interfaces to various solvers like this was
 * originally intended to be.
 */
class OSQPEigenSolver : public trajopt::QPSolver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  OSQPEigenSolver() = default;

  /**
   * @brief Initializes the solver. This should be called any time costs, constraints, or variables are added or
   * modified
   * @param nlp The IFOPT problem to be approximated and solved. Must remain in scope until Solve() is called
   * @return true if succeeded
   */
  void init(ifopt::Problem& nlp) override;

  /** @brief Convexifies the problem about the current vars. Sets the hessian, gradient, and constraint matrix
   *
   * TODO: Detailed description
   *
   */
  void convexify() override;

  /** @brief Sets the NLP constraint bounds, linearized about current vars */
  void updateConstraintBounds() override;

  /** @brief Updates variable constraint bounds. Should be called if box size or variable limits change */
  void updateVariableBounds() override;

  /** @brief Linearizes the problem about the current variable values and solves the QP */
  void solve() override;

  /** @brief Convexifies, updates bounds, and solves */
  void convexifyAndSolve() override;

  /** @brief Evaluates the convexified cost at the new vars */
  double evaluateConvexCost(const Eigen::Ref<Eigen::VectorXd>& var_vals) override;

  Eigen::VectorXd getConstraintViolations() override;

  /** @brief Get the results vector after Solve()  */
  Eigen::VectorXd getResults() override { return results_; };

  /** @brief Low level SQP solver. Use solver_.settings() to change settings */
  OsqpEigen::Solver solver_;

  /** @brief Sets the box size. Vector must be num_vars_ long */
  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) override;

  /** @brief Scale box size uniformly by scale
   * @param scale Value by which the box size is scaled*/
  void scaleBoxSize(double scale) override { box_size_ = box_size_ * scale; };

  /**
   * @brief Returns the box size vector
   * @return The box size vector
   */
  Eigen::VectorXd getBoxSize() override { return box_size_; }

private:
  ifopt::Problem* nlp_;

  /** @brief Number of optimization variables in QP */
  Eigen::Index num_vars_;
  /** @brief Number of optimization variables in QP. This includes NLP_cnts, variable limits, and box constraints */
  Eigen::Index num_cnts_;
  Eigen::VectorXd results_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;
  /** @brief Full bounds vector for lower limit- Top part is NLP bounds, bottom part is variable limits/box constraints
   */
  Eigen::VectorXd full_bounds_lower_;
  /** @brief Full bounds vector for upper limit - Top part is NLP bounds, bottom part is variable limits/box constraints
   */
  Eigen::VectorXd full_bounds_upper_;

  Eigen::SparseMatrix<double> hessian_;
  Eigen::VectorXd gradient_;
};

}  // namespace trajopt

#endif
