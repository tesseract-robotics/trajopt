#ifndef TRAJOPT_SQP_INCLUDE_QP_PROBLEM_H_
#define TRAJOPT_SQP_INCLUDE_QP_PROBLEM_H_

#include <memory>
#include <trajopt_sqp/types.h>
#include <ifopt/problem.h>
//#include <trajopt_sqp/osqp_eigen_solver.h>

namespace trajopt_sqp
{
enum class ConstraintType
{
  EQ,
  INEQ
};

class QPProblem
{
public:
  using Ptr = std::shared_ptr<QPProblem>;
  using ConstPtr = std::shared_ptr<const QPProblem>;

  /** @brief Sets up the problem and initializes matrices
   * @param nlp
   */
  void init(ifopt::Problem& nlp);
  /** @brief Run the full convexification routine. If in doubt, init(nlp) then convexify() */
  void convexify();

  /** @brief Updates the cost hessian (Currently does nothing since IFOPT doesn't support Hessians*/
  void updateHessian();
  /** @brief Update the cost gradient including slack variables */
  void updateGradient();

  void linearizeConstraints();

  void updateNLPConstraintBounds();

  void updateNLPVariableBounds();

  void updateSlackVariableBounds();

  void convexifyAndSolve();

  double evaluateTotalConvexCost(const Eigen::Ref<Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<Eigen::VectorXd>& var_vals);

  Eigen::VectorXd getExactConstraintViolations();

  void scaleBoxSize(double& scale);

  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size);

  Eigen::VectorXd getBoxSize() const;

  void print() const;

protected:
  ifopt::Problem* nlp_;

  // TODO: I don't really want people to have write access to these. make const getters
public:
  Eigen::Index num_nlp_vars_;
  Eigen::Index num_nlp_cnts_;
  Eigen::Index num_qp_vars_;
  Eigen::Index num_qp_cnts_;
  std::vector<ConstraintType> constraint_types_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;
  Eigen::VectorXd constraint_merit_coeff_;

  Eigen::SparseMatrix<double> hessian_;
  Eigen::VectorXd gradient_;

  Eigen::SparseMatrix<double> constraint_matrix_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
};

}  // namespace trajopt_sqp

#endif
