#ifndef TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_

#include <ifopt/problem.h>

namespace trajopt
{
/**
 * @brief Base class for Quadratic programming solvers
 */
class QPSolver
{
public:
  using Ptr = std::shared_ptr<QPSolver>;
  using ConstPtr = std::shared_ptr<const QPSolver>;

  virtual void init(ifopt::Problem& nlp) = 0;

  virtual void convexify() = 0;

  virtual void updateConstraintBounds() = 0;

  virtual void updateVariableBounds() = 0;

  virtual void solve() = 0;

  virtual void convexifyAndSolve() = 0;

  virtual double evaluateConvexCost(const Eigen::Ref<Eigen::VectorXd>& var_vals) = 0;

  virtual Eigen::VectorXd getConstraintViolations() = 0;

  virtual Eigen::VectorXd getResults() = 0;

  virtual void scaleBoxSize(double) = 0;

  virtual void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) = 0;

  virtual Eigen::VectorXd getBoxSize() = 0;
};

}  // namespace trajopt

#endif
