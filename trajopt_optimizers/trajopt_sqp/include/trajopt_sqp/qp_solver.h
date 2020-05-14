#ifndef TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_

#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
enum class QP_SOLVER_STATUS
{
  UNITIALIZED,
  INITIALIZED,
  ERROR
};

/**
 * @brief Base class for Quadratic programming solvers
 */
class QPSolver
{
public:
  using Ptr = std::shared_ptr<QPSolver>;
  using ConstPtr = std::shared_ptr<const QPSolver>;

  //  virtual bool init() = 0;

  //  virtual bool solve() = 0;

  //  virtual Eigen::VectorXd getSolution() = 0;

  //  virtual bool updateHessianMatrix(const Hessian hessianMatrix) = 0;

  //  virtual bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>> &gradient) = 0;

  //  virtual bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>> &lowerBound) = 0;

  //  virtual bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>> &upperBound) = 0;

  //  virtual bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>> &lowerBound,
  //                            const Eigen::Ref<const Eigen::VectorXd>> &upperBound) = 0;

  //  virtual bool updateLinearConstraintsMatrix(const Jacobian& linearConstraintsMatrix) = 0;

  //  virtual QP_SOLVER_STATUS getSolverStatus() = 0;
};
}  // namespace trajopt_sqp

#endif
