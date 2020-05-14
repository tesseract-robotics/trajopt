#ifndef TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <ifopt/problem.h>
#include <trajopt_sqp/qp_solver.h>
#include <OsqpEigen/OsqpEigen.h>

namespace trajopt_sqp
{
class OSQPEigenSolver : public QPSolver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts);

  bool clear();

  bool solve();

  Eigen::VectorXd getSolution();

  bool updateHessianMatrix(const Hessian& hessian);

  bool updateGradient(const Eigen::Ref<Eigen::VectorXd>& gradient);

  bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound);

  bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound);

  bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                    const Eigen::Ref<const Eigen::VectorXd>& upperBound);

  bool updateLinearConstraintsMatrix(const Jacobian& linearConstraintsMatrix);

  QP_SOLVER_STATUS getSolverStatus();

  OsqpEigen::Solver solver_;

private:
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  Eigen::Index num_vars_;
  Eigen::Index num_cnts_;
};

}  // namespace trajopt_sqp

#endif
