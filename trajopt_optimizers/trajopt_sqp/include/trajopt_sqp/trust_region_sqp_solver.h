#ifndef TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/solver.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/qp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/sqp_callback.h>

namespace trajopt_sqp
{
/**
 * @brief A simple SQP Solver that uses the QPSolver passed in
 */
class TrustRegionSQPSolver : public ifopt::Solver
{
public:
  using Ptr = std::shared_ptr<TrustRegionSQPSolver>;
  using ConstPtr = std::shared_ptr<const TrustRegionSQPSolver>;

  TrustRegionSQPSolver(OSQPEigenSolver::Ptr qp_solver);

  void Solve(ifopt::Problem& nlp) override;

  /**
   * @brief Take a single QP optimization step, storing the results and calling callbacks
   * @param nlp
   */
  bool stepOptimization(ifopt::Problem& nlp);

  bool callCallbacks();

  void printStepInfo() const;

  SQPStatus status_;
  SQPResults results_;
  SQPParameters params_;

  void registerCallback(const SQPCallback::Ptr& callback) { callbacks_.push_back(callback); };
  std::vector<SQPCallback::Ptr> callbacks_;

  OSQPEigenSolver::Ptr qp_solver_;
  QPProblem::Ptr qp_problem_;

private:
  ifopt::Problem* nlp_;
};

}  // namespace trajopt_sqp
#endif
