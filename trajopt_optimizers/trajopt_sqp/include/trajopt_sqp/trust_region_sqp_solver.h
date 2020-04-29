#ifndef TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

namespace trajopt
{
/**
 * @brief This struct defines parameters for the SQP optimization. The optimization should not change this struct
 */
struct SQPParameters
{
  double improve_ratio_threshold = 0.25;
  /** @brief NLP converges if trust region is smaller than this */
  double min_trust_box_size = 1e-4;
  /** @brief NLP converges if approx_merit_improves is smaller than this */
  double min_approx_improve = 1e-4;
  /** @brief NLP converges if approx_merit_improve / best_exact_merit < min_approx_improve_frac */
  double min_approx_improve_frac = static_cast<double>(-INFINITY);
  /** @brief Max number of QP calls allowed */
  int max_iterations = 50;

  /** @brief Trust region is scaled by this when it is shrunk */
  double trust_shrink_ratio = 0.1;
  /** @brief Trust region is expanded by this when it is expanded */
  double trust_expand_ratio = 1.5;
  /** @brief Max number of times the trust region will be expanded */
  int max_trust_region_expansions = 10;

  /** @brief Any constraint under this value is not considered a violation */
  double cnt_tolerance = 1e-4;
  /** @brief Max number of times the constraints will be inflated */
  double max_merit_coeff_increases = 5;
  /** @brief Constraints are scaled by this amount when inflated */
  double merit_coeff_increase_ratio = 10;
  /** @brief Unused */
  double max_time = static_cast<double>(INFINITY);
  /** @brief If true, only the constraints that are violated will be inflated */
  bool inflate_constraints_individually = true;
  /** @brief Initial size of the trust region */
  double initial_trust_box_size = 1e-1;
  /** @brief Unused */
  double log_results = false;
  /** @brief Unused */
  std::string log_dir = "/tmp";
};

/** @brief This struct contains information and results for the SQP problem */
struct SQPResults
{
  SQPResults(Eigen::Index num_vars, Eigen::Index num_cnts)
  {
    constraint_violations = Eigen::VectorXd::Zero(num_cnts);
    best_var_vals = Eigen::VectorXd::Zero(num_vars);
    new_var_vals = Eigen::VectorXd::Zero(num_vars);
    box_size = Eigen::VectorXd::Ones(num_vars);
    merit_error_coeffs = Eigen::VectorXd::Ones(num_cnts);
  }
  SQPResults() = default;
  /** @brief The lowest cost ever achieved */
  double best_exact_merit{ std::numeric_limits<double>::max() };
  /** @brief The cost achieved this iteration */
  double new_exact_merit{ std::numeric_limits<double>::max() };
  /** @brief The lowest convexified cost ever achieved */
  double best_approx_merit{ std::numeric_limits<double>::max() };
  /** @brief The convexified cost achieved this iteration */
  double new_approx_merit{ std::numeric_limits<double>::max() };

  /** @brief Variable values associated with best_exact_merit */
  Eigen::VectorXd best_var_vals;
  /** @brief Variable values associated with this iteration */
  Eigen::VectorXd new_var_vals;

  /** @brief Amount the convexified cost improved over the best this iteration */
  double approx_merit_improve;
  /** @brief Amount the exact cost improved over the best this iteration */
  double exact_merit_improve;
  /** @brief The amount the cost improved as a ratio of the total cost */
  double merit_improve_ratio;

  /** @brief Vector defing the box size. The box is var_vals +/- box_size */
  Eigen::VectorXd box_size;
  /** @brief Coefficients used to weight the constraint violations */
  Eigen::VectorXd merit_error_coeffs;
  /** @brief Vector of the worst constraint violations. Positive is a violation */
  Eigen::VectorXd constraint_violations;
};

enum class SQPStatus
{
  RUNNING,
  TRUST_REGION_CONVERGED,
  NLP_CONVERGED,
  ITERATION_LIMIT,
  ERROR
};

/**
 * @brief A simple SQP Solver that uses the QPSolver passed in
 */
class TrustRegionSQPSolver : public ifopt::Solver
{
public:
  using Ptr = std::shared_ptr<TrustRegionSQPSolver>;
  using ConstPtr = std::shared_ptr<const TrustRegionSQPSolver>;

  TrustRegionSQPSolver(trajopt::OSQPEigenSolver::Ptr qp_solver);

  void Solve(ifopt::Problem& nlp) override;

  void stepOptimization(ifopt::Problem& nlp);

  /** @brief  Get the return status for the optimization.*/
  int getReturnStatus();

  SQPParameters params_;

  void callCallbacks(){};

private:
  SQPStatus status_;
  SQPResults results_;

  trajopt::OSQPEigenSolver::Ptr qp_solver_;
};

}  // namespace trajopt

#endif
