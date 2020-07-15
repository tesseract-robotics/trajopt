#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>

#include <trajopt_optimizers_test_suite/joint_velocity_optimization_unit.hpp>

/**
@brief Joint position constraints with a squared velocity cost in between. Optimized using ipopt
 */
TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_ipopt)  // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 @brief Joint position constraints with a squared velocity cost in between. Optimized using trajopt_sqp
 */
TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_trajopt_sqp)  // NOLINT
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);

  solver.verbose = DEBUG;
  runTests<trajopt_sqp::TrustRegionSQPSolver>(solver, nlp_);
}
