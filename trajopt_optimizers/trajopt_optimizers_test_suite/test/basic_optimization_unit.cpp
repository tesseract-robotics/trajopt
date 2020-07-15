#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

// Problem Header Files
#include <basic_optimization_unit.hpp>

/**
 * @brief Solves the basic optimization problem with the ipopt solver using exact jacobian approximation
 */
TEST_F(BasicOptimization, basic_optimization_ipopt_jacobian_approximation) // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 * @brief Solves the basic optimization problem with the ipopt solver using finite difference values
 */
TEST_F(BasicOptimization, basic_optimization_ipopt_finite_difference_values) // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 * @brief Solves the basic optimization problem with the trajopt_sqp solver
 * NOTE: CURRENTLY BROKEN
 */
TEST_F(BasicOptimization, basic_optimization_trajopt_sqp) // NOLINT
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  solver.verbose = DEBUG;
  runTests<trajopt_sqp::TrustRegionSQPSolver>(solver, nlp_);
}
