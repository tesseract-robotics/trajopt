#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

//Solver Files
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

//Problem Header Files
#include <basic_optimization_unit.hpp>

/**
 * @brief Applies a joint position constraint and solves the problem with IPOPT
 */
TEST_F(BasicOptimization, basic_optimization_ipopt)
{
  ifopt::IpoptSolver solver;
  // solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
   // ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}
/**
 * @brief Applies a joint position constraint and solves the problem with trajopt_sqp
 */
TEST_F(BasicOptimization, basic_optimization_trajopt_sqp)
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

  // solve
  solver.verbose = DEBUG;
  runTests<trajopt_sqp::TrustRegionSQPSolver>(solver, nlp_);
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  std::cout << "testing" << std::endl;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
