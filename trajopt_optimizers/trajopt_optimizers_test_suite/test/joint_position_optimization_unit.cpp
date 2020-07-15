#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

// Solver Files
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

#include <trajopt_optimizers_test_suite/joint_position_optimization_unit.hpp>

/**
 * @brief Applies a joint position constraint and solves the problem with IPOPT using exact jacobian approximation
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ipopt_exact)  // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 * @brief Applies a joint position constraint and solves the problem with IPOPT, using finite difference values
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ipopt_fdv)  // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}
