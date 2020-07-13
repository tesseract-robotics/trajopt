#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <tesseract/tesseract.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <boost/filesystem/path.hpp>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>


#include <cart_position_optimization_unit.hpp>

/**
 * @brief Applies a cartesian position constraint and solves the problem with trajopt_sqp
 */
TEST_F(CartPositionOptimization, cart_position_optimization_trajopt_sqp)
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

  runTests<trajopt_sqp::TrustRegionSQPSolver>(solver, nlp_, joint_target_);
}

// ////////////////////////////////////////////////////////////////////
//
// int main(int argc, char** argv)
// {
//   testing::InitGoogleTest(&argc, argv);
//
//   return RUN_ALL_TESTS();
// }
