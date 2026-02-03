/**
 * @file joint_position_optimization_unit.cpp
 * @brief A joint position constraint unit test
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <OsqpEigen/OsqpEigen.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

const bool DEBUG = false;

class JointPositionOptimization : public testing::TestWithParam<const char*>
{
public:
  void SetUp() override
  {
    if (DEBUG)
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    else
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);
  }
};

template <typename T>
void runJointPositionOptimizationTest()
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_->settings()->setVerbosity(DEBUG);
  qp_solver->solver_->settings()->setWarmStart(true);
  qp_solver->solver_->settings()->setPolish(true);
  qp_solver->solver_->settings()->setAdaptiveRho(false);
  qp_solver->solver_->settings()->setMaxIteration(8192);
  qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_->settings()->setRelativeTolerance(1e-6);

  // 1) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  for (int ind = 0; ind < 2; ind++)
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_" + std::to_string(ind));
    auto pos = Eigen::VectorXd::Zero(7);
    const std::vector<std::string> joint_names(7, "name");
    const std::vector<trajopt_ifopt::Bounds> bounds(7, trajopt_ifopt::NoBound);
    vars.push_back(node->addVar("position", joint_names, pos, bounds));
    nodes.push_back(std::move(node));
  }
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));

  // 2) Create problem
  auto qp_problem = std::make_shared<T>(variables);

  // 3) Add constraints
  const Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(7, 1);
  auto start_constraint =
      std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, vars.front(), coeffs, "StartPosition");
  qp_problem->addConstraintSet(start_constraint);

  const Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7);
  auto end_constraint =
      std::make_shared<trajopt_ifopt::JointPosConstraint>(end_pos, vars.back(), coeffs, "EndPosition");
  qp_problem->addConstraintSet(end_constraint);

  qp_problem->setup();

  // solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-5);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 1.0, 1e-5);

  if (DEBUG)
    qp_problem->print();
}

/**
 * @brief Applies a joint position constraint and solves the ifopt problem with trajopt_sqp
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ifopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointPositionOptimization, joint_position_optimization_ifopt_problem");
  runJointPositionOptimizationTest<trajopt_sqp::IfoptQPProblem>();
}

/**
 * @brief Applies a joint position constraint and solves the ifopt problem with trajopt_sqp
 */
TEST_F(JointPositionOptimization, joint_position_optimization_trajopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointPositionOptimization, joint_position_optimization_trajopt_problem");
  runJointPositionOptimizationTest<trajopt_sqp::TrajOptQPProblem>();
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
