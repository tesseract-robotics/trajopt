/**
 * @file joint_velocity_optimization_unit.cpp
 * @brief A joint velocity constraint unit test
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
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

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>

const bool DEBUG = false;

class JerkConstraintOptimization : public testing::TestWithParam<const char*>
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

void runJerkConstraintOptimizationTest(const trajopt_sqp::QPProblem::Ptr& qp_problem)
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);

  // 2) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  std::vector<std::string> joint_names(7, "name");
  {
    auto pos = Eigen::VectorXd::Zero(7);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_0");
    auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
    var->SetBounds(bounds);
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  for (int ind = 1; ind < 6; ind++)
  {
    Eigen::VectorXd pos;
    if (ind == 5)
      pos = Eigen::VectorXd::Ones(7) * 10;
    else
      pos = Eigen::VectorXd::Ones(7) * ((ind / 5.0) * 10 + 0.01);
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
    auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
    var->SetBounds(bounds);
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }

  // 3) Add constraints
  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> start;
  start.push_back(vars.front());
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(7, 5);
  auto start_constraint =
      std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, start, coeffs, "StartPosition");
  qp_problem->addConstraintSet(start_constraint);

  Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7) * 10;
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> end;
  end.push_back(vars.back());
  auto end_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(end_pos, end, coeffs, "EndPosition");
  qp_problem->addConstraintSet(end_constraint);

  // 4) Add costs
  Eigen::VectorXd jerk_target = Eigen::VectorXd::Zero(7);
  coeffs = Eigen::VectorXd::Constant(1, 1);
  auto jerk_constraint = std::make_shared<trajopt_ifopt::JointJerkConstraint>(jerk_target, vars, coeffs, "ja");
  qp_problem->addCostSet(jerk_constraint, trajopt_sqp::CostPenaltyType::SQUARED);

  qp_problem->setup();

  // 6) solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-5);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 2.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 4.0, 1e-1);
  for (Eigen::Index i = 21; i < 28; i++)
    EXPECT_NEAR(x[i], 6, 1e-1);
  for (Eigen::Index i = 28; i < 35; i++)
    EXPECT_NEAR(x[i], 8, 1e-1);
  for (Eigen::Index i = 35; i < 42; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-5);

  if (DEBUG)
    qp_problem->print();
}

/** @brief Joint position constraints with a squared jerk cost in between. Optimized using trajopt_sqp */
TEST_F(JerkConstraintOptimization, jerk_constraint_optimization_ifopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JerkConstraintOptimization, jerk_constraint_optimization_ifopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runJerkConstraintOptimizationTest(qp_problem);
}

TEST_F(JerkConstraintOptimization, jerk_constraint_optimization_trajopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JerkConstraintOptimization, jerk_constraint_optimization_trajopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runJerkConstraintOptimizationTest(qp_problem);
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
