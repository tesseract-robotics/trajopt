/**
 * @file joint_velocity_optimization_unit.cpp
 * @brief A joint velocity constraint unit test
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
#include <tesseract/common/logging.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/costs/squared_cost.h>

const bool DEBUG = false;

class VelocityConstraintOptimization : public testing::TestWithParam<const char*>
{
public:
  void SetUp() override
  {
    if (DEBUG)
      tesseract::common::getLogger()->set_level(spdlog::level::debug);
    else
      tesseract::common::getLogger()->set_level(spdlog::level::off);
  }
};

template <typename T>
void runVelocityConstraintOptimizationTest()
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_->settings()->setVerbosity(DEBUG);
  qp_solver->solver_->settings()->setWarmStart(true);
  qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_->settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_->settings()->setMaxIteration(8192);
  qp_solver->solver_->settings()->setPolish(true);
  qp_solver->solver_->settings()->setAdaptiveRho(false);

  // 1) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  const std::vector<std::string> joint_names(7, "name");
  const auto bounds = std::vector<trajopt_ifopt::Bounds>(7, trajopt_ifopt::NoBound);
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_0");
    auto pos = Eigen::VectorXd::Zero(7);
    vars.push_back(node->addVar("position", joint_names, pos, bounds));
    nodes.push_back(std::move(node));
  }
  for (int ind = 1; ind < 3; ind++)
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_" + std::to_string(ind));
    auto pos = Eigen::VectorXd::Ones(7) * 10;
    vars.push_back(node->addVar("position", joint_names, pos, bounds));
    nodes.push_back(std::move(node));
  }
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));

  // 2) Create problem
  auto qp_problem = std::make_shared<T>(variables);

  // 3) Add constraints
  const Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(7, 5);
  auto start_constraint =
      std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, vars.front(), coeffs, "StartPosition");
  qp_problem->addConstraintSet(start_constraint);

  const Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7) * 10;
  auto end_constraint =
      std::make_shared<trajopt_ifopt::JointPosConstraint>(end_pos, vars.back(), coeffs, "EndPosition");
  qp_problem->addConstraintSet(end_constraint);

  // 4) Add costs
  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(7);
  coeffs = Eigen::VectorXd::Constant(1, 1);
  auto vel_constraint = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, vars, coeffs, "jv");
  qp_problem->addCostSet(vel_constraint, trajopt_sqp::CostPenaltyType::kSquared);

  qp_problem->setup();

  // 6) solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-3);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 5.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-3);

  if (DEBUG)
    qp_problem->print();
}

/** @brief Joint position constraints with a squared velocity cost in between. Optimized using trajopt_sqp */
TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_ifopt_problem)  // NOLINT
{
  TESSERACT_LOG_DEBUG("VelocityConstraintOptimization, velocity_constraint_optimization_ifopt_problem");
  runVelocityConstraintOptimizationTest<trajopt_sqp::IfoptQPProblem>();
}

TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_trajopt_problem)  // NOLINT
{
  TESSERACT_LOG_DEBUG("VelocityConstraintOptimization, velocity_constraint_optimization_trajopt_problem");
  runVelocityConstraintOptimizationTest<trajopt_sqp::TrajOptQPProblem>();
}

/**
 * @brief A symmetric velocity band (-v <= dq <= v) is a range bound. The bounds constructor splits it into two
 * one-sided rows, which TrajOptQPProblem accepts, and the solution stays inside the band.
 */
TEST_F(VelocityConstraintOptimization, velocity_band_constraint_trajopt_problem)  // NOLINT
{
  TESSERACT_LOG_DEBUG("VelocityConstraintOptimization, velocity_band_constraint_trajopt_problem");

  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_->settings()->setVerbosity(DEBUG);
  qp_solver->solver_->settings()->setAbsoluteTolerance(1e-6);
  qp_solver->solver_->settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_->settings()->setMaxIteration(8192);
  qp_solver->solver_->settings()->setPolish(true);

  // 1) Four states of two joints, all starting at zero
  constexpr Eigen::Index n_dof = 2;
  constexpr int n_states = 4;
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  const std::vector<std::string> joint_names{ "a", "b" };
  const auto var_bounds = std::vector<trajopt_ifopt::Bounds>(n_dof, trajopt_ifopt::NoBound);
  for (int ind = 0; ind < n_states; ind++)
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_" + std::to_string(ind));
    vars.push_back(node->addVar("position", joint_names, Eigen::VectorXd::Zero(n_dof), var_bounds));
    nodes.push_back(std::move(node));
  }
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);

  // 2) Start fixed at zero
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(n_dof, 5);
  qp_problem->addConstraintSet(std::make_shared<trajopt_ifopt::JointPosConstraint>(
      Eigen::VectorXd::Zero(n_dof), vars.front(), coeffs, "StartPosition"));

  // 3) Velocity band: each step may move each joint by at most v
  const double v = 1.0;
  const std::vector<trajopt_ifopt::Bounds> band(n_dof, trajopt_ifopt::Bounds(-v, v));
  auto vel_band = std::make_shared<trajopt_ifopt::JointVelConstraint>(band, vars, Eigen::VectorXd(), "VelocityBand");
  EXPECT_EQ(vel_band->getRows(), 2 * n_dof * (n_states - 1));
  qp_problem->addConstraintSet(vel_band);

  // 4) Pull the last state beyond the band's reach: joint a up, joint b down
  const Eigen::Vector2d end_target(10, -10);
  qp_problem->addCostSet(
      std::make_shared<trajopt_ifopt::JointPosConstraint>(end_target, vars.back(), coeffs, "EndPosition"),
      trajopt_sqp::CostPenaltyType::kSquared);

  EXPECT_NO_THROW(qp_problem->setup());  // NOLINT

  // 5) Solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  EXPECT_EQ(solver.getStatus(), trajopt_sqp::SQPStatus::kConverged);
  const Eigen::VectorXd x = qp_problem->getVariableValues();

  // Every step stays in the band, and the band is what stops the last state short of its target
  constexpr double tol = 1e-4;
  for (int step = 0; step < n_states - 1; ++step)
  {
    const Eigen::VectorXd dq = x.segment((step + 1) * n_dof, n_dof) - x.segment(step * n_dof, n_dof);
    for (Eigen::Index k = 0; k < n_dof; ++k)
    {
      EXPECT_LE(dq(k), v + tol);
      EXPECT_GE(dq(k), -v - tol);
    }
  }
  EXPECT_NEAR(x((n_states - 1) * n_dof), (n_states - 1) * v, tol);
  EXPECT_NEAR(x(((n_states - 1) * n_dof) + 1), -(n_states - 1) * v, tol);
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
