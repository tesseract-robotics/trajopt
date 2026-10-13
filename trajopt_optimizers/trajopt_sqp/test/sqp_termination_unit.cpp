/**
 * @file sqp_termination_unit.cpp
 * @brief Tests how TrustRegionSQPSolver ends a solve and reports why
 *
 * @author Roelof Oomen
 * @date October 13, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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
#include <Eigen/Core>
#include <tesseract/common/logging.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>

#include "scripted_qp_solver.h"

using trajopt_sqp::SQPStatus;

namespace
{
/** @brief Two variables in [-1, 1], starting at 0 and pulled toward (0.8, -0.3) by a squared cost */
std::shared_ptr<trajopt_sqp::TrajOptQPProblem> makeProblem()
{
  auto node = std::make_unique<trajopt_ifopt::Node>("Joints");
  const std::vector<std::string> names{ "j0", "j1" };
  const std::vector<trajopt_ifopt::Bounds> bounds(2, trajopt_ifopt::Bounds(-1.0, 1.0));
  const std::shared_ptr<const trajopt_ifopt::Var> var =
      node->addVar("position", names, Eigen::Vector2d::Zero(), bounds);
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  nodes.push_back(std::move(node));
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes));

  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);
  qp->addCostSet(std::make_shared<trajopt_ifopt::JointPosConstraint>(
                     Eigen::Vector2d(0.8, -0.3), var, Eigen::VectorXd::Ones(1), "Target"),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  return qp;
}

/** @brief A QP solver that solves with OSQP and reports what its script says */
std::shared_ptr<trajopt_sqp::test::ScriptedQPSolver> scriptedOsqp()
{
  return std::make_shared<trajopt_sqp::test::ScriptedQPSolver>(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
}
}  // namespace

class SQPTermination : public testing::Test
{
protected:
  void SetUp() override
  {
    level_ = tesseract::common::getLogger()->level();
    tesseract::common::getLogger()->set_level(spdlog::level::off);
  }

  void TearDown() override { tesseract::common::getLogger()->set_level(level_); }

private:
  spdlog::level::level_enum level_{ spdlog::level::info };
};

TEST_F(SQPTermination, SpentFailureBudgetEndsTheSolve)  // NOLINT
{
  auto scripted = scriptedOsqp();
  scripted->every_solve = trajopt_sqp::test::ScriptedSolve{ false };
  trajopt_sqp::TrustRegionSQPSolver solver(scripted);
  solver.solve(makeProblem());
  EXPECT_EQ(solver.getStatus(), SQPStatus::kQPSolveFailed);
  EXPECT_EQ(scripted->solves, solver.params.max_qp_solver_failures + 1);
}

TEST_F(SQPTermination, FailureShrinkingToTinyBoxIsATinyBoxExit)  // NOLINT
{
  auto scripted = scriptedOsqp();
  scripted->script.push_back({ false });
  trajopt_sqp::TrustRegionSQPSolver solver(scripted);
  // One shrink by trust_shrink_ratio lands below min_trust_box_size
  solver.params.initial_trust_box_size = 1.5 * solver.params.min_trust_box_size;
  solver.solve(makeProblem());
  EXPECT_EQ(solver.getStatus(), SQPStatus::kConverged);
  EXPECT_EQ(scripted->solves, 1);
}
