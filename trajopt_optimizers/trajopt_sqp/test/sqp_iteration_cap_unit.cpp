/**
 * @file sqp_iteration_cap_unit.cpp
 * @brief Tests the convexification budget of each SQP penalty iteration
 *
 * @author Roelof Oomen
 * @date October 1, 2026
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
#include <optional>
#include <string>
#include <vector>

#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>

using trajopt_sqp::SQPStatus;

namespace
{
/**
 * @brief One variable in [-1, 1] pulled toward 0.8 by a squared cost, optionally constrained to x = target
 * @param constraint_target When set, adds the hard constraint x = constraint_target (outside [-1, 1] is infeasible)
 */
std::shared_ptr<trajopt_sqp::TrajOptQPProblem> makeProblem(std::optional<double> constraint_target = std::nullopt)
{
  auto node = std::make_unique<trajopt_ifopt::Node>("Joints");
  const std::shared_ptr<const trajopt_ifopt::Var> var =
      node->addVar("position", { "j0" }, Eigen::VectorXd::Zero(1), { trajopt_ifopt::Bounds(-1.0, 1.0) });
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  nodes.push_back(std::move(node));
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes));

  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);
  qp->addCostSet(std::make_shared<trajopt_ifopt::JointPosConstraint>(
                     Eigen::VectorXd::Constant(1, 0.8), var, Eigen::VectorXd::Ones(1), "Target"),
                 trajopt_sqp::CostPenaltyType::kSquared);
  if (constraint_target)
  {
    const std::vector<trajopt_ifopt::Bounds> cnt_bounds{ trajopt_ifopt::Bounds(*constraint_target,
                                                                               *constraint_target) };
    qp->addConstraintSet(
        std::make_shared<trajopt_ifopt::JointPosConstraint>(cnt_bounds, var, Eigen::VectorXd::Ones(1), "Pin"));
  }
  qp->setup();
  return qp;
}

trajopt_sqp::TrustRegionSQPSolver makeSolver() { return { std::make_shared<trajopt_sqp::OSQPEigenSolver>() }; }
}  // namespace

class SQPIterationCap : public testing::Test
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

TEST_F(SQPIterationCap, ARoundRunsUpToMaxIterConvexifications)  // NOLINT
{
  // A far target, a small fixed box and no expansion: every convexification accepts one short step
  auto node = std::make_unique<trajopt_ifopt::Node>("Joints");
  const std::shared_ptr<const trajopt_ifopt::Var> var =
      node->addVar("position", { "j0" }, Eigen::VectorXd::Zero(1), { trajopt_ifopt::Bounds(-1000.0, 1000.0) });
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  nodes.push_back(std::move(node));
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(
      std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes)));
  qp->addCostSet(std::make_shared<trajopt_ifopt::JointPosConstraint>(
                     Eigen::VectorXd::Constant(1, 100.0), var, Eigen::VectorXd::Ones(1), "Far"),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();

  auto solver = makeSolver();
  solver.params.initial_trust_box_size = 0.5;
  solver.params.trust_expand_ratio = 1.0;
  solver.params.max_iter = 150;
  solver.solve(qp);
  EXPECT_EQ(solver.getResults().convexify_iteration, 150);
  EXPECT_NEAR(solver.getResults().best_var_vals[0], 75.0, 1e-3);
  // Without constraints the iterate is feasible, which ends the solve
  EXPECT_EQ(solver.getStatus(), SQPStatus::kConverged);
}

TEST_F(SQPIterationCap, IterationLimitAtAnInfeasibleIterateRaisesThePenalty)  // NOLINT
{
  auto solver = makeSolver();
  solver.params.max_iter = 1;
  solver.params.max_merit_coeff_increases = 2;
  solver.solve(makeProblem(5.0));  // the pin lies outside the variable bounds
  EXPECT_EQ(solver.getStatus(), SQPStatus::kPenaltyIterationLimit);
  EXPECT_EQ(solver.getResults().penalty_iteration, 1);
}

TEST_F(SQPIterationCap, EachPenaltyIterationGetsItsOwnIterationBudget)  // NOLINT
{
  // A fixed 0.01 box and no expansion: every convexification moves x0 by 0.01 toward a pin it cannot reach in time
  auto solver = makeSolver();
  solver.params.initial_trust_box_size = 0.01;
  solver.params.trust_expand_ratio = 1.0;
  solver.params.max_iter = 3;
  solver.params.max_merit_coeff_increases = 3;
  solver.solve(makeProblem(0.9));
  EXPECT_EQ(solver.getStatus(), SQPStatus::kPenaltyIterationLimit);
  EXPECT_GE(solver.getResults().overall_iteration, 9);
  EXPECT_NEAR(solver.getResults().best_var_vals[0], 0.09, 1e-3);
}

TEST_F(SQPIterationCap, ZeroIterationBudgetEndsWithoutAStep)  // NOLINT
{
  {  // A feasible start ends at once
    auto solver = makeSolver();
    solver.params.max_iter = 0;
    solver.solve(makeProblem());
    EXPECT_EQ(solver.getStatus(), SQPStatus::kConverged);
    EXPECT_EQ(solver.getResults().overall_iteration, 0);
  }

  {  // An infeasible start raises the penalty every penalty iteration without a step
    auto solver = makeSolver();
    solver.params.max_iter = 0;
    solver.solve(makeProblem(0.5));
    EXPECT_EQ(solver.getStatus(), SQPStatus::kPenaltyIterationLimit);
    EXPECT_EQ(solver.getResults().overall_iteration, 0);
  }
}
