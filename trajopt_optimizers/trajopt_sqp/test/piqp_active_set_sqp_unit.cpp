/**
 * @file piqp_active_set_sqp_unit.cpp
 * @brief The PIQP solver's active-set prediction inside the trust region SQP solver
 *
 * @copyright Copyright (c) 2026, Jelle Feringa
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
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_sqp/piqp_solver.h>
#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>

namespace
{
/**
 * @brief Keep a 2D point out of a disc: |p - center|^2 >= radius^2
 * @details Nonconvex and active along the path's arc around the disc, as a collision constraint is around an
 * obstacle.
 */
class DiscConstraint : public trajopt_ifopt::ConstraintSet
{
public:
  DiscConstraint(std::shared_ptr<const trajopt_ifopt::Var> var, Eigen::Vector2d center, double radius, std::string name)
    : ConstraintSet(std::move(name), 1), var_(std::move(var)), center_(std::move(center)), radius_(radius)
  {
    non_zeros_ = 2;
  }

  int update() override { return rows_; }

  Eigen::VectorXd getValues() const override
  {
    return Eigen::VectorXd::Constant(1, (var_->value() - center_).squaredNorm());
  }
  Eigen::VectorXd getCoefficients() const override { return Eigen::VectorXd::Ones(1); }
  std::vector<trajopt_ifopt::Bounds> getBounds() const override
  {
    return { trajopt_ifopt::Bounds(radius_ * radius_, std::numeric_limits<double>::infinity()) };
  }

  trajopt_ifopt::Jacobian getJacobian() const override
  {
    const Eigen::Vector2d gradient = 2.0 * (var_->value() - center_);
    trajopt_ifopt::Jacobian jac(1, variables_->getRows());
    jac.reserve(2);
    jac.startVec(0);
    jac.insertBack(0, var_->getIndex()) = gradient[0];
    jac.insertBack(0, var_->getIndex() + 1) = gradient[1];
    jac.finalize();
    return jac;
  }

private:
  std::shared_ptr<const trajopt_ifopt::Var> var_;
  Eigen::Vector2d center_;
  double radius_;
};

/**
 * @brief A 2D point path from (-2, 0.05) to (2, 0) around two discs, smoothed by a squared velocity cost
 * @details Starts on the straight line through the discs, so the SQP solver needs many iterations, the trust region
 * moves, and the binding disc rows change on the way before settling.
 */
std::shared_ptr<trajopt_sqp::TrajOptQPProblem> makeDiscProblem(int n_nodes)
{
  const Eigen::Vector2d start(-2.0, 0.05);
  const Eigen::Vector2d end(2.0, 0.0);
  const std::vector<std::string> names{ "x", "y" };
  const std::vector<trajopt_ifopt::Bounds> bounds(2, trajopt_ifopt::Bounds(-3.0, 3.0));

  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  for (int i = 0; i < n_nodes; ++i)
  {
    const double s = static_cast<double>(i) / (n_nodes - 1);
    auto node = std::make_unique<trajopt_ifopt::Node>("Node" + std::to_string(i));
    vars.push_back(node->addVar("position", names, (1.0 - s) * start + s * end, bounds));
    nodes.push_back(std::move(node));
  }
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes));

  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);
  qp->addConstraintSet(
      std::make_shared<trajopt_ifopt::JointPosConstraint>(start, vars.front(), Eigen::VectorXd::Ones(2), "Start"));
  qp->addConstraintSet(
      std::make_shared<trajopt_ifopt::JointPosConstraint>(end, vars.back(), Eigen::VectorXd::Ones(2), "End"));
  for (int i = 1; i + 1 < n_nodes; ++i)
  {
    const auto idx = static_cast<std::size_t>(i);
    qp->addConstraintSet(
        std::make_shared<DiscConstraint>(vars[idx], Eigen::Vector2d(-0.6, 0.0), 0.5, "DiscA" + std::to_string(i)));
    qp->addConstraintSet(
        std::make_shared<DiscConstraint>(vars[idx], Eigen::Vector2d(0.7, -0.1), 0.6, "DiscB" + std::to_string(i)));
  }
  qp->addCostSet(std::make_shared<trajopt_ifopt::JointVelConstraint>(
                     Eigen::VectorXd::Zero(2), vars, Eigen::VectorXd::Ones(1), "Velocity"),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  return qp;
}

/** @brief Counts the QP solves of an SQP run and how many of them were the active-set KKT solve */
class CountActiveSetSolves : public trajopt_sqp::SQPCallback
{
public:
  explicit CountActiveSetSolves(std::shared_ptr<const trajopt_sqp::PIQPSolver> solver) : solver_(std::move(solver)) {}

  bool execute(const trajopt_sqp::QPProblem& /*problem*/, const trajopt_sqp::SQPResults& /*sqp_results*/) override
  {
    ++solves;
    active_set_solves += solver_->usedActiveSet() ? 1 : 0;
    last_used_active_set = solver_->usedActiveSet();
    return true;
  }

  int solves{ 0 };
  int active_set_solves{ 0 };
  bool last_used_active_set{ false };

private:
  std::shared_ptr<const trajopt_sqp::PIQPSolver> solver_;
};

struct SQPRun
{
  trajopt_sqp::SQPStatus status;
  trajopt_sqp::SQPResults results;
  int solves;
  int active_set_solves;
  bool last_used_active_set;
};

SQPRun solveDiscProblem(bool active_set_prediction)
{
  auto piqp = std::make_shared<trajopt_sqp::PIQPSolver>();
  piqp->active_set_prediction = active_set_prediction;
  auto counter = std::make_shared<CountActiveSetSolves>(piqp);
  trajopt_sqp::TrustRegionSQPSolver solver(piqp);
  solver.registerCallback(counter);
  solver.solve(makeDiscProblem(24));
  return {
    solver.getStatus(), solver.getResults(), counter->solves, counter->active_set_solves, counter->last_used_active_set
  };
}
}  // namespace

TEST(PIQPActiveSetSQP, SameSolutionWithAndWithoutPrediction)  // NOLINT
{
  const SQPRun plain = solveDiscProblem(false);
  const SQPRun predicted = solveDiscProblem(true);

  ASSERT_EQ(plain.status, trajopt_sqp::SQPStatus::kConverged);
  ASSERT_EQ(predicted.status, trajopt_sqp::SQPStatus::kConverged);
  EXPECT_EQ(plain.active_set_solves, 0);

  // The KKT solve is exact where PIQP stops at its tolerances, so the runs may part in the last digits, not more
  EXPECT_NEAR(predicted.results.best_exact_merit,
              plain.results.best_exact_merit,
              1e-6 * (1.0 + plain.results.best_exact_merit));
  EXPECT_TRUE(predicted.results.best_var_vals.isApprox(plain.results.best_var_vals, 1e-3));
  EXPECT_LE(predicted.results.best_constraint_violations.raw.maxCoeff(), 1e-4);

  // The binding disc rows change over the first iterations, which PIQP solves; once they settle the prediction holds
  EXPECT_EQ(predicted.solves, plain.solves);
  EXPECT_GE(predicted.active_set_solves, 2);
  EXPECT_TRUE(predicted.last_used_active_set);
  RecordProperty("qp_solves", predicted.solves);
  RecordProperty("active_set_solves", predicted.active_set_solves);
  std::cout << "QP solves: " << plain.solves << " plain, " << predicted.solves << " predicted, of which "
            << predicted.active_set_solves << " by the active-set KKT solve\n";
}
