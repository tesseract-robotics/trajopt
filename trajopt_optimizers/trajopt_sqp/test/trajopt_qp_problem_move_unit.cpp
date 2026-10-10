/**
 * @file trajopt_qp_problem_move_unit.cpp
 * @brief Pins that TrajOptQPProblem is nothrow movable by value from a translation unit that sees only the
 *        public header, where its Implementation is incomplete.
 */
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>

static_assert(std::is_nothrow_move_constructible_v<trajopt_sqp::TrajOptQPProblem>);
static_assert(std::is_nothrow_move_assignable_v<trajopt_sqp::TrajOptQPProblem>);

namespace
{
std::shared_ptr<trajopt_ifopt::NodesVariables> makeVariables(const Eigen::VectorXd& start)
{
  const auto n = static_cast<std::size_t>(start.size());
  auto node = std::make_unique<trajopt_ifopt::Node>("node0");
  node->addVar("position",
               std::vector<std::string>(n, "j"),
               start,
               std::vector<trajopt_ifopt::Bounds>(n, trajopt_ifopt::NoBound));
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  nodes.push_back(std::move(node));
  return std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes));
}
}  // namespace

TEST(TrajOptQPProblemMove, MoveConstructCarriesTheProblem)  // NOLINT
{
  trajopt_sqp::TrajOptQPProblem source(makeVariables(Eigen::Vector3d(1, 2, 3)));
  trajopt_sqp::TrajOptQPProblem moved(std::move(source));
  EXPECT_EQ(moved.getNumNLPVars(), 3);
  EXPECT_TRUE(moved.getVariableValues().isApprox(Eigen::Vector3d(1, 2, 3)));
}

TEST(TrajOptQPProblemMove, MoveAssignCarriesTheProblem)  // NOLINT
{
  trajopt_sqp::TrajOptQPProblem source(makeVariables(Eigen::Vector3d(1, 2, 3)));
  trajopt_sqp::TrajOptQPProblem target(makeVariables(Eigen::Vector2d(4, 5)));
  target = std::move(source);
  EXPECT_EQ(target.getNumNLPVars(), 3);
  EXPECT_TRUE(target.getVariableValues().isApprox(Eigen::Vector3d(1, 2, 3)));
}
