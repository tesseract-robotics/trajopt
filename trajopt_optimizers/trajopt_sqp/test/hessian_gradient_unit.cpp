/**
 * @file hessian_gradient_unit.cpp
 * @brief White-box tests for Gauss-Newton Hessian and gradient correction
 *
 * Tests verify updateHessian() and updateGradient() produce analytically
 * correct values, not just that the solver converges to the right answer.
 * Templated to run against both IfoptQPProblem and TrajOptQPProblem.
 *
 * @author Jelle Feringa
 * @date February 26, 2026
 *
 * @copyright Copyright (c) 2026, Southwest Research Institute
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
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

const bool DEBUG = false;

class HessianGradientTest : public testing::Test
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

/**
 * @brief Helper: create a 2-waypoint, n-joint problem with given initial positions
 */
struct MinimalProblem
{
  std::shared_ptr<trajopt_ifopt::NodesVariables> variables;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;

  MinimalProblem(Eigen::Index n_joints, const Eigen::VectorXd& pos0, const Eigen::VectorXd& pos1)
  {
    std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
    const std::vector<std::string> joint_names(static_cast<std::size_t>(n_joints), "j");
    const std::vector<trajopt_ifopt::Bounds> bounds(static_cast<std::size_t>(n_joints), trajopt_ifopt::NoBound);

    auto node0 = std::make_unique<trajopt_ifopt::Node>("wp_0");
    vars.push_back(node0->addVar("position", joint_names, pos0, bounds));
    nodes.push_back(std::move(node0));

    auto node1 = std::make_unique<trajopt_ifopt::Node>("wp_1");
    vars.push_back(node1->addVar("position", joint_names, pos1, bounds));
    nodes.push_back(std::move(node1));

    variables = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));
  }
};

/**
 * @brief Helper: create a 3-waypoint, n-joint problem
 */
struct ThreeWaypointProblem
{
  std::shared_ptr<trajopt_ifopt::NodesVariables> variables;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;

  ThreeWaypointProblem(Eigen::Index n_joints,
                       const Eigen::VectorXd& pos0,
                       const Eigen::VectorXd& pos1,
                       const Eigen::VectorXd& pos2)
  {
    std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
    const std::vector<std::string> joint_names(static_cast<std::size_t>(n_joints), "j");
    const std::vector<trajopt_ifopt::Bounds> bounds(static_cast<std::size_t>(n_joints), trajopt_ifopt::NoBound);

    auto node0 = std::make_unique<trajopt_ifopt::Node>("wp_0");
    vars.push_back(node0->addVar("position", joint_names, pos0, bounds));
    nodes.push_back(std::move(node0));

    auto node1 = std::make_unique<trajopt_ifopt::Node>("wp_1");
    vars.push_back(node1->addVar("position", joint_names, pos1, bounds));
    nodes.push_back(std::move(node1));

    auto node2 = std::make_unique<trajopt_ifopt::Node>("wp_2");
    vars.push_back(node2->addVar("position", joint_names, pos2, bounds));
    nodes.push_back(std::move(node2));

    variables = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));
  }
};

// ============================================================
// Templated test functions — run against both QP problem types
// ============================================================

/** @brief Single velocity cost, unit weights → H = J^T J */
template <typename T>
void runHessianSingleCostTest()
{
  const Eigen::Index nj = 2;
  MinimalProblem prob(nj, Eigen::VectorXd::Zero(nj), Eigen::VectorXd::Ones(nj) * 3.0);

  auto qp = std::make_shared<T>(prob.variables);

  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(nj);
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(1);
  auto vel_cost = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  qp->addCostSet(vel_cost, trajopt_sqp::CostPenaltyType::kSquared);

  qp->setup();
  qp->convexify();

  // Velocity Jacobian: J = [-I | I] for 2 joints, 2 waypoints
  // H = J^T J = [ I  -I ]
  //             [-I   I ]
  const auto& H = qp->getHessian();
  const Eigen::Index n = qp->getNumNLPVars();
  ASSERT_EQ(n, 4);

  Eigen::MatrixXd H_dense = Eigen::MatrixXd(H.block(0, 0, n, n));

  EXPECT_NEAR(H_dense(0, 0), 1.0, 1e-8);
  EXPECT_NEAR(H_dense(1, 1), 1.0, 1e-8);
  EXPECT_NEAR(H_dense(2, 2), 1.0, 1e-8);
  EXPECT_NEAR(H_dense(3, 3), 1.0, 1e-8);
  EXPECT_NEAR(H_dense(0, 2), -1.0, 1e-8);
  EXPECT_NEAR(H_dense(2, 0), -1.0, 1e-8);
  EXPECT_NEAR(H_dense(1, 3), -1.0, 1e-8);
  EXPECT_NEAR(H_dense(3, 1), -1.0, 1e-8);
  EXPECT_NEAR(H_dense(0, 1), 0.0, 1e-8);
  EXPECT_NEAR(H_dense(0, 3), 0.0, 1e-8);
  EXPECT_NEAR(H_dense(1, 2), 0.0, 1e-8);
  EXPECT_NEAR(H_dense(2, 3), 0.0, 1e-8);

  // Slack variable region should be zero
  for (Eigen::Index r = n; r < H.rows(); ++r)
    for (Eigen::Index c = 0; c < H.cols(); ++c)
      EXPECT_NEAR(H.coeff(r, c), 0.0, 1e-8);
}

/** @brief Velocity cost with non-unit weights → H = J^T diag(w) J */
template <typename T>
void runHessianWeightedTest()
{
  const Eigen::Index nj = 2;
  MinimalProblem prob(nj, Eigen::VectorXd::Zero(nj), Eigen::VectorXd::Ones(nj) * 3.0);

  auto qp = std::make_shared<T>(prob.variables);

  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(nj);
  Eigen::VectorXd coeffs(1);
  coeffs[0] = 5.0;
  auto vel_cost = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  qp->addCostSet(vel_cost, trajopt_sqp::CostPenaltyType::kSquared);

  qp->setup();
  qp->convexify();

  const auto& H = qp->getHessian();
  const Eigen::Index n = qp->getNumNLPVars();
  Eigen::MatrixXd H_dense = Eigen::MatrixXd(H.block(0, 0, n, n));

  // H = J^T * diag(w) * J, where w = [5, 5] (coefficient applied per-row)
  // Expected: 5 * [I -I; -I I]
  EXPECT_NEAR(H_dense(0, 0), 5.0, 1e-8);
  EXPECT_NEAR(H_dense(1, 1), 5.0, 1e-8);
  EXPECT_NEAR(H_dense(0, 2), -5.0, 1e-8);
  EXPECT_NEAR(H_dense(2, 0), -5.0, 1e-8);
}

/** @brief Multiple costs accumulate → H = H_vel + H_accel */
template <typename T>
void runHessianMultipleCostsTest()
{
  const Eigen::Index nj = 2;
  ThreeWaypointProblem prob(
      nj, Eigen::VectorXd::Zero(nj), Eigen::VectorXd::Ones(nj) * 5.0, Eigen::VectorXd::Ones(nj) * 10.0);

  // Velocity-only problem
  auto qp_vel = std::make_shared<T>(prob.variables);
  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(nj);
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(1);
  auto vel_cost = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  qp_vel->addCostSet(vel_cost, trajopt_sqp::CostPenaltyType::kSquared);
  qp_vel->setup();
  qp_vel->convexify();

  // Acceleration-only problem
  auto qp_accel = std::make_shared<T>(prob.variables);
  const Eigen::VectorXd accel_target = Eigen::VectorXd::Zero(nj);
  auto accel_cost = std::make_shared<trajopt_ifopt::JointAccelConstraint>(accel_target, prob.vars, coeffs, "accel");
  qp_accel->addCostSet(accel_cost, trajopt_sqp::CostPenaltyType::kSquared);
  qp_accel->setup();
  qp_accel->convexify();

  // Combined problem
  auto qp_both = std::make_shared<T>(prob.variables);
  auto vel_cost2 = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  auto accel_cost2 = std::make_shared<trajopt_ifopt::JointAccelConstraint>(accel_target, prob.vars, coeffs, "accel");
  qp_both->addCostSet(vel_cost2, trajopt_sqp::CostPenaltyType::kSquared);
  qp_both->addCostSet(accel_cost2, trajopt_sqp::CostPenaltyType::kSquared);
  qp_both->setup();
  qp_both->convexify();

  const Eigen::Index n = qp_both->getNumNLPVars();
  Eigen::MatrixXd H_vel = Eigen::MatrixXd(qp_vel->getHessian().block(0, 0, n, n));
  Eigen::MatrixXd H_accel = Eigen::MatrixXd(qp_accel->getHessian().block(0, 0, n, n));
  Eigen::MatrixXd H_both = Eigen::MatrixXd(qp_both->getHessian().block(0, 0, n, n));

  Eigen::MatrixXd H_expected = H_vel + H_accel;
  for (Eigen::Index r = 0; r < n; ++r)
    for (Eigen::Index c = 0; c < n; ++c)
      EXPECT_NEAR(H_both(r, c), H_expected(r, c), 1e-8) << "Mismatch at (" << r << "," << c << ")";
}

/** @brief Gradient correction q = 2*J^T*W*e - 2*H*x0 */
template <typename T>
void runGradientCorrectionTest()
{
  const Eigen::Index nj = 2;
  Eigen::VectorXd pos0(nj);
  Eigen::VectorXd pos1(nj);
  pos0 << 0.0, 0.0;
  pos1 << 3.0, 5.0;

  MinimalProblem prob(nj, pos0, pos1);

  auto qp = std::make_shared<T>(prob.variables);

  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(nj);
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(1);
  auto vel_cost = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  qp->addCostSet(vel_cost, trajopt_sqp::CostPenaltyType::kSquared);

  qp->setup();
  qp->convexify();

  const auto& gradient = qp->getGradient();
  const Eigen::Index n = qp->getNumNLPVars();

  // For velocity constraint: J = [-I | I], e = pos1 - pos0 = [3, 5]
  // g = 2 * J^T * e = 2 * [-3, -5, 3, 5]
  // H*x0 = [I -I; -I I] * [0, 0, 3, 5] = [-3, -5, 3, 5]
  // q = g - 2*H*x0 = [-6, -10, 6, 10] - 2*[-3, -5, 3, 5] = [0, 0, 0, 0]
  //
  // The NLP gradient is zero because velocity is linear (Gauss-Newton is exact).
  for (Eigen::Index i = 0; i < n; ++i)
    EXPECT_NEAR(gradient[i], 0.0, 1e-8) << "Non-zero NLP gradient at index " << i;
}

/** @brief Sparsity pattern preserved across convexify() calls (#312) */
template <typename T>
void runSparsityPreservationTest()
{
  const Eigen::Index nj = 2;
  MinimalProblem prob(nj, Eigen::VectorXd::Zero(nj), Eigen::VectorXd::Ones(nj) * 3.0);

  auto qp = std::make_shared<T>(prob.variables);

  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(nj);
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(1);
  auto vel_cost = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  qp->addCostSet(vel_cost, trajopt_sqp::CostPenaltyType::kSquared);

  qp->setup();

  // First convexify
  qp->convexify();
  const Eigen::Index nnz_first = qp->getHessian().nonZeros();
  ASSERT_GT(nnz_first, 0);

  // Move variables to a different point and reconvexify
  Eigen::VectorXd new_x = Eigen::VectorXd::Zero(qp->getNumNLPVars());
  new_x << 1.0, 1.0, 1.0, 1.0;
  qp->setVariables(new_x.data());
  qp->convexify();
  const Eigen::Index nnz_second = qp->getHessian().nonZeros();

  // Sparsity pattern must not change between iterations (critical for OSQP warm-start / #312)
  EXPECT_EQ(nnz_first, nnz_second);
}

/** @brief No costs → Hessian is zero */
template <typename T>
void runEmptyCostsTest()
{
  const Eigen::Index nj = 2;
  MinimalProblem prob(nj, Eigen::VectorXd::Zero(nj), Eigen::VectorXd::Ones(nj));

  auto qp = std::make_shared<T>(prob.variables);

  // Add a constraint but no costs
  const Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(nj);
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(nj);
  auto start_constraint =
      std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, prob.vars.front(), coeffs, "start");
  qp->addConstraintSet(start_constraint);

  qp->setup();
  qp->convexify();

  const auto& H = qp->getHessian();
  EXPECT_EQ(H.nonZeros(), 0);

  const auto& gradient = qp->getGradient();
  for (Eigen::Index i = 0; i < qp->getNumNLPVars(); ++i)
    EXPECT_NEAR(gradient[i], 0.0, 1e-8);
}

/** @brief Hessian is symmetric positive semi-definite */
template <typename T>
void runHessianSymmetricPSDTest()
{
  const Eigen::Index nj = 3;
  Eigen::VectorXd pos0(nj);
  Eigen::VectorXd pos1(nj);
  pos0 << 1.0, 2.0, 3.0;
  pos1 << 4.0, 1.0, 7.0;

  MinimalProblem prob(nj, pos0, pos1);

  auto qp = std::make_shared<T>(prob.variables);

  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(nj);
  Eigen::VectorXd coeffs(1);
  coeffs[0] = 3.0;
  auto vel_cost = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  qp->addCostSet(vel_cost, trajopt_sqp::CostPenaltyType::kSquared);

  qp->setup();
  qp->convexify();

  const auto& H = qp->getHessian();
  const Eigen::Index n = qp->getNumNLPVars();
  Eigen::MatrixXd H_dense = Eigen::MatrixXd(H.block(0, 0, n, n));

  // Symmetry
  for (Eigen::Index r = 0; r < n; ++r)
    for (Eigen::Index c = r + 1; c < n; ++c)
      EXPECT_NEAR(H_dense(r, c), H_dense(c, r), 1e-10) << "Asymmetric at (" << r << "," << c << ")";

  // Positive semi-definite: all eigenvalues >= 0
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(H_dense);
  ASSERT_EQ(eigensolver.info(), Eigen::Success);
  for (Eigen::Index i = 0; i < n; ++i)
    EXPECT_GE(eigensolver.eigenvalues()[i], -1e-10) << "Negative eigenvalue at index " << i;
}

// ============================================================
// IfoptQPProblem tests
// ============================================================

TEST_F(HessianGradientTest, hessian_single_cost_ifopt_problem)
{
  runHessianSingleCostTest<trajopt_sqp::IfoptQPProblem>();
}
TEST_F(HessianGradientTest, hessian_weighted_ifopt_problem) { runHessianWeightedTest<trajopt_sqp::IfoptQPProblem>(); }
TEST_F(HessianGradientTest, hessian_multiple_costs_ifopt_problem)
{
  runHessianMultipleCostsTest<trajopt_sqp::IfoptQPProblem>();
}
TEST_F(HessianGradientTest, gradient_correction_ifopt_problem)
{
  runGradientCorrectionTest<trajopt_sqp::IfoptQPProblem>();
}
TEST_F(HessianGradientTest, sparsity_preservation_ifopt_problem)
{
  runSparsityPreservationTest<trajopt_sqp::IfoptQPProblem>();
}
TEST_F(HessianGradientTest, empty_costs_ifopt_problem) { runEmptyCostsTest<trajopt_sqp::IfoptQPProblem>(); }
TEST_F(HessianGradientTest, hessian_symmetric_psd_ifopt_problem)
{
  runHessianSymmetricPSDTest<trajopt_sqp::IfoptQPProblem>();
}

// ============================================================
// TrajOptQPProblem tests
// ============================================================

TEST_F(HessianGradientTest, hessian_single_cost_trajopt_problem)
{
  runHessianSingleCostTest<trajopt_sqp::TrajOptQPProblem>();
}
TEST_F(HessianGradientTest, hessian_weighted_trajopt_problem)
{
  runHessianWeightedTest<trajopt_sqp::TrajOptQPProblem>();
}
TEST_F(HessianGradientTest, hessian_multiple_costs_trajopt_problem)
{
  runHessianMultipleCostsTest<trajopt_sqp::TrajOptQPProblem>();
}
TEST_F(HessianGradientTest, gradient_correction_trajopt_problem)
{
  runGradientCorrectionTest<trajopt_sqp::TrajOptQPProblem>();
}
TEST_F(HessianGradientTest, sparsity_preservation_trajopt_problem)
{
  runSparsityPreservationTest<trajopt_sqp::TrajOptQPProblem>();
}
TEST_F(HessianGradientTest, empty_costs_trajopt_problem) { runEmptyCostsTest<trajopt_sqp::TrajOptQPProblem>(); }
TEST_F(HessianGradientTest, hessian_symmetric_psd_trajopt_problem)
{
  runHessianSymmetricPSDTest<trajopt_sqp::TrajOptQPProblem>();
}

// ============================================================
// Cross-implementation agreement: both produce the same Hessian and gradient
// ============================================================

TEST_F(HessianGradientTest, cross_implementation_agreement)
{
  const Eigen::Index nj = 3;
  Eigen::VectorXd pos0(nj);
  Eigen::VectorXd pos1(nj);
  pos0 << 1.0, -2.0, 0.5;
  pos1 << 4.0, 1.0, -3.0;

  MinimalProblem prob(nj, pos0, pos1);

  // Build identical problems with both implementations
  auto qp_ifopt = std::make_shared<trajopt_sqp::IfoptQPProblem>(prob.variables);
  auto qp_trajopt = std::make_shared<trajopt_sqp::TrajOptQPProblem>(prob.variables);

  const Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(nj);
  Eigen::VectorXd coeffs(1);
  coeffs[0] = 2.5;

  auto vel_ifopt = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");
  auto vel_trajopt = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, prob.vars, coeffs, "vel");

  qp_ifopt->addCostSet(vel_ifopt, trajopt_sqp::CostPenaltyType::kSquared);
  qp_trajopt->addCostSet(vel_trajopt, trajopt_sqp::CostPenaltyType::kSquared);

  qp_ifopt->setup();
  qp_trajopt->setup();
  qp_ifopt->convexify();
  qp_trajopt->convexify();

  const Eigen::Index n = qp_ifopt->getNumNLPVars();
  ASSERT_EQ(n, qp_trajopt->getNumNLPVars());

  // Hessians should match
  Eigen::MatrixXd H_ifopt = Eigen::MatrixXd(qp_ifopt->getHessian().block(0, 0, n, n));
  Eigen::MatrixXd H_trajopt = Eigen::MatrixXd(qp_trajopt->getHessian().block(0, 0, n, n));

  for (Eigen::Index r = 0; r < n; ++r)
    for (Eigen::Index c = 0; c < n; ++c)
      EXPECT_NEAR(H_ifopt(r, c), H_trajopt(r, c), 1e-8) << "Hessian mismatch at (" << r << "," << c << ")";

  // NLP gradients should match
  const auto& g_ifopt = qp_ifopt->getGradient();
  const auto& g_trajopt = qp_trajopt->getGradient();
  for (Eigen::Index i = 0; i < n; ++i)
    EXPECT_NEAR(g_ifopt[i], g_trajopt[i], 1e-8) << "Gradient mismatch at index " << i;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
