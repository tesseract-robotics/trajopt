/**
 * @file cart_axis_optimization_unit.cpp
 * @brief Solves position targets with the cartesian axis cone and align constraints
 *
 * @author Jelle Feringa
 * @date October 1, 2026
 *
 * @copyright Copyright (c) 2026
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
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <gtest/gtest.h>

#include <OsqpEigen/OsqpEigen.h>

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <tesseract/common/logging.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

#include <trajopt_ifopt/constraints/cartesian_axis_align_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_axis_cone_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

namespace
{
const bool DEBUG = false;
const std::string TOOL{ "r_gripper_tool_frame" };
const std::string BASE{ "base_footprint" };

/** Converged position error [m] and axis error [rad]; matches the OSQP absolute tolerance used below. */
constexpr double SOLVE_TOL = 1e-4;

/** The tool position at this configuration is the position target; its tool z axis is the reference axis. */
Eigen::VectorXd referenceConfiguration()
{
  Eigen::VectorXd q(7);
  q << 0.0, 0, 0, -1.0, 0, -1, -0.00;
  return q;
}

enum class AxisConstraint : std::uint8_t
{
  kCone,
  kAlign
};

struct Solution
{
  Eigen::Isometry3d reference_pose;  // tool pose at the reference configuration, base coordinates
  Eigen::Vector3d target_axis;       // base coordinates
  Eigen::Isometry3d optimized_pose;  // tool pose at the solution, base coordinates
  trajopt_sqp::SQPStatus status;
};

/**
 * Position-only CartPos to the reference tool position, plus an axis constraint whose axis is the reference tool z
 * axis tilted by @p tilt [rad]. The solve starts from the zero configuration.
 */
Solution solve(const tesseract::environment::Environment::Ptr& env, AxisConstraint type, double tilt, double half_angle)
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

  const tesseract::kinematics::JointGroup::ConstPtr manip = env->getJointGroup("right_arm");
  const std::vector<trajopt_ifopt::Bounds> bounds = trajopt_ifopt::toBounds(manip->getLimits().joint_limits);

  const auto reference_poses = manip->calcFwdKin(referenceConfiguration());
  const Eigen::Isometry3d reference_pose = reference_poses.at(BASE).inverse() * reference_poses.at(TOOL);
  const Eigen::Vector3d target_axis =
      reference_pose.linear() * (Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()) * Eigen::Vector3d::UnitZ());

  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  auto node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_0");
  auto var = node->addVar(
      "position", tesseract::common::toNames(manip->getJointIds()), Eigen::VectorXd::Zero(manip->numJoints()), bounds);
  nodes.push_back(std::move(node));
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);

  Eigen::VectorXd position_only(6);
  position_only << 1, 1, 1, 0, 0, 0;
  qp_problem->addConstraintSet(std::make_shared<trajopt_ifopt::CartPosConstraint>(
      var,
      position_only,
      std::vector<trajopt_ifopt::Bounds>(6, trajopt_ifopt::BoundZero),
      manip,
      TOOL,
      BASE,
      Eigen::Isometry3d::Identity(),
      reference_pose));

  if (type == AxisConstraint::kCone)
    qp_problem->addConstraintSet(std::make_shared<trajopt_ifopt::CartAxisConeConstraint>(
        var, manip, TOOL, Eigen::Vector3d::UnitZ(), BASE, target_axis, half_angle));
  else
    qp_problem->addConstraintSet(std::make_shared<trajopt_ifopt::CartAxisAlignConstraint>(
        var, manip, TOOL, Eigen::Vector3d::UnitZ(), BASE, target_axis));

  qp_problem->setup();
  solver.verbose = DEBUG;
  solver.solve(qp_problem);

  const auto poses = manip->calcFwdKin(qp_problem->getVariableValues());
  return { reference_pose, target_axis, poses.at(BASE).inverse() * poses.at(TOOL), solver.getStatus() };
}

double angleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

void runCone(const tesseract::environment::Environment::Ptr& env)
{
  // The reference orientation is outside the cone (tilt > half angle); the solution must move onto its boundary.
  constexpr double tilt = 0.6;
  constexpr double half_angle = 0.2;
  const Solution s = solve(env, AxisConstraint::kCone, tilt, half_angle);
  ASSERT_EQ(s.status, trajopt_sqp::SQPStatus::kConverged);

  EXPECT_LT((s.optimized_pose.translation() - s.reference_pose.translation()).norm(), SOLVE_TOL);
  const double angle = angleBetween(s.optimized_pose.linear() * Eigen::Vector3d::UnitZ(), s.target_axis);
  EXPECT_LE(angle, half_angle + SOLVE_TOL);
}

void runAlign(const tesseract::environment::Environment::Ptr& env)
{
  constexpr double tilt = 0.6;
  const Solution s = solve(env, AxisConstraint::kAlign, tilt, 0.0);
  ASSERT_EQ(s.status, trajopt_sqp::SQPStatus::kConverged);

  EXPECT_LT((s.optimized_pose.translation() - s.reference_pose.translation()).norm(), SOLVE_TOL);
  const double angle = angleBetween(s.optimized_pose.linear() * Eigen::Vector3d::UnitZ(), s.target_axis);
  EXPECT_LT(angle, SOLVE_TOL);
}
}  // namespace

class CartAxisOptimization : public testing::Test
{
public:
  tesseract::environment::Environment::Ptr env;

  void SetUp() override
  {
    tesseract::common::getLogger()->set_level(DEBUG ? spdlog::level::debug : spdlog::level::off);
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    env = std::make_shared<tesseract::environment::Environment>();
    ASSERT_TRUE(env->init(urdf_file, srdf_file, locator));
  }
};

/** @brief Position target plus a cone around a tilted axis: the tool axis ends on or inside the cone */
TEST_F(CartAxisOptimization, cone) { runCone(env); }  // NOLINT

/** @brief Position target plus axis alignment to a tilted axis: the tool axis ends aligned */
TEST_F(CartAxisOptimization, align) { runAlign(env); }  // NOLINT

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
