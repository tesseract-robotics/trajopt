/**
 * @file cart_offset_optimization_unit.cpp
 * @brief Solves for a state held at an offset from the pose another state reaches
 *
 * @author Roelof Oomen
 * @date October 13, 2026
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
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <gtest/gtest.h>

#include <OsqpEigen/OsqpEigen.h>

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <tesseract/common/logging.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#ifdef TRAJOPT_SQP_HAS_PIQP
#include <trajopt_sqp/piqp_solver.h>
#endif

#include <trajopt_ifopt/constraints/cartesian_offset_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

namespace
{
const bool DEBUG = false;
const std::string TOOL{ "r_gripper_tool_frame" };
const std::string BASE{ "base_footprint" };

/** Largest row of a converged constraint [m, rad]: twice the solver's constraint tolerance */
constexpr double SOLVE_TOL = 2e-4;
/** The turn about the tool x axis the reference is seeded with [rad] */
constexpr double SEED_TURN = 0.5;
/** The follower sits this far along the tool z axis of the reference [m] */
constexpr double DISTANCE = -0.02;

trajopt_sqp::QPSolver::Ptr makeOSQPSolver()
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  qp_solver->solver_->settings()->setVerbosity(DEBUG);
  qp_solver->solver_->settings()->setWarmStart(true);
  qp_solver->solver_->settings()->setPolish(true);
  qp_solver->solver_->settings()->setAdaptiveRho(false);
  qp_solver->solver_->settings()->setMaxIteration(8192);
  qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_->settings()->setRelativeTolerance(1e-6);
  return qp_solver;
}

/** The tool pose in base coordinates */
Eigen::Isometry3d toolPose(const tesseract::kinematics::JointGroup& manip, const Eigen::VectorXd& joint_values)
{
  const auto poses = manip.calcFwdKin(joint_values);
  return poses.at(BASE).inverse() * poses.at(TOOL);
}

void runCartOffsetOptimization(const tesseract::environment::Environment::Ptr& env,
                               const trajopt_sqp::QPSolver::Ptr& qp_solver)
{
  const tesseract::kinematics::JointGroup::ConstPtr manip = env->getJointGroup("right_arm");
  const Eigen::Index dof = manip->numJoints();
  const std::vector<trajopt_ifopt::Bounds> bounds = trajopt_ifopt::toBounds(manip->getLimits().joint_limits);
  const std::vector<std::string> joint_names = tesseract::common::toNames(manip->getJointIds());
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

  // Both states are seeded off the nominal pose, so the reference has to move to reach it
  Eigen::VectorXd follower_seed(dof);
  follower_seed << 0.0, 0, 0, -1.0, 0, -1, -0.00;
  const Eigen::Isometry3d nominal_pose = toolPose(*manip, follower_seed) * Eigen::Translation3d(0.05, -0.05, 0.05);

  // The reference is seeded turned about the tool x axis: the last joint turns the tool about that axis
  Eigen::VectorXd reference_seed = follower_seed;
  reference_seed[dof - 1] += SEED_TURN;

  // The follower comes before the reference, as the approach to the last state of a trajectory does
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  auto follower_node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_0");
  const auto follower_var = follower_node->addVar("position", joint_names, follower_seed, bounds);
  nodes.push_back(std::move(follower_node));
  auto reference_node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_1");
  const auto reference_var = reference_node->addVar("position", joint_names, reference_seed, bounds);
  nodes.push_back(std::move(reference_node));
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes));
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);

  // The reference reaches the nominal pose, free to turn about its x axis
  Eigen::VectorXd free_about_x = Eigen::VectorXd::Ones(6);
  free_about_x[3] = 0.0;
  qp_problem->addConstraintSet(std::make_shared<trajopt_ifopt::CartPosConstraint>(
      reference_var,
      free_about_x,
      std::vector<trajopt_ifopt::Bounds>(6, trajopt_ifopt::BoundZero),
      manip,
      TOOL,
      BASE,
      Eigen::Isometry3d::Identity(),
      nominal_pose));

  // A cost holds the reference at its seed, so the turn it keeps is defined by the problem
  qp_problem->addCostSet(std::make_shared<trajopt_ifopt::JointPosConstraint>(
                             reference_seed, reference_var, Eigen::VectorXd::Ones(1), "ReferenceSeed"),
                         trajopt_sqp::CostPenaltyType::kSquared);

  // The follower is held at the offset from the pose the reference reaches
  auto offset_constraint = std::make_shared<trajopt_ifopt::CartOffsetConstraint>(
      reference_var, follower_var, manip, TOOL, Eigen::Isometry3d::Identity(), axis, DISTANCE);
  ASSERT_GT(offset_constraint->calcValues(reference_seed, follower_seed).tail(3).norm(), 0.5 * SEED_TURN);
  qp_problem->addConstraintSet(offset_constraint);

  qp_problem->setup();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  ASSERT_EQ(solver.getStatus(), trajopt_sqp::SQPStatus::kConverged);

  const Eigen::VectorXd x = qp_problem->getVariableValues();
  const Eigen::Isometry3d reference_pose = toolPose(*manip, x.segment(reference_var->getIndex(), dof));
  const Eigen::Isometry3d follower_pose = toolPose(*manip, x.segment(follower_var->getIndex(), dof));

  // The reference is on the nominal pose and kept a turn about its x axis
  const Eigen::VectorXd reference_error = tesseract::common::calcTransformError(nominal_pose, reference_pose);
  EXPECT_LT(reference_error.head(3).cwiseAbs().maxCoeff(), SOLVE_TOL);
  EXPECT_LT(reference_error.tail(2).cwiseAbs().maxCoeff(), SOLVE_TOL);
  EXPECT_GT(std::abs(reference_error[3]), 0.5 * SEED_TURN);

  // The follower sits at the offset in the frame the reference reached, with its orientation
  const Eigen::VectorXd offset_error =
      tesseract::common::calcTransformError(reference_pose * Eigen::Translation3d(DISTANCE * axis), follower_pose);
  EXPECT_LT(offset_error.cwiseAbs().maxCoeff(), SOLVE_TOL);

  // That is not the approach to the nominal pose
  EXPECT_GT((follower_pose.translation() - nominal_pose * (DISTANCE * axis)).norm(), 1e-3);
}
}  // namespace

class CartOffsetOptimization : public testing::Test
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

/** @brief The follower moves onto the approach axis of the pose the reference reaches, with OSQP */
TEST_F(CartOffsetOptimization, follower_tracks_turned_reference)  // NOLINT
{
  runCartOffsetOptimization(env, makeOSQPSolver());
}

#ifdef TRAJOPT_SQP_HAS_PIQP
/** @brief The follower moves onto the approach axis of the pose the reference reaches, with PIQP */
TEST_F(CartOffsetOptimization, follower_tracks_turned_reference_piqp)  // NOLINT
{
  runCartOffsetOptimization(env, std::make_shared<trajopt_sqp::PIQPSolver>());
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
