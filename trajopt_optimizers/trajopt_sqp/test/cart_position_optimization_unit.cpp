/**
 * @file cart_position_optimization_unit.cpp
 * @brief This is example is made to pair with cart_position_example.cpp.
 * This is the same motion planning problem in the trajopt_sco framework
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

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

const bool DEBUG = false;

class CartPositionOptimization : public testing::TestWithParam<const char*>
{
public:
  tesseract_environment::Environment::Ptr env;

  void SetUp() override
  {
    if (DEBUG)
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    else
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    // 1)  Load Robot
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    const tesseract_common::ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    env = std::make_shared<tesseract_environment::Environment>();
    env->init(urdf_file, srdf_file, locator);
  }
};

void runCartPositionOptimization(const trajopt_sqp::QPProblem::Ptr& qp_problem,
                                 const tesseract_environment::Environment::Ptr& env)
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

  // Extract necessary kinematic information
  const tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("right_arm");
  const tesseract_common::KinematicLimits limits = manip->getLimits();
  const std::vector<ifopt::Bounds> bounds = trajopt_ifopt::toBounds(limits.joint_limits);

  // Get target position
  Eigen::VectorXd start_pos(manip->numJoints());
  start_pos << 0.0, 0, 0, -1.0, 0, -1, -0.00;
  if (DEBUG)
    std::cout << "Joint Limits:\n" << manip->getLimits().joint_limits.transpose() << '\n';

  const Eigen::VectorXd joint_target = start_pos;
  Eigen::Isometry3d target_pose = manip->calcFwdKin(joint_target).at("r_gripper_tool_frame");

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  for (int ind = 0; ind < 1; ind++)
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_" + std::to_string(ind));
    auto zero = Eigen::VectorXd::Zero(7);
    vars.push_back(node->addVar("position", manip->getJointNames(), zero, bounds));
    nodes.push_back(std::move(node));
  }
  qp_problem->addVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes)));

  // 4) Add constraints
  for (const auto& var : vars)
  {
    const trajopt_ifopt::CartPosInfo cart_info(
        manip, "r_gripper_tool_frame", "base_footprint", Eigen::Isometry3d::Identity(), target_pose);
    auto cnt = std::make_shared<trajopt_ifopt::CartPosConstraint>(cart_info, var);
    qp_problem->addConstraintSet(cnt);
  }

  qp_problem->setup();

  // solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();

  auto optimized_pose = manip->calcFwdKin(x).at("r_gripper_tool_frame");
  EXPECT_TRUE(target_pose.translation().isApprox(optimized_pose.translation(), 1e-4));
  const Eigen::Quaterniond target_q(target_pose.rotation());
  const Eigen::Quaterniond optimized_q(optimized_pose.rotation());
  EXPECT_TRUE(target_q.isApprox(optimized_q, 1e-5));

  if (DEBUG)
  {
    std::cout << x.transpose() << '\n';
    qp_problem->print();
  }
}

/** @brief Applies a cartesian position constraint and solves the ifopt problem with trajopt_sqp */
TEST_F(CartPositionOptimization, cart_position_optimization_ifopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartPositionOptimization, cart_position_optimization_trajopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runCartPositionOptimization(qp_problem, env);
}

/** @brief Applies a cartesian position constraint and solves the ifopt problem with trajopt_sqp */
TEST_F(CartPositionOptimization, cart_position_optimization_trajopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartPositionOptimization, cart_position_optimization_ifopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runCartPositionOptimization(qp_problem, env);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
