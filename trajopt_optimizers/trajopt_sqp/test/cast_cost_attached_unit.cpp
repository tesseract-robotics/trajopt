/**
 * @file cast_cost_attached_unit.cpp
 * @brief A cast collision cost attached collision object unit test
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
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include "test_suite_utils.hpp"

using namespace trajopt_ifopt;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

class CastAttachedTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    // Next add objects that can be attached/detached to the scene
    auto box = std::make_shared<Box>(0.25, 0.25, 0.25);
    auto visual = std::make_shared<Visual>();
    visual->geometry = box;
    visual->origin = Eigen::Isometry3d::Identity();

    auto collision = std::make_shared<Collision>();
    collision->geometry = box;
    collision->origin = Eigen::Isometry3d::Identity();

    Link box_attached_link("box_attached");
    box_attached_link.visual.push_back(visual);
    box_attached_link.collision.push_back(collision);

    Joint box_attached_joint("boxbot_link-box_attached");
    box_attached_joint.type = JointType::FIXED;
    box_attached_joint.parent_link_name = "boxbot_link";
    box_attached_joint.child_link_name = "box_attached";
    box_attached_joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.5, -0.5, 0);

    env->addLink(box_attached_link, box_attached_joint);
    env->setLinkCollisionEnabled("box_attached", false);
    env->addAllowedCollision("box_attached", "boxbot_link", "Adjacent");

    Link box_attached2_link("box_attached2");
    box_attached2_link.visual.push_back(visual);
    box_attached2_link.collision.push_back(collision);

    Joint box_attached2_joint("no_geom_link-box_attached2");
    box_attached2_joint.type = JointType::FIXED;
    box_attached2_joint.parent_link_name = "no_geom_link";
    box_attached2_joint.child_link_name = "box_attached2";
    box_attached2_joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, 0);

    env->addLink(box_attached2_link, box_attached2_joint);
    env->setLinkCollisionEnabled("box_attached2", false);
    env->addAllowedCollision("box_attached2", "boxbot_link", "Adjacent");
  }
};

void runCastAttachedLinkWithGeomTest(trajopt_sqp::QPProblem::Ptr qp_problem, Environment::Ptr env)
{
  env->setLinkCollisionEnabled("box_attached", true);

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  // 3) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_1");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_2");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }

  // Step 3: Setup collision
  auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  double margin_coeff = 20;
  double margin = 0.3;
  auto trajopt_collision_config = std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(margin, margin_coeff);
  trajopt_collision_config->collision_margin_buffer = 0.05;

  // 4) Add constraints
  {  // Fix start position
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[0] };
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[0], fixed_vars);
    qp_problem->addConstraintSet(cnt);
  }

  {  // Fix end position
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[2] };
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[2], fixed_vars);
    qp_problem->addConstraintSet(cnt);
  }

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  for (std::size_t i = 1; i < (vars.size() - 1); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
        collision_cache, kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_config);

    std::array<JointPosition::ConstPtr, 3> position_vars{ vars[i - 1], vars[i], vars[i + 1] };
    auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintIfopt>(
        collision_evaluator,
        trajopt_ifopt::ContinuousCombineCollisionData(CombineCollisionDataMethod::WEIGHTED_AVERAGE),
        position_vars);
    qp_problem->addConstraintSet(cnt);
  }

  qp_problem->setup();
  qp_problem->print();

  // 5) Setup solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(true);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // 6) solve
  solver.verbose = true;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();
  std::cout << x.transpose() << std::endl;

  EXPECT_TRUE(solver.getStatus() == trajopt_sqp::SQPStatus::NLP_CONVERGED);

  tesseract_common::TrajArray inputs(3, 2);
  inputs << -1.9, 0, 0, 1.9, 1.9, 3.8;
  Eigen::Map<tesseract_common::TrajArray> results(x.data(), 3, 2);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found =
      checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), inputs, config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), results, config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

void runCastAttachedLinkWithoutGeomTest(trajopt_sqp::QPProblem::Ptr qp_problem, Environment::Ptr env)
{
  env->setLinkCollisionEnabled("box_attached2", true);

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  // 3) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_1");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_2");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }

  // Step 3: Setup collision
  auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  double margin_coeff = 10;
  double margin = 0.02;
  auto trajopt_collision_config = std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(margin, margin_coeff);
  trajopt_collision_config->collision_margin_buffer = 0.05;

  // 4) Add constraints
  {  // Fix start position
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[0] };
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[0], fixed_vars);
    qp_problem->addConstraintSet(cnt);
  }

  {  // Fix end position
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[2] };
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[2], fixed_vars);
    qp_problem->addConstraintSet(cnt);
  }

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  for (std::size_t i = 1; i < (vars.size() - 1); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
        collision_cache, kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_config);

    std::array<JointPosition::ConstPtr, 3> position_vars{ vars[i - 1], vars[i], vars[i + 1] };
    auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintIfopt>(
        collision_evaluator,
        trajopt_ifopt::ContinuousCombineCollisionData(CombineCollisionDataMethod::WEIGHTED_AVERAGE),
        position_vars);
    qp_problem->addConstraintSet(cnt);
  }

  qp_problem->setup();
  qp_problem->print();

  // 5) Setup solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(true);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // 6) solve
  solver.verbose = true;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();
  std::cout << x.transpose() << std::endl;

  EXPECT_TRUE(solver.getStatus() == trajopt_sqp::SQPStatus::NLP_CONVERGED);

  tesseract_common::TrajArray inputs(3, 2);
  inputs << -1.9, 0, 0, 1.9, 1.9, 3.8;
  Eigen::Map<tesseract_common::TrajArray> results(x.data(), 3, 2);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found =
      checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), inputs, config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), results, config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastAttachedTest, LinkWithGeomIfoptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeomIfoptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runCastAttachedLinkWithGeomTest(qp_problem, env);
}

TEST_F(CastAttachedTest, LinkWithGeomTrajOptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeomTrajOptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runCastAttachedLinkWithGeomTest(qp_problem, env);
}

TEST_F(CastAttachedTest, LinkWithoutGeomIfoptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithoutGeomIfoptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runCastAttachedLinkWithoutGeomTest(qp_problem, env);
}

TEST_F(CastAttachedTest, LinkWithoutGeomTrajOptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithoutGeomTrajOptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runCastAttachedLinkWithoutGeomTest(qp_problem, env);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
