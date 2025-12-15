/**
 * @file cast_cost_attached_unit.cpp
 * @brief A cast collision cost attached collision object unit test
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
#include <ctime>
#include <gtest/gtest.h>
#include <console_bridge/console.h>
#include <OsqpEigen/OsqpEigen.h>
#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/modify_allowed_collisions_command.h>
#include <tesseract_environment/commands/change_link_collision_enabled_command.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/octree.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_types.h>

#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

using namespace trajopt_ifopt;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

class CastAttachedTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));

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

    tesseract_common::AllowedCollisionMatrix modify_acm;
    modify_acm.addAllowedCollision("box_attached", "boxbot_link", "Adjacent");

    env->applyCommand(std::make_shared<AddLinkCommand>(box_attached_link, box_attached_joint));
    env->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached", false));
    env->applyCommand(std::make_shared<ModifyAllowedCollisionsCommand>(modify_acm, ModifyAllowedCollisionsType::ADD));

    Link box_attached2_link("box_attached2");
    box_attached2_link.visual.push_back(visual);
    box_attached2_link.collision.push_back(collision);

    Joint box_attached2_joint("no_geom_link-box_attached2");
    box_attached2_joint.type = JointType::FIXED;
    box_attached2_joint.parent_link_name = "no_geom_link";
    box_attached2_joint.child_link_name = "box_attached2";
    box_attached2_joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, 0);

    tesseract_common::AllowedCollisionMatrix modify_acm2;
    modify_acm2.addAllowedCollision("box_attached2", "boxbot_link", "Adjacent");

    env->applyCommand(std::make_shared<AddLinkCommand>(box_attached2_link, box_attached2_joint));
    env->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached2", false));
    env->applyCommand(std::make_shared<ModifyAllowedCollisionsCommand>(modify_acm2, ModifyAllowedCollisionsType::ADD));
  }
};

void runCastAttachedLinkWithGeomTest(const trajopt_sqp::QPProblem::Ptr& qp_problem,
                                     const Environment::Ptr& env,
                                     bool fixed_size)
{
  env->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached", true));

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  const tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  const ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  const tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");
  const std::vector<trajopt_ifopt::Bounds> bounds = trajopt_ifopt::toBounds(manip->getLimits().joint_limits);

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMargin(0);

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_0"));
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  {
    nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_1"));
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  {
    nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_2"));
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  qp_problem->addVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes)));

  // Step 3: Setup collision
  const double margin_coeff = 10;
  const double margin = 0.02;
  trajopt_common::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_check_config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  trajopt_collision_config.collision_check_config.longest_valid_segment_length = 0.05;
  trajopt_collision_config.collision_margin_buffer = 0.5;

  // 4) Add constraints
  {  // Fix start position
    const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[0], vars[0], coeffs);
    qp_problem->addConstraintSet(cnt);
  }

  {  // Fix end position
    const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[2], vars[2], coeffs);
    qp_problem->addConstraintSet(cnt);
  }

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  std::array<bool, 2> vars_fixed{ true, false };
  for (std::size_t i = 1; i < vars.size(); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
        collision_cache, manip, env, trajopt_collision_config);

    const std::array<std::shared_ptr<const Var>, 2> position_vars{ vars[i - 1], vars[i] };
    if (fixed_size)
    {
      auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
          collision_evaluator, position_vars, vars_fixed[0], vars_fixed[1], 3);
      qp_problem->addConstraintSet(cnt);
    }
    else
    {
      auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintD>(
          collision_evaluator, position_vars, vars_fixed[0], vars_fixed[1]);
      qp_problem->addConstraintSet(cnt);
    }

    vars_fixed = { false, true };
  }

  auto vel_target = Eigen::VectorXd::Zero(2);
  auto vel_coeff = Eigen::VectorXd::Ones(2);
  qp_problem->addCostSet(std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, vars, vel_coeff),
                         trajopt_sqp::CostPenaltyType::SQUARED);

  qp_problem->setup();
  qp_problem->print();

  // 5) Setup solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_->settings()->setVerbosity(false);
  qp_solver->solver_->settings()->setWarmStart(true);
  qp_solver->solver_->settings()->setPolish(true);
  qp_solver->solver_->settings()->setAdaptiveRho(false);
  qp_solver->solver_->settings()->setMaxIteration(8192);
  qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_->settings()->setRelativeTolerance(1e-6);

  // 6) solve
  solver.verbose = true;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();
  std::cout << x.transpose() << '\n';

  EXPECT_TRUE(solver.getStatus() == trajopt_sqp::SQPStatus::NLP_CONVERGED);

  tesseract_common::TrajArray inputs(3, 2);
  inputs << -1.9, 0, 0, 1.9, 1.9, 3.8;
  const Eigen::Map<tesseract_common::TrajArray> results(x.data(), 3, 2);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found = checkTrajectory(collisions, *manager, *state_solver, manip->getJointNames(), inputs, config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, manip->getJointNames(), results, config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

void runCastAttachedLinkWithoutGeomTest(const trajopt_sqp::QPProblem::Ptr& qp_problem,
                                        const Environment::Ptr& env,
                                        bool fixed_size)
{
  env->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached2", true));

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  const tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  const ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  const tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");
  const std::vector<trajopt_ifopt::Bounds> bounds = trajopt_ifopt::toBounds(manip->getLimits().joint_limits);

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMargin(0);

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_0"));
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  {
    nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_1"));
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  {
    nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_2"));
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  qp_problem->addVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes)));

  // Step 3: Setup collision
  const double margin_coeff = 10;
  const double margin = 0.02;
  trajopt_common::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_check_config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  trajopt_collision_config.collision_check_config.longest_valid_segment_length = 0.05;
  trajopt_collision_config.collision_margin_buffer = 0.01;

  // 4) Add constraints
  {  // Fix start position
    const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[0], vars[0], coeffs);
    qp_problem->addConstraintSet(cnt);
  }

  {  // Fix end position
    const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[2], vars[2], coeffs);
    qp_problem->addConstraintSet(cnt);
  }

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  std::array<bool, 2> vars_fixed{ true, false };
  for (std::size_t i = 1; i < vars.size(); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
        collision_cache, manip, env, trajopt_collision_config);

    const std::array<std::shared_ptr<const Var>, 2> position_vars{ vars[i - 1], vars[i] };

    if (fixed_size)
    {
      auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
          collision_evaluator, position_vars, vars_fixed[0], vars_fixed[1], 2);
      qp_problem->addConstraintSet(cnt);
    }
    else
    {
      auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintD>(
          collision_evaluator, position_vars, vars_fixed[0], vars_fixed[1]);
      qp_problem->addConstraintSet(cnt);
    }

    vars_fixed = { false, true };
  }

  qp_problem->setup();
  qp_problem->print();

  // 5) Setup solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_->settings()->setVerbosity(false);
  qp_solver->solver_->settings()->setWarmStart(true);
  qp_solver->solver_->settings()->setPolish(true);
  qp_solver->solver_->settings()->setAdaptiveRho(false);
  qp_solver->solver_->settings()->setMaxIteration(8192);
  qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_->settings()->setRelativeTolerance(1e-6);

  // 6) solve
  solver.verbose = false;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();
  std::cout << x.transpose() << '\n';

  EXPECT_TRUE(solver.getStatus() == trajopt_sqp::SQPStatus::NLP_CONVERGED);

  tesseract_common::TrajArray inputs(3, 2);
  inputs << -1.9, 0, 0, 1.9, 1.9, 3.8;
  const Eigen::Map<tesseract_common::TrajArray> results(x.data(), 3, 2);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found = checkTrajectory(collisions, *manager, *state_solver, manip->getJointNames(), inputs, config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, manip->getJointNames(), results, config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastAttachedTest, LinkWithGeomIfoptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeomIfoptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runCastAttachedLinkWithGeomTest(qp_problem, env, true);
}

TEST_F(CastAttachedTest, LinkWithGeomTrajOptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeomTrajOptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runCastAttachedLinkWithGeomTest(qp_problem, env, false);
}

TEST_F(CastAttachedTest, LinkWithoutGeomIfoptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithoutGeomIfoptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runCastAttachedLinkWithoutGeomTest(qp_problem, env, true);
}

TEST_F(CastAttachedTest, LinkWithoutGeomTrajOptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithoutGeomTrajOptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runCastAttachedLinkWithoutGeomTest(qp_problem, env, false);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
