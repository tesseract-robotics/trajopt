/**
 * @file simple_collision_unit.cpp
 * @brief A simple collision unit test which matches exactly a unit test in trajopt
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
#include <tesseract_common/stopwatch.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_types.h>

#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

using namespace trajopt_ifopt;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

class SimpleCollisionTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.srdf");
    const ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));
  }
};

void runSimpleCollisionTest(const trajopt_sqp::QPProblem::Ptr& qp_problem, const Environment::Ptr& env)
{
  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -0.75;
  ipos["spherebot_y_joint"] = 0.75;
  env->setState(ipos);

  //  plotter_->plotScene();

  std::vector<ContactResultMap> collisions;
  const tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  const DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
  const tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");
  const std::vector<Bounds> bounds = toBounds(manip->getLimits().joint_limits);

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMargin(0);

  collisions.clear();

  // 3) Add Variables
  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    auto node = std::make_unique<Node>("Joint_Position_0");
    Eigen::VectorXd pos(2);
    pos << -0.75, 0.75;
    positions.push_back(pos);
    auto var = node->addVar("position", manip->getJointNames(), pos, bounds);
    vars.push_back(var);
    nodes.push_back(std::move(node));
  }

  qp_problem->addVariableSet(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  // Step 3: Setup collision
  trajopt_common::TrajOptCollisionConfig trajopt_collision_cnt_config(0.2, 1);
  trajopt_collision_cnt_config.collision_margin_buffer = 0.5;
  trajopt_collision_cnt_config.collision_check_config.longest_valid_segment_length = 0.05;

  const DiscreteCollisionEvaluator::Ptr collision_cnt_evaluator =
      std::make_shared<SingleTimestepCollisionEvaluator>(manip, env, trajopt_collision_cnt_config);
  auto collision_cnt = std::make_shared<DiscreteCollisionConstraintD>(collision_cnt_evaluator, vars[0]);
  qp_problem->addConstraintSet(collision_cnt);

  trajopt_common::TrajOptCollisionConfig trajopt_collision_cost_config(0.3, 10);
  trajopt_collision_cost_config.collision_margin_buffer = 0.5;
  trajopt_collision_cost_config.collision_check_config.longest_valid_segment_length = 0.05;

  const DiscreteCollisionEvaluator::Ptr collision_cost_evaluator =
      std::make_shared<SingleTimestepCollisionEvaluator>(manip, env, trajopt_collision_cost_config);
  auto collision_cost = std::make_shared<DiscreteCollisionConstraintD>(collision_cost_evaluator, vars[0]);
  qp_problem->addCostSet(collision_cost, trajopt_sqp::CostPenaltyType::kHinge);

  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(2, 1);
  auto jp_cost = std::make_shared<JointPosConstraint>(Eigen::Vector2d(0, 0), vars[0], coeffs);
  qp_problem->addCostSet(jp_cost, trajopt_sqp::CostPenaltyType::kSquared);

  qp_problem->setup();
  qp_problem->print();

  // 5) choose solver and options
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

  tesseract_common::Stopwatch stopwatch;
  stopwatch.start();
  solver.solve(qp_problem);
  stopwatch.stop();
  CONSOLE_BRIDGE_logError("Test took %f seconds.", stopwatch.elapsedSeconds());

  Eigen::VectorXd x = qp_problem->getVariableValues();

  std::cout << x.transpose() << '\n';

  tesseract_common::TrajArray inputs(1, 2);
  inputs << -0.75, 0.75;
  const Eigen::Map<tesseract_common::TrajArray> results(x.data(), 1, 2);

  bool found = checkTrajectory(collisions,
                               *manager,
                               *state_solver,
                               manip->getJointNames(),
                               inputs,
                               trajopt_collision_cnt_config.collision_check_config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions,
                          *manager,
                          *state_solver,
                          manip->getJointNames(),
                          results,
                          trajopt_collision_cnt_config.collision_check_config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

// TEST_F(SimpleCollisionTest, spheres_ifopt_problem)  // NOLINT
//{
//  CONSOLE_BRIDGE_logDebug("SimpleCollisionTest, spheres_ifopt_problem");
//  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
//  runSimpleCollisionTest(qp_problem, env);
//}

TEST_F(SimpleCollisionTest, spheres_trajopt_problem)  // NOLINT
{
  /**
   * @todo This test should produce the same table for cost and constraints but they do not.
   * Though this is the same as the older trajopt, it should be corrected. I believe there
   * is something different in how we calculate the merit for constraints compared to costs.
   * Constraints use violation versus cost uses the error.
   */
  CONSOLE_BRIDGE_logDebug("SimpleCollisionTest, spheres_trajopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runSimpleCollisionTest(qp_problem, env);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
