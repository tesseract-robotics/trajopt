/**
 * @file cast_cost_unit.h
 * @brief The casted collision unit test
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
#include <tesseract_common/resource_locator.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/config.hpp>
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

using namespace trajopt_ifopt;
using namespace std;
using namespace trajopt_common;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

class CastTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");
    const ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));

    gLogLevel = trajopt_common::LevelError;
  }
};

TEST_F(CastTest, boxes)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastTest, boxes");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  const tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  const ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  const tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");
  const std::vector<Bounds> bounds = toBounds(manip->getLimits().joint_limits);

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMargin(0);

  collisions.clear();

  // 2) Create the problem
  Problem nlp;

  // 3) Add Variables
  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    nodes.push_back(std::make_unique<Node>("Joint_Position_0"));
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  {
    nodes.push_back(std::make_unique<Node>("Joint_Position_1"));
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  {
    nodes.push_back(std::make_unique<Node>("Joint_Position_2"));
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  nlp.AddVariableSet(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  // Step 3: Setup collision
  const double margin_coeff = 1;
  const double margin = 0.02;
  trajopt_common::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_check_config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  trajopt_collision_config.collision_margin_buffer = 0.05;

  // 4) Add constraints
  {  // Fix start position
    const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 1);
    auto cnt = std::make_shared<JointPosConstraint>(positions[0], vars[0], coeffs);
    nlp.AddConstraintSet(cnt);
  }

  {  // Fix end position
    const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 1);
    auto cnt = std::make_shared<JointPosConstraint>(positions[2], vars[2], coeffs);
    nlp.AddConstraintSet(cnt);
  }

  auto collision_cache = std::make_shared<CollisionCache>(100);
  std::array<bool, 2> vars_fixed{ true, false };
  for (std::size_t i = 1; i < (vars.size()); ++i)
  {
    auto collision_evaluator =
        std::make_shared<LVSContinuousCollisionEvaluator>(collision_cache, manip, env, trajopt_collision_config);

    const std::array<std::shared_ptr<const Var>, 2> position_vars{ vars[i - 1], vars[i] };
    auto cnt = std::make_shared<ContinuousCollisionConstraint>(
        collision_evaluator, position_vars, vars_fixed[0], vars_fixed[1], 1, true);
    nlp.AddConstraintSet(cnt);
    if (i == vars.size() - 1)
      vars_fixed = { false, true };
    else
      vars_fixed = { false, false };
  }

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints().toDense() << '\n';

  // 5) choose solver and options
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("derivative_test", "first-order");
  ipopt.SetOption("linear_solver", "mumps");
  //  ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  ipopt.SetOption("jacobian_approximation", "exact");
  ipopt.SetOption("print_level", 5);
  ipopt.SetOption("nlp_scaling_method", "gradient-based");

  // 6) solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.transpose() << '\n';

  EXPECT_TRUE(ipopt.GetReturnStatus() == 0);

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
