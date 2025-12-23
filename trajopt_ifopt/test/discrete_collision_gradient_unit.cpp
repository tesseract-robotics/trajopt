/**
 * @file continuous_collision_gradient_unit.cpp
 * @brief Unit test for the different methods for combining the gradients
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date June 1, 2021
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <trajopt_common/collision_types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

using namespace trajopt_ifopt;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

class DiscreteCollisionGradientTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */

  void SetUp() override
  {
    std::filesystem::path const urdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.urdf");
    std::filesystem::path const srdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.srdf");

    ResourceLocator::Ptr const locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));
  }
};

void runDiscreteGradientTest(const Environment::Ptr& env, double coeff)
{
  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -0.75;
  ipos["spherebot_y_joint"] = 0.75;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  tesseract_scene_graph::StateSolver::Ptr const state_solver = env->getStateSolver();
  DiscreteContactManager::Ptr const manager = env->getDiscreteContactManager();
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
    pos << -0.75, 0.75;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }

  nlp.addVariableSet(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  // Step 3: Setup collision
  const double margin_coeff = coeff;
  const double margin = 0.2;
  trajopt_common::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_margin_buffer = 0.0;  // 0.05

  auto collision_cache = std::make_shared<CollisionCache>(100);
  auto collision_evaluator =
      std::make_shared<SingleTimestepCollisionEvaluator>(collision_cache, manip, env, trajopt_collision_config);
  auto cnt = std::make_shared<DiscreteCollisionConstraint>(collision_evaluator, vars[0], 3);
  nlp.addConstraintSet(cnt);

  std::cout << "Jacobian: \n" << nlp.getJacobianOfConstraints().toDense() << '\n';

  Jacobian const num_jac_block = calcNumericalConstraintGradient(positions[0].data(), nlp, 1e-8);
  std::cout << "Numerical Jacobian: \n" << num_jac_block.toDense() << '\n';
}

TEST_F(DiscreteCollisionGradientTest, DiscreteCollisionGradientTest)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("DiscreteCollisionGradientTest, DiscreteCollisionGradientTest");
  runDiscreteGradientTest(env, 1);
  runDiscreteGradientTest(env, 10);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
