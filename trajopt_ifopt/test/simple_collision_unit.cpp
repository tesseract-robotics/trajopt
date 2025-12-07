/**
 * @file simple_collision_unit.cpp
 * @brief A simple collision unit test for the discrete constraint
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
#include <tesseract_common/resource_locator.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <trajopt_common/collision_types.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>

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
    std::filesystem::path const urdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.urdf");
    std::filesystem::path const srdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.srdf");
    ResourceLocator::Ptr const locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));
  }
};

TEST_F(SimpleCollisionTest, spheres)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("SimpleCollisionTest, spheres");

  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -0.75;
  ipos["spherebot_y_joint"] = 0.75;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  auto state_solver = env->getStateSolver();
  DiscreteContactManager::Ptr const manager = env->getDiscreteContactManager();
  const tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");
  const std::vector<ifopt::Bounds> bounds = trajopt_ifopt::toBounds(manip->getLimits().joint_limits);

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMargin(0);

  collisions.clear();

  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    nodes.push_back(std::make_unique<trajopt_ifopt::Node>("Joint_Position_0"));
    Eigen::VectorXd pos(2);
    pos << -0.75, 0.75;
    positions.push_back(pos);
    vars.push_back(nodes.back()->addVar("position", manip->getJointNames(), pos, bounds));
  }
  nlp.AddVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes)));

  // Step 3: Setup collision
  const double margin_coeff = 10;
  const double margin = 0.2;
  trajopt_common::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_margin_buffer = 0.05;

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  const trajopt_ifopt::DiscreteCollisionEvaluator::Ptr collision_evaluator =
      std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
          collision_cache, manip, env, trajopt_collision_config);

  auto cnt = std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(collision_evaluator, vars[0], 3, true);
  nlp.AddConstraintSet(cnt);

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints() << '\n';

  auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return cnt->CalcValues(x); };
  trajopt_ifopt::SparseMatrix const num_jac_block =
      trajopt_ifopt::calcForwardNumJac(error_calculator, positions[0], 1e-4);
  std::cout << "Numerical Jacobian: \n" << num_jac_block << '\n';

  // 5) choose solver and options
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("derivative_test", "first-order");
  ipopt.SetOption("linear_solver", "mumps");
  //  ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  ipopt.SetOption("jacobian_approximation", "exact");
  ipopt.SetOption("print_level", 5);

  // 6) solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.transpose() << '\n';  // NOLINT

  EXPECT_TRUE(ipopt.GetReturnStatus() == 0);

  tesseract_common::TrajArray inputs(1, 2);
  inputs << -0.75, 0.75;
  Eigen::Map<tesseract_common::TrajArray> const results(x.data(), 1, 2);

  bool found = checkTrajectory(collisions,
                               *manager,
                               *state_solver,
                               manip->getJointNames(),
                               inputs,
                               trajopt_collision_config.collision_check_config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions,
                          *manager,
                          *state_solver,
                          manip->getJointNames(),
                          results,
                          trajopt_collision_config.collision_check_config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
