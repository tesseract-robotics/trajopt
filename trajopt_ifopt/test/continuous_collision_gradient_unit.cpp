/**
 * @file continuous_collision_gradient_unit.cpp
 * @brief  Unit test for the different methods for combining the gradients
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date June 1, 2021
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/config.hpp>
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
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

class ContinuousCollisionGradientTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));

    gLogLevel = trajopt_common::LevelError;
  }
};

void runContinuousGradientTest(const Environment::Ptr& env, double coeff)
{
  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -1.9;
  ipos["spherebot_y_joint"] = 0;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  const tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  const ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  const tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMargin(0);

  collisions.clear();

  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_1");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_2");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // Step 3: Setup collision
  const double margin_coeff = coeff;
  const double margin = 0.02;
  trajopt_common::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_check_config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  trajopt_collision_config.collision_margin_buffer = 0.05;

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  for (std::size_t i = 1; i < vars.size(); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
        collision_cache, manip, env, trajopt_collision_config);

    const std::array<JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };
    auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
        collision_evaluator, position_vars, false, false, 3);
    nlp.AddConstraintSet(cnt);
  }

  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints().toDense() << '\n';
  std::vector<double> init_vals{ -1.9, 0, 0, 1.9, 1.9, 3.8 };
  const trajopt_ifopt::SparseMatrix num_jac_block =
      trajopt_ifopt::calcNumericalConstraintGradient(init_vals.data(), nlp, 1e-8);
  std::cout << "Numerical Jacobian: \n" << num_jac_block.toDense() << '\n';
}

TEST_F(ContinuousCollisionGradientTest, ContinuousCollisionGradientTest)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ContinuousCollisionGradientTest, ContinuousCollisionGradientTest");
  runContinuousGradientTest(env, 1);
  runContinuousGradientTest(env, 10);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
