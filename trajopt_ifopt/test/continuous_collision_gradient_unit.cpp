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
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_scene_graph/utils.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_test_utils.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

using namespace trajopt_ifopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

class ContinuousCollisionGradientTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                            /**< Plotter */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;
  }
};

void runContinuousGradientTest(const Environment::Ptr& env, double coeff)
{
  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -1.9;
  ipos["spherebot_y_joint"] = 0;
  env->setState(ipos);

  //  plotter_->plotScene();

  std::vector<ContactResultMap> collisions;
  tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

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
  double margin_coeff = coeff;
  double margin = 0.02;
  auto trajopt_collision_config = std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(margin, margin_coeff);
  trajopt_collision_config->collision_margin_buffer = 0.05;

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  for (std::size_t i = 1; i < vars.size(); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
        collision_cache, manip, env, trajopt_collision_config);

    std::array<JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };
    std::array<bool, 2> position_vars_fixed{ false, false };
    auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
        collision_evaluator, position_vars, position_vars_fixed, 3);
    nlp.AddConstraintSet(cnt);
  }

  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints().toDense() << std::endl;
  std::vector<double> init_vals{ -1.9, 0, 0, 1.9, 1.9, 3.8 };
  trajopt_ifopt::SparseMatrix num_jac_block =
      trajopt_ifopt::calcNumericalConstraintGradient(init_vals.data(), nlp, 1e-8);
  std::cout << "Numerical Jacobian: \n" << num_jac_block.toDense() << std::endl;
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
