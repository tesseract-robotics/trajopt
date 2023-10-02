/**
 * @file cast_cost_octomap_unit.cpp
 * @brief A cast collision cost octomap collision object unit test
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
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <octomap/Pointcloud.h>
#include <octomap/OcTree.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/octree.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

using namespace trajopt_ifopt;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

class CastOctomapTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot_world.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    octomap::Pointcloud point_cloud;
    double delta = 0.05;
    auto length = static_cast<int>(1 / delta);

    for (int x = 0; x < length; ++x)
      for (int y = 0; y < length; ++y)
        for (int z = 0; z < length; ++z)
          point_cloud.push_back(-0.5F + static_cast<float>(x * delta),
                                -0.5F + static_cast<float>(y * delta),
                                -0.5F + static_cast<float>(z * delta));

    std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(2 * delta);
    octree->insertPointCloud(point_cloud, octomap::point3d(0, 0, 0));

    // Next add objects that can be attached/detached to the scene
    Octree::Ptr coll_octree = std::make_shared<Octree>(octree, Octree::SubType::BOX);
    auto vis_box = std::make_shared<Box>(1.0, 1.0, 1.0);

    auto visual = std::make_shared<Visual>();
    visual->geometry = vis_box;
    visual->origin = Eigen::Isometry3d::Identity();

    auto collision = std::make_shared<Collision>();
    collision->geometry = coll_octree;
    collision->origin = Eigen::Isometry3d::Identity();

    Link new_link("octomap_attached");
    new_link.visual.push_back(visual);
    new_link.collision.push_back(collision);

    Joint new_joint("base_link-octomap_attached");
    new_joint.type = JointType::FIXED;
    new_joint.parent_link_name = "base_link";
    new_joint.child_link_name = "octomap_attached";

    env->applyCommand(std::make_shared<AddLinkCommand>(new_link, new_joint));
  }
};

void runCastOctomapTest(const trajopt_sqp::QPProblem::Ptr& qp_problem, const Environment::Ptr& env)
{
  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");

  manager->setActiveCollisionObjects(manip->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  // 3) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_1");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_2");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }

  // Step 3: Setup collision
  double margin_coeff = 10;
  double margin = 0.02;
  auto trajopt_collision_config = std::make_shared<trajopt_common::TrajOptCollisionConfig>(margin, margin_coeff);
  trajopt_collision_config->type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  trajopt_collision_config->collision_margin_buffer = 0.05;

  // 4) Add constraints
  {  // Fix start position
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[0] };
    Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[0], fixed_vars, coeffs);
    qp_problem->addConstraintSet(cnt);
  }

  {  // Fix end position
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[2] };
    Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(positions[2], fixed_vars, coeffs);
    qp_problem->addConstraintSet(cnt);
  }

  auto collision_cache = std::make_shared<trajopt_common::CollisionCache>(100);
  std::array<bool, 2> position_vars_fixed{ true, false };
  for (std::size_t i = 1; i < vars.size(); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
        collision_cache, manip, env, trajopt_collision_config);

    std::array<JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };

    auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
        collision_evaluator, position_vars, position_vars_fixed, 3);
    qp_problem->addConstraintSet(cnt);

    position_vars_fixed = { false, true };
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
  bool found = checkTrajectory(collisions, *manager, *state_solver, manip->getJointNames(), inputs, config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, manip->getJointNames(), results, config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastOctomapTest, boxesIfoptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastOctomapTest, boxesIfoptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runCastOctomapTest(qp_problem, env);
}

TEST_F(CastOctomapTest, boxesTrajOptProblem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastOctomapTest, boxesTrajOptProblem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runCastOctomapTest(qp_problem, env);  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
