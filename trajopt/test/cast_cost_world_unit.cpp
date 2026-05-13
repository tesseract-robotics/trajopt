#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract/common/types.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/visualization/visualization.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/utils.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>
#include "trajopt_test_utils.hpp"

using namespace trajopt;
using namespace std;
using namespace trajopt_common;
using namespace tesseract::environment;
using namespace tesseract::collision;
using namespace tesseract::kinematics;
using namespace tesseract::visualization;
using namespace tesseract::scene_graph;
using namespace tesseract::geometry;
using namespace tesseract::common;

static const bool plotting = false;

class CastWorldTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot_world.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    gLogLevel = trajopt_common::LevelError;

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    // Next add objects that can be attached/detached to the scene
    auto box = std::make_shared<Box>(1.0, 1.0, 1.0);

    auto visual = std::make_shared<Visual>();
    visual->geometry = box;
    visual->origin = Eigen::Isometry3d::Identity();

    auto collision = std::make_shared<Collision>();
    collision->geometry = box;
    collision->origin = Eigen::Isometry3d::Identity();

    Link new_link("box_world");
    new_link.visual.push_back(visual);
    new_link.collision.push_back(collision);

    Joint new_joint("box_world-base_link");
    new_joint.type = JointType::FIXED;
    new_joint.parent_link_name = "base_link";
    new_joint.child_link_name = "box_world";

    env_->applyCommand(std::make_shared<AddLinkCommand>(new_link, new_joint));

    // TODO: Need to add method to environment to disable collision and hid objects
  }
};

void runTest(const Environment::Ptr& env, const Visualization::Ptr& plotter, bool use_multi_threaded)
{
  CONSOLE_BRIDGE_logDebug("CastWorldTest, boxes");

  const Json::Value root = readJsonFile(std::string(TRAJOPT_DATA_DIR) + "/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  //  plotter_->plotScene();

  const TrajOptProb::Ptr prob = ConstructProblem(root, env);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  const tesseract::scene_graph::StateSolver::UPtr state_solver = prob->GetEnv()->getStateSolver();
  const ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();

  manager->setActiveCollisionObjects(prob->GetKin()->getActiveLinkNames());
  manager->setDefaultCollisionMargin(0);

  tesseract::collision::CollisionCheckConfig config;
  config.type = tesseract::collision::CollisionEvaluatorType::CONTINUOUS;
  bool found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), prob->GetInitTraj(), config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP::Ptr opt;
  if (use_multi_threaded)
  {
    opt = std::make_shared<sco::BasicTrustRegionSQPMultiThreaded>(prob);
    opt->getParameters().num_threads = 5;
  }
  else
  {
    opt = std::make_shared<sco::BasicTrustRegionSQP>(prob);
  }

  if (plotting)
    opt->addCallback(PlotCallback(plotter));
  opt->initialize(trajToDblVec(prob->GetInitTraj()));
  opt->optimize();

  if (plotting)
    plotter->clear();

  collisions.clear();
  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(opt->x(), prob->GetVars()), config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastWorldTest, boxes)  // NOLINT
{
  runTest(env_, plotter_, false);
}

TEST_F(CastWorldTest, boxes_multi_threaded)  // NOLINT
{
  runTest(env_, plotter_, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
