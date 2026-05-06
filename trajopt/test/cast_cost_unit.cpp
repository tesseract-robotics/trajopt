#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
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
using namespace tesseract::kinematics;
using namespace tesseract::collision;
using namespace tesseract::visualization;
using namespace tesseract::scene_graph;
using namespace tesseract::geometry;
using namespace tesseract::common;

static const bool plotting = false;

class CastTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    gLogLevel = trajopt_common::LevelError;

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));
  }
};

void runTest(const Environment::Ptr& env, const Visualization::Ptr& plotter, bool use_multi_threaded)
{
  CONSOLE_BRIDGE_logDebug("CastTest, boxes");

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

  manager->setActiveCollisionObjects(prob->GetKin()->getActiveLinkIds());
  manager->setDefaultCollisionMargin(0);

  collisions.clear();
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
  std::cout << getTraj(opt->x(), prob->GetVars()) << '\n';

  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(opt->x(), prob->GetVars()), config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastTest, boxes)  // NOLINT
{
  runTest(env_, plotter_, false);
}

TEST_F(CastTest, boxes_multi_threaded)  // NOLINT
{
  runTest(env_, plotter_, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
