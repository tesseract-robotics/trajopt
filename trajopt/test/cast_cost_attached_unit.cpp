#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/commands.h>
#include <tesseract_environment/utils.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/octree.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
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
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

static const bool plotting = false;

class CastAttachedTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    gLogLevel = trajopt_common::LevelError;

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

    env_->applyCommand(std::make_shared<AddLinkCommand>(box_attached_link, box_attached_joint));
    env_->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached", false));
    env_->applyCommand(std::make_shared<ModifyAllowedCollisionsCommand>(modify_acm, ModifyAllowedCollisionsType::ADD));

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

    env_->applyCommand(std::make_shared<AddLinkCommand>(box_attached2_link, box_attached2_joint));
    env_->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached2", false));
    env_->applyCommand(std::make_shared<ModifyAllowedCollisionsCommand>(modify_acm2, ModifyAllowedCollisionsType::ADD));
  }
};

void runLinkWithGeomTest(const Environment::Ptr& env, const Visualization::Ptr& plotter, bool use_multi_threaded)
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeom");

  env->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached", true));

  Json::Value root = readJsonFile(std::string(TRAJOPT_DATA_DIR) + "/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  //  plotter_->plotScene();

  TrajOptProb::Ptr prob = ConstructProblem(root, env);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  tesseract_scene_graph::StateSolver::UPtr state_solver = prob->GetEnv()->getStateSolver();
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();

  manager->setActiveCollisionObjects(prob->GetKin()->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
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

void runLinkWithoutGeomTest(const Environment::Ptr& env, const Visualization::Ptr& plotter, bool use_multi_threaded)
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeom");

  env->applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>("box_attached2", true));

  Json::Value root = readJsonFile(std::string(TRAJOPT_DATA_DIR) + "/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  //  plotter_->plotScene();

  TrajOptProb::Ptr prob = ConstructProblem(root, env);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  tesseract_scene_graph::StateSolver::UPtr state_solver = prob->GetEnv()->getStateSolver();
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();

  manager->setActiveCollisionObjects(prob->GetKin()->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
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

TEST_F(CastAttachedTest, LinkWithGeom)  // NOLINT
{
  runLinkWithGeomTest(env_, plotter_, false);
}

TEST_F(CastAttachedTest, LinkWithGeomMultiThreaded)  // NOLINT
{
  runLinkWithGeomTest(env_, plotter_, true);
}

TEST_F(CastAttachedTest, LinkWithoutGeom)  // NOLINT
{
  runLinkWithoutGeomTest(env_, plotter_, false);
}

TEST_F(CastAttachedTest, LinkWithoutGeomMultiThreaded)  // NOLINT
{
  runLinkWithoutGeomTest(env_, plotter_, false);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
