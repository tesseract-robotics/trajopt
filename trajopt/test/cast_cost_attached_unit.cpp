#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>

#include <tesseract/tesseract.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt_utils/clock.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

static bool plotting = false;

class CastAttachedTest : public testing::TestWithParam<const char*>
{
public:
  Tesseract::Ptr tesseract_ = std::make_shared<Tesseract>(); /**< Tesseract */
  Visualization::Ptr plotter_;                               /**< Trajopt Plotter */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(tesseract_->init(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelDebug;

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    // Next add objects that can be attached/detached to the scene
    Box::Ptr box(new Box(0.25, 0.25, 0.25));
    Visual::Ptr visual(new Visual());
    visual->geometry = box;
    visual->origin = Eigen::Isometry3d::Identity();

    Collision::Ptr collision(new Collision());
    collision->geometry = box;
    collision->origin = Eigen::Isometry3d::Identity();

    Link box_attached_link("box_attached");
    box_attached_link.visual.push_back(visual);
    box_attached_link.collision.push_back(collision);

    Joint box_attached_joint("boxbot_link-box_attached");
    box_attached_joint.parent_link_name = "boxbot_link";
    box_attached_joint.child_link_name = "box_attached";
    box_attached_joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.5, -0.5, 0);

    tesseract_->getEnvironment()->addLink(box_attached_link, box_attached_joint);
    tesseract_->getEnvironment()->setLinkCollisionEnabled("box_attached", false);

    Link box_attached2_link("box_attached2");
    box_attached2_link.visual.push_back(visual);
    box_attached2_link.collision.push_back(collision);

    Joint box_attached2_joint("no_geom_link-box_attached2");
    box_attached2_joint.parent_link_name = "no_geom_link";
    box_attached2_joint.child_link_name = "box_attached2";
    box_attached2_joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, 0);

    tesseract_->getEnvironment()->addLink(box_attached2_link, box_attached2_joint);
    tesseract_->getEnvironment()->setLinkCollisionEnabled("box_attached2", false);
  }
};

TEST_F(CastAttachedTest, LinkWithGeom)
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeom");

  tesseract_->getEnvironment()->setLinkCollisionEnabled("box_attached", true);

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  tesseract_->getEnvironment()->setState(ipos);

  //  plotter_->plotScene();

  TrajOptProb::Ptr prob = ConstructProblem(root, tesseract_);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(tesseract_->getEnvironment()->getSceneGraph(),
                                                                   prob->GetKin()->getActiveLinkNames(),
                                                                   prob->GetEnv()->getCurrentState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found =
      checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), prob->GetInitTraj(), collisions);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
    opt.addCallback(PlotCallback(*prob, plotter_));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting)
    plotter_->clear();

  collisions.clear();
  found = checkTrajectory(
      *manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastAttachedTest, LinkWithoutGeom)
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeom");

  tesseract_->getEnvironment()->setLinkCollisionEnabled("box_attached2", true);

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  tesseract_->getEnvironment()->setState(ipos);

  //  plotter_->plotScene();

  TrajOptProb::Ptr prob = ConstructProblem(root, tesseract_);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(tesseract_->getEnvironment()->getSceneGraph(),
                                                                   prob->GetKin()->getActiveLinkNames(),
                                                                   prob->GetEnv()->getCurrentState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found =
      checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), prob->GetInitTraj(), collisions);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
    opt.addCallback(PlotCallback(*prob, plotter_));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting)
    plotter_->clear();

  collisions.clear();
  found = checkTrajectory(
      *manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
