#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <octomap/Pointcloud.h>
#include <octomap/OcTree.h>
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
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

static bool plotting = false;

class CastOctomapTest : public testing::TestWithParam<const char*>
{
public:
  Tesseract::Ptr tesseract_ = std::make_shared<Tesseract>(); /**< Tesseract */
  Visualization::Ptr plotter_;                               /**< Trajopt Plotter */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot_world.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(tesseract_->init(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    octomap::Pointcloud point_cloud;
    double delta = 0.05;
    int length = static_cast<int>(1 / delta);

    for (int x = 0; x < length; ++x)
      for (int y = 0; y < length; ++y)
        for (int z = 0; z < length; ++z)
          point_cloud.push_back(-0.5f + static_cast<float>(x * delta),
                                -0.5f + static_cast<float>(y * delta),
                                -0.5f + static_cast<float>(z * delta));

    std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(2 * delta);
    octree->insertPointCloud(point_cloud, octomap::point3d(0, 0, 0));

    // Next add objects that can be attached/detached to the scene
    Octree::Ptr coll_octree = std::make_shared<Octree>(octree, Octree::SubType::BOX);
    Box::Ptr vis_box(new Box(1.0, 1.0, 1.0));

    Visual::Ptr visual(new Visual());
    visual->geometry = vis_box;
    visual->origin = Eigen::Isometry3d::Identity();

    Collision::Ptr collision(new Collision());
    collision->geometry = coll_octree;
    collision->origin = Eigen::Isometry3d::Identity();

    Link new_link("octomap_attached");
    new_link.visual.push_back(visual);
    new_link.collision.push_back(collision);

    Joint new_joint("base_link-octomap_attached");
    new_joint.parent_link_name = "base_link";
    new_joint.child_link_name = "octomap_attached";

    tesseract_->getEnvironment()->addLink(new_link, new_joint);
  }
};

TEST_F(CastOctomapTest, boxes)
{
  CONSOLE_BRIDGE_logDebug("CastOctomapTest, boxes");

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
