#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <octomap/Pointcloud.h>
#include <octomap/OcTree.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/commands.h>
#include <tesseract_environment/utils.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

static bool plotting = false;

class CastOctomapTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot_world.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;

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

    env_->applyCommand(std::make_shared<AddLinkCommand>(new_link, new_joint));
  }
};

TEST_F(CastOctomapTest, boxes)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastOctomapTest, boxes");

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env_->setState(ipos);

  //  plotter_->plotScene();

  TrajOptProb::Ptr prob = ConstructProblem(root, env_);
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

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
    opt.addCallback(PlotCallback(*prob, plotter_));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting)
    plotter_->clear();

  collisions.clear();
  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()), config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
