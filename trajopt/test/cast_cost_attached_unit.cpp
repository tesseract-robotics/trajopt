#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>

#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
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
  SceneGraphPtr scene_graph_;             /**< Scene Graph */
  SRDFModel srdf_model_;                  /**< SRDF Model */
  KDLEnvPtr env_;                         /**< Trajopt Basic Environment */
  AllowedCollisionMatrixPtr acm_;         /**< Allowed Collision Matrix */
  VisualizationPtr plotter_;              /**< Trajopt Plotter */
  ForwardKinematicsConstPtrMap kin_map_;  /**< A map between manipulator name and kinematics object */
//  tesseract_ros::ROSBasicPlottingPtr plotter_; /**< Trajopt Plotter */

  void SetUp() override
  {
    std::string urdf_file = std::string(DATA_DIR) + "/data/boxbot.urdf";
    std::string srdf_file = std::string(DATA_DIR) + "/data/boxbot.srdf";

    ResourceLocatorFn locator = locateResource;
    scene_graph_ = parseURDF(urdf_file, locator);
    EXPECT_TRUE(scene_graph_ != nullptr);
    EXPECT_TRUE(srdf_model_.initFile(*scene_graph_, srdf_file));

    env_ = KDLEnvPtr(new KDLEnv);
    EXPECT_TRUE(env_ != nullptr);
    EXPECT_TRUE(env_->init(scene_graph_));

    // Add collision detectors
    tesseract_collision_bullet::BulletDiscreteBVHManagerPtr dc(new tesseract_collision_bullet::BulletDiscreteBVHManager());
    tesseract_collision_bullet::BulletCastBVHManagerPtr cc(new tesseract_collision_bullet::BulletCastBVHManager());

    EXPECT_TRUE(env_->setDiscreteContactManager(dc));
    EXPECT_TRUE(env_->setContinuousContactManager(cc));

    // Add Allowed Collision Matrix
    acm_ = getAllowedCollisionMatrix(srdf_model_);
    IsContactAllowedFn fn = std::bind(&CastAttachedTest::defaultIsContactAllowedFn, this, std::placeholders::_1, std::placeholders::_2);
    env_->setIsContactAllowedFn(fn);

    // Generate Kinematics Map
    kin_map_ = createKinematicsMap<KDLFwdKinChain, KDLFwdKinTree>(scene_graph_, srdf_model_);

    gLogLevel = util::LevelDebug;

    // Create plotting tool
//    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));


    // Next add objects that can be attached/detached to the scene
    BoxPtr box(new Box(0.25, 0.25, 0.25));
    VisualPtr visual(new Visual());
    visual->geometry = box;
    visual->origin = Eigen::Isometry3d::Identity();

    CollisionPtr collision(new Collision());
    collision->geometry = box;
    collision->origin = Eigen::Isometry3d::Identity();

    LinkPtr box_attached_link(new Link("box_attached"));
    box_attached_link->visual.push_back(visual);
    box_attached_link->collision.push_back(collision);

    JointPtr box_attached_joint(new Joint("boxbot_link-box_attached"));
    box_attached_joint->parent_link_name = "boxbot_link";
    box_attached_joint->child_link_name = "box_attached";
    box_attached_joint->parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.5, -0.5, 0);

    env_->addLink(box_attached_link, box_attached_joint);
    env_->disableCollision("box_attached");

    LinkPtr box_attached2_link(new Link("box_attached2"));
    box_attached2_link->visual.push_back(visual);
    box_attached2_link->collision.push_back(collision);

    JointPtr box_attached2_joint(new Joint("no_geom_link-box_attached2"));
    box_attached2_joint->parent_link_name = "no_geom_link";
    box_attached2_joint->child_link_name = "box_attached2";
    box_attached2_joint->parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, 0);

    env_->addLink(box_attached2_link, box_attached2_joint);
    env_->disableCollision("box_attached2");
  }

  bool defaultIsContactAllowedFn(const std::string& link_name1, const std::string& link_name2) const
  {
    if (acm_ != nullptr && acm_->isCollisionAllowed(link_name1, link_name2))
      return true;

    return false;
  }
};

TEST_F(CastAttachedTest, LinkWithGeom)
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeom");

  env_->enableCollision("box_attached");

  Json::Value root = readJsonFile(std::string(DATA_DIR)  + "/data/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env_->setState(ipos);

//  plotter_->plotScene();

  TrajOptProbPtr prob = ConstructProblem(root, env_, kin_map_);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  ContinuousContactManagerPtr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMapPtr adjacency_map = std::make_shared<AdjacencyMap>(scene_graph_,
                                                                 prob->GetKin()->getActiveLinkNames(),
                                                                 prob->GetEnv()->getLinkNames(),
                                                                 prob->GetEnv()->getState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), adjacency_map->getActiveLinkNames(), prob->GetInitTraj(), collisions);

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
  found = checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), adjacency_map->getActiveLinkNames(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastAttachedTest, LinkWithoutGeom)
{
  CONSOLE_BRIDGE_logDebug("CastAttachedTest, LinkWithGeom");

  env_->enableCollision("box_attached2");

  Json::Value root = readJsonFile(std::string(DATA_DIR)  + "/data/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env_->setState(ipos);

//  plotter_->plotScene();

  TrajOptProbPtr prob = ConstructProblem(root, env_, kin_map_);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  ContinuousContactManagerPtr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMapPtr adjacency_map = std::make_shared<AdjacencyMap>(scene_graph_,
                                                                 prob->GetKin()->getActiveLinkNames(),
                                                                 prob->GetEnv()->getLinkNames(),
                                                                 prob->GetEnv()->getState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), adjacency_map->getActiveLinkNames(), prob->GetInitTraj(), collisions);

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
  found = checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), adjacency_map->getActiveLinkNames(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

//  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
