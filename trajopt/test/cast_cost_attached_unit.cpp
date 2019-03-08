#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_visualization/visualization.h>
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

bool plotting = false;

class CastAttachedTest : public testing::TestWithParam<const char*>
{
public:
  ros::NodeHandle nh_;
  SceneGraph scene_graph_;   /**< Scene Graph */
  KDLEnvPtr env_;            /**< Trajopt Basic Environment */
  VisualizationPtr plotter_; /**< Trajopt Plotter */

  void SetUp() override
  {
    std::string urdf_file = std::string(DATA_DIR) + "/data/lbr_iiwa_14_r820.urdf";
    std::string srdf_file = std::string(DATA_DIR) + "/data/lbr_iiwa_14_r820.srdf";

    ResourceLocatorFn locator = locateResource;
    scene_graph_ = parseURDF(urdf_file, locator);
    EXPECT_TRUE(scene_graph_ != nullptr);

    SRDFModel srdf;
    EXPECT_TRUE(srdf.initFile(*scene_graph_, srdf_file));

    env_ = KDLEnvPtr(new KDLEnv);
    EXPECT_TRUE(env_->init(scene_graph_));

    // Create plotting tool
//    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    // Next add objects that can be attached/detached to the scene
    AttachableObjectPtr obj1(new AttachableObject());
    AttachableObjectPtr obj2(new AttachableObject());
    std::shared_ptr<shapes::Box> box(new shapes::Box(0.25, 0.25, 0.25));
    BoxCollisionShapePtr c_box(new BoxCollisionShape(0.25, 0.25, 0.25));
    Eigen::Isometry3d box_pose;

    box_pose.setIdentity();
    box_pose.translation() = Eigen::Vector3d(0.5, -0.5, 0);

    obj1->name = "box_attached";
    obj1->visual.shapes.push_back(box);
    obj1->visual.shape_poses.push_back(box_pose);
    obj1->collision.shapes.push_back(c_box);
    obj1->collision.shape_poses.push_back(box_pose);
    env_->addAttachableObject(obj1);

    std::shared_ptr<shapes::Box> box2(new shapes::Box(0.25, 0.25, 0.25));
    BoxCollisionShapePtr c_box2(new BoxCollisionShape(0.25, 0.25, 0.25));
    Eigen::Isometry3d box_pose2;

    box_pose2.setIdentity();
    box_pose2.translation() = Eigen::Vector3d(0, 0, 0);

    obj2->name = "box_attached2";
    obj2->visual.shapes.push_back(box2);
    obj2->visual.shape_poses.push_back(box_pose2);
    obj2->collision.shapes.push_back(c_box2);
    obj2->collision.shape_poses.push_back(box_pose2);
    env_->addAttachableObject(obj2);

    gLogLevel = util::LevelInfo;
  }
};

TEST_F(CastAttachedTest, LinkWithGeom)
{
  ROS_DEBUG("CastTest, LinkWithGeom");

  AttachedBodyInfo attached_body;
  attached_body.object_name = "box_attached";
  attached_body.parent_link_name = "boxbot_link";

  env_->attachBody(attached_body);

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env_->setState(ipos);

  plotter_->plotScene();

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  ContinuousContactManagerPtr manager = prob->GetEnv()->getContinuousContactManager();
  manager->setActiveCollisionObjects(prob->GetKin()->getLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), prob->GetInitTraj(), collisions);

  EXPECT_TRUE(found);
  ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
    opt.addCallback(PlotCallback(*prob, plotter_));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting)
    plotter_->clear();

  collisions.clear();
  found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastAttachedTest, LinkWithoutGeom)
{
  ROS_DEBUG("CastTest, LinkWithGeom");

  AttachedBodyInfo attached_body;
  attached_body.object_name = "box_attached2";
  attached_body.parent_link_name = "no_geom_link";

  env_->attachBody(attached_body);

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/box_cast_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env_->setState(ipos);

  plotter_->plotScene();

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  ContinuousContactManagerPtr manager = prob->GetEnv()->getContinuousContactManager();
  manager->setActiveCollisionObjects(prob->GetKin()->getLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), prob->GetInitTraj(), collisions);

  EXPECT_TRUE(found);
  ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
    opt.addCallback(PlotCallback(*prob, plotter_));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting)
    plotter_->clear();

  collisions.clear();
  found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_cast_cost_attached_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
