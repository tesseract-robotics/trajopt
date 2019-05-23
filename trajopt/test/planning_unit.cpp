#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <sstream>
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
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

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

bool plotting = false;                                                 /**< Enable plotting */

class PlanningTest : public testing::TestWithParam<const char*>
{
public:
  SceneGraphPtr scene_graph_;                    /**< Scene Graph */
  SRDFModel srdf_model_;                         /**< SRDF Model */
  KDLEnvPtr env_;                                /**< Trajopt Basic Environment */
  VisualizationPtr plotter_;                     /**< Trajopt Plotter */
  void SetUp() override
  {
    std::string urdf_file = std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf";
    std::string srdf_file = std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf";

    ResourceLocatorFn locator = locateResource;
    std::pair<tesseract_scene_graph::SceneGraphPtr, tesseract_scene_graph::SRDFModelPtr> data;
    data = tesseract_scene_graph::createSceneGraphFromFiles(urdf_file, srdf_file, locator);
    EXPECT_TRUE(data.first != nullptr && data.second != nullptr);

    scene_graph_ = data.first;
    srdf_model_ = data.second;

    env_ = KDLEnvPtr(new KDLEnv);
    EXPECT_TRUE(env_ != nullptr);
    EXPECT_TRUE(env_->init(scene_graph_));

    // Register contact manager
    EXPECT_TRUE(env_->registerDiscreteContactManager("bullet", &tesseract_collision_bullet::BulletDiscreteBVHManager::create));
    EXPECT_TRUE(env_->registerContinuousContactManager("bullet", &tesseract_collision_bullet::BulletCastBVHManager::create));

    // Set Active contact manager
    EXPECT_TRUE(env_->setActiveDiscreteContactManager("bullet"));
    EXPECT_TRUE(env_->setActiveContinuousContactManager("bullet"));

    // Create plotting tool
//    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    std::unordered_map<std::string, double> ipos;
    ipos["torso_lift_joint"] = 0.0;
    env_->setState(ipos);

    gLogLevel = util::LevelInfo;
  }
};

TEST_F(PlanningTest, numerical_ik1)
{
  CONSOLE_BRIDGE_logDebug("PlanningTest, numerical_ik1");

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/numerical_ik1.json");

//  plotter_->plotScene();

  ForwardKinematicsConstPtrMap kin_map = createKinematicsMap<KDLFwdKinChain, KDLFwdKinTree>(scene_graph_, srdf_model_);
  TrajOptProbPtr prob = ConstructProblem(root, env_, kin_map);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
//  if (plotting)
//  {
//    opt.addCallback(PlotCallback(*prob, plotter_));
//  }

  CONSOLE_BRIDGE_logDebug("DOF: %d", prob->GetNumDOF());
  opt.initialize(DblVec(static_cast<size_t>(prob->GetNumDOF()), 0));
  double tStart = GetClock();
  CONSOLE_BRIDGE_logDebug("Size: %d", opt.x().size());
  std::stringstream ss;
  ss << toVectorXd(opt.x()).transpose();
  CONSOLE_BRIDGE_logDebug("Initial Vars: %s", ss.str().c_str());
  Eigen::Isometry3d initial_pose, final_pose, change_base;
  change_base = prob->GetEnv()->getLinkTransform(prob->GetKin()->getBaseLinkName());
  prob->GetKin()->calcFwdKin(initial_pose, toVectorXd(opt.x()));
  initial_pose = change_base * initial_pose;

  ss = std::stringstream();
  ss << initial_pose.translation().transpose();
  CONSOLE_BRIDGE_logDebug("Initial Position: %s", ss.str().c_str());
  sco::OptStatus status = opt.optimize();
  CONSOLE_BRIDGE_logDebug("Status: %s", sco::statusToString(status).c_str());
  prob->GetKin()->calcFwdKin(final_pose, toVectorXd(opt.x()));
  final_pose = change_base * final_pose;

  Eigen::Isometry3d goal;
  goal.setIdentity();
  goal.translation() << 0.4, 0, 0.8;
  goal.linear() = Eigen::Quaterniond(0, 0, 1, 0).toRotationMatrix();

  for (auto i = 0; i < 4; ++i)
  {
    for (auto j = 0; j < 4; ++j)
    {
      EXPECT_NEAR(goal(i, j), final_pose(i, j), 1e-8);
    }
  }

  ss = std::stringstream();
  ss << final_pose.translation().transpose();
  CONSOLE_BRIDGE_logDebug("Final Position: %s", ss.str().c_str());

  ss = std::stringstream();
  ss << toVectorXd(opt.x()).transpose();
  CONSOLE_BRIDGE_logDebug("Final Vars: ", ss.str().c_str());

  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);
}

TEST_F(PlanningTest, arm_around_table)
{
  CONSOLE_BRIDGE_logDebug("PlanningTest, arm_around_table");

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR)  + "/test/data/config/arm_around_table.json");

  std::unordered_map<std::string, double> ipos;
  ipos["torso_lift_joint"] = 0;
  ipos["r_shoulder_pan_joint"] = -1.832;
  ipos["r_shoulder_lift_joint"] = -0.332;
  ipos["r_upper_arm_roll_joint"] = -1.011;
  ipos["r_elbow_flex_joint"] = -1.437;
  ipos["r_forearm_roll_joint"] = -1.1;
  ipos["r_wrist_flex_joint"] = -1.926;
  ipos["r_wrist_roll_joint"] = 3.074;
  env_->setState(ipos);

//  plotter_->plotScene();

  ForwardKinematicsConstPtrMap kin_map = createKinematicsMap<KDLFwdKinChain, KDLFwdKinTree>(scene_graph_, srdf_model_);
  TrajOptProbPtr prob = ConstructProblem(root, env_, kin_map);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  ContinuousContactManagerPtr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMapPtr adjacency_map = std::make_shared<AdjacencyMap>(scene_graph_,
                                                                 prob->GetKin()->getActiveLinkNames(),
                                                                 prob->GetEnv()->getCurrentState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), prob->GetInitTraj(), collisions);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  CONSOLE_BRIDGE_logDebug("DOF: %d", prob->GetNumDOF());
//  if (plotting)
//  {
//    opt.addCallback(PlotCallback(*prob, plotter_));
//  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();
  opt.optimize();
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);

  double d = 0;
  TrajArray traj = getTraj(opt.x(), prob->GetVars());
  for (unsigned i = 1; i < traj.rows(); ++i)
  {
    for (unsigned j = 0; j < traj.cols(); ++j)
    {
      d += std::abs(traj(i, j) - traj(i - 1, j));
    }
  }
  CONSOLE_BRIDGE_logDebug("trajectory norm: %.3f", d);

//  if (plotting)
//  {
//    plotter_->clear();
//  }

  collisions.clear();
  found = checkTrajectory(*manager, *prob->GetEnv(), prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

//  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
