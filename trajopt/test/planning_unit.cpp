#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
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

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
bool plotting = false;                                                 /**< Enable plotting */

class PlanningTest : public testing::TestWithParam<const char*>
{
public:
  ros::NodeHandle nh_;
  urdf::ModelInterfaceSharedPtr urdf_model_;   /**< URDF Model */
  srdf::ModelSharedPtr srdf_model_;            /**< SRDF Model */
  tesseract_ros::KDLEnvPtr env_;               /**< Trajopt Basic Environment */
  tesseract_ros::ROSBasicPlottingPtr plotter_; /**< Trajopt Plotter */

  void SetUp() override
  {
    std::string urdf_xml_string, srdf_xml_string;
    nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
    urdf_model_ = urdf::parseURDF(urdf_xml_string);

    srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model_->initString(*urdf_model_, srdf_xml_string);
    env_ = tesseract_ros::KDLEnvPtr(new tesseract_ros::KDLEnv);
    assert(urdf_model_ != nullptr);
    assert(env_ != nullptr);

    bool success = env_->init(urdf_model_, srdf_model_);
    assert(success);

    // Create plotting tool
    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    std::unordered_map<std::string, double> ipos;
    ipos["torso_lift_joint"] = 0.0;
    env_->setState(ipos);

    gLogLevel = util::LevelInfo;
  }
};

TEST_F(PlanningTest, numerical_ik1)
{
  ROS_DEBUG("PlanningTest, numerical_ik1");

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/numerical_ik1.json");

  plotter_->plotScene();

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  ROS_DEBUG_STREAM("DOF: " << prob->GetNumDOF());
  opt.initialize(DblVec(static_cast<size_t>(prob->GetNumDOF()), 0));
  double tStart = GetClock();
  ROS_DEBUG_STREAM("Size: " << opt.x().size());
  ROS_DEBUG_STREAM("Initial Vars: " << toVectorXd(opt.x()).transpose());
  Eigen::Isometry3d initial_pose, final_pose, change_base;
  change_base = prob->GetEnv()->getLinkTransform(prob->GetKin()->getBaseLinkName());
  prob->GetKin()->calcFwdKin(initial_pose, change_base, toVectorXd(opt.x()));

  ROS_DEBUG_STREAM("Initial Position: " << initial_pose.translation().transpose());
  sco::OptStatus status = opt.optimize();
  ROS_DEBUG_STREAM("Status: " << sco::statusToString(status));
  prob->GetKin()->calcFwdKin(final_pose, change_base, toVectorXd(opt.x()));

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

  ROS_DEBUG_STREAM("Final Position: " << final_pose.translation().transpose());
  ROS_DEBUG_STREAM("Final Vars: " << toVectorXd(opt.x()).transpose());
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);
}

TEST_F(PlanningTest, arm_around_table)
{
  ROS_DEBUG("PlanningTest, arm_around_table");

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/arm_around_table.json");

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

  plotter_->plotScene();

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  std::vector<tesseract::ContactResultMap> collisions;
  ContinuousContactManagerBasePtr manager = prob->GetEnv()->getContinuousContactManager();
  manager->setActiveCollisionObjects(prob->GetKin()->getLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), prob->GetInitTraj(), collisions);

  EXPECT_TRUE(found);
  ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  ROS_DEBUG_STREAM("DOF: " << prob->GetNumDOF());
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();
  opt.optimize();
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);

  double d = 0;
  TrajArray traj = getTraj(opt.x(), prob->GetVars());
  for (unsigned i = 1; i < traj.rows(); ++i)
  {
    for (unsigned j = 0; j < traj.cols(); ++j)
    {
      d += std::abs(traj(i, j) - traj(i - 1, j));
    }
  }
  ROS_INFO("trajectory norm: %.3f", d);

  if (plotting)
  {
    plotter_->clear();
  }

  collisions.clear();
  found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), getTraj(opt.x(), prob->GetVars()), collisions);

  EXPECT_FALSE(found);
  ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_planning_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
