#include <ctime>
#include <gtest/gtest.h>
#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_moveit/trajopt_moveit_env.h>
#include <trajopt_moveit/trajopt_moveit_plotting.h>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt_utils/clock.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace trajopt;
using namespace std;
using namespace util;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
bool plotting = false;                                           /**< Enable plotting */

class PlanningTest : public testing::TestWithParam<const char*>
{
public:
  robot_model_loader::RobotModelLoaderPtr loader_;   /**< Used to load the robot model */
  moveit::core::RobotModelPtr robot_model_;          /**< Robot model */
  planning_scene::PlanningScenePtr planning_scene_;  /**< Planning scene for the current robot model */
  trajopt_moveit::TrajOptMoveItEnvPtr env_;          /**< Trajopt Basic Environment */
  trajopt_moveit::TrajoptMoveItPlottingPtr plotter_; /**< Trajopt Plotter */

  virtual void SetUp()
  {
    loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
    robot_model_ = loader_->getModel();
    env_ = trajopt_moveit::TrajOptMoveItEnvPtr(new trajopt_moveit::TrajOptMoveItEnv);
    ASSERT_TRUE(robot_model_ != nullptr);
    ASSERT_NO_THROW(planning_scene_.reset(new planning_scene::PlanningScene(robot_model_)));
    robot_state::RobotState& rs = planning_scene_->getCurrentStateNonConst();
    std::vector<double> val;
    val.push_back(0);
    rs.setJointPositions("torso_lift_joint", val);

    // Now assign collision detection plugin
    collision_detection::CollisionPluginLoader cd_loader;
    std::string class_name = "BULLET";
    ASSERT_TRUE(cd_loader.activate(class_name, planning_scene_, true));

    ASSERT_TRUE(env_->init(planning_scene_));

    plotter_.reset(new trajopt_moveit::TrajoptMoveItPlotting(planning_scene_));

    gLogLevel = util::LevelError;
  }
};

TEST_F(PlanningTest, numerical_ik1)
{
  ROS_DEBUG("PlanningTest, numerical_ik1");

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/numerical_ik1.json");

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  ROS_DEBUG_STREAM("DOF: " << prob->GetNumDOF());
  opt.initialize(DblVec(prob->GetNumDOF(), 0));
  double tStart = GetClock();
  ROS_DEBUG_STREAM("Size: " << opt.x().size());
  ROS_DEBUG_STREAM("Initial Vars: " << toVectorXd(opt.x()).transpose());
  Eigen::Isometry3d initial_pose, final_pose, change_base;
  change_base = prob->GetEnv()->getLinkTransform(prob->GetKin()->getBaseLinkName());
  prob->GetKin()->calcFwdKin(initial_pose, change_base, toVectorXd(opt.x()));

  ROS_DEBUG_STREAM("Initial Position: " << initial_pose.translation().transpose());
  OptStatus status = opt.optimize();
  ROS_DEBUG_STREAM("Status: " << sco::statusToString(status));
  prob->GetKin()->calcFwdKin(final_pose, change_base, toVectorXd(opt.x()));

  Eigen::Isometry3d goal;
  goal.translation() << 0.4, 0, 0.8;
  goal.linear() = Eigen::Quaterniond(0, 0, 1, 0).toRotationMatrix();

  assert(goal.isApprox(final_pose, 1e-8));

  ROS_DEBUG_STREAM("Final Position: " << final_pose.translation().transpose());
  ROS_DEBUG_STREAM("Final Vars: " << toVectorXd(opt.x()).transpose());
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);
}

TEST_F(PlanningTest, arm_around_table)
{
  ROS_DEBUG("PlanningTest, arm_around_table");

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/arm_around_table.json");
  robot_state::RobotState& rs = planning_scene_->getCurrentStateNonConst();
  std::map<std::string, double> ipos;
  ipos["torso_lift_joint"] = 0;
  ipos["r_shoulder_pan_joint"] = -1.832;
  ipos["r_shoulder_lift_joint"] = -0.332;
  ipos["r_upper_arm_roll_joint"] = -1.011;
  ipos["r_elbow_flex_joint"] = -1.437;
  ipos["r_forearm_roll_joint"] = -1.1;
  ipos["r_wrist_flex_joint"] = -1.926;
  ipos["r_wrist_roll_joint"] = 3.074;
  rs.setVariablePositions(ipos);

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  tesseract::ContactResultVector collisions;
  const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
  const std::vector<std::string>& link_names = prob->GetKin()->getLinkNames();

  env_->continuousCollisionCheckTrajectory(joint_names, link_names, prob->GetInitTraj(), collisions);
  ROS_DEBUG("Initial trajector number of continuous collisions: %lui\n", collisions.size());
  ASSERT_NE(collisions.size(), 0);

  BasicTrustRegionSQP opt(prob);
  ROS_DEBUG_STREAM("DOF: " << prob->GetNumDOF());
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();
  opt.optimize();
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);

  if (plotting)
  {
    plotter_->clear();
  }

  collisions.clear();
  env_->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);
  ROS_DEBUG("Final trajectory number of continuous collisions: %lui\n", collisions.size());
  ASSERT_EQ(collisions.size(), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_planning_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
