#include <ctime>
#include <gtest/gtest.h>
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

#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <trajopt_moveit/trajopt_moveit_env.h>
#include <trajopt_moveit/trajopt_moveit_plotting.h>

#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace trajopt;
using namespace std;
using namespace util;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
bool plotting = false;

class CastTest : public testing::TestWithParam<const char*>
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

    // Now assign collision detection plugin
    collision_detection::CollisionPluginLoader cd_loader;
    std::string class_name = "BULLET";
    ASSERT_TRUE(cd_loader.activate(class_name, planning_scene_, true));

    ASSERT_TRUE(env_->init(planning_scene_));

    plotter_.reset(new trajopt_moveit::TrajoptMoveItPlotting(planning_scene_));

    gLogLevel = util::LevelInfo;
  }
};

TEST_F(CastTest, boxes)
{
  ROS_DEBUG("CastTest, boxes");

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/box_cast_test.json");

  robot_state::RobotState& rs = planning_scene_->getCurrentStateNonConst();
  std::map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
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
  if (plotting)
    opt.addCallback(PlotCallback(*prob, plotter_));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting)
    plotter_->clear();

  collisions.clear();
  env_->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);
  ROS_DEBUG("Final trajectory number of continuous collisions: %lui\n", collisions.size());
  ASSERT_EQ(collisions.size(), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_cast_cost_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
