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

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace trajopt;
using namespace std;
using namespace util;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
bool plotting = false;

class CastOctomapTest : public testing::TestWithParam<const char*>
{
public:
  ros::NodeHandle nh_;
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

    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    double delta = 0.05;
    int length = (1 / delta);

    for (int x = 0; x < length; ++x)
      for (int y = 0; y < length; ++y)
        for (int z = 0; z < length; ++z)
          full_cloud.push_back(pcl::PointXYZ(-0.5 + x * delta, -0.5 + y * delta, -0.5 + z * delta));

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(full_cloud, pointcloud_msg);

    octomap::Pointcloud octomap_data;
    octomap_msgs::OctomapWithPose octomap_world;
    geometry_msgs::Pose octomap_pose;
    octomap::pointCloud2ToOctomap(pointcloud_msg, octomap_data);
    octomap::OcTree octree(2 * delta);
    octree.insertPointCloud(octomap_data, octomap::point3d(0, 0, 0));

    octomap_msgs::fullMapToMsg(octree, octomap_world.octomap);
    octomap_world.octomap.header.frame_id = "base_link";
    octomap_world.octomap.header.stamp = ros::Time::now();

    octomap_pose.position.x = 0;
    octomap_pose.position.y = 0;
    octomap_pose.position.z = 0;
    octomap_pose.orientation.x = 0;
    octomap_pose.orientation.y = 0;
    octomap_pose.orientation.z = 0;
    octomap_pose.orientation.w = 1;

    octomap_world.header.frame_id = "base_link";
    octomap_world.header.stamp = ros::Time::now();
    octomap_world.origin = octomap_pose;

    planning_scene_->processOctomapMsg(octomap_world);

    ros::Publisher planning_scene_diff_publisher =
        nh_.advertise<moveit_msgs::PlanningScene>("/trajopt/planning_scene", 1, true);

    moveit_msgs::PlanningScene msg;
    planning_scene_->getPlanningSceneMsg(msg);
    planning_scene_diff_publisher.publish(msg);
    ros::Duration(0.25).sleep();

    gLogLevel = util::LevelInfo;
  }
};

TEST_F(CastOctomapTest, boxes)
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
  ros::init(argc, argv, "trajopt_cast_cost_octomap_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
