#include <gtest/gtest.h>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt/common.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <ctime>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/clock.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt/collision_terms.hpp>
#include <trajopt_utils/logging.hpp>

#include <trajopt_scene/kdl_chain_kin.h>
#include <trajopt_scene/bullet_env.h>

#include <ros/ros.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <ros/package.h>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace boost::assign;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot description */
bool plotting=false;

class CastOctomapTest : public testing::TestWithParam<const char*> {
public:
  ros::NodeHandle nh_;
  urdf::ModelInterfaceSharedPtr model_;  /**< URDF Model */
  srdf::ModelSharedPtr srdf_model_;      /**< SRDF Model */
  trajopt_scene::BulletEnvPtr env_;   /**< Trajopt Basic Environment */

  virtual void SetUp()
  {
    std::string urdf_xml_string, srdf_xml_string;
    nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
    model_ = urdf::parseURDF(urdf_xml_string);

    srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model_->initString(*model_, srdf_xml_string);
    env_ = trajopt_scene::BulletEnvPtr(new trajopt_scene::BulletEnv);
    assert(model_ != nullptr);
    assert(env_ != nullptr);

    bool success = env_->init(model_, srdf_model_);
    assert(success);

    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    double delta = 0.05;
    int length = (1/delta);

    for (int x = 0; x < length; ++x)
      for (int y = 0; y < length; ++y)
        for (int z = 0; z < length; ++z)
          full_cloud.push_back(pcl::PointXYZ(-0.5 + x*delta, -0.5 + y*delta, -0.5 + z*delta));

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(full_cloud, pointcloud_msg);

    octomap::Pointcloud octomap_data;
    octomap::pointCloud2ToOctomap(pointcloud_msg, octomap_data);
    octomap::OcTree* octree = new octomap::OcTree(2*delta);
    octree->insertPointCloud(octomap_data, octomap::point3d(0,0,0));

    trajopt_scene::AttachableObjectPtr obj(new trajopt_scene::AttachableObject());
    shapes::OcTree* octomap_world = new shapes::OcTree(std::shared_ptr<const octomap::OcTree>(octree));
    Eigen::Affine3d octomap_pose;

    octomap_pose.setIdentity();
    octomap_pose.translation() = Eigen::Vector3d(0, 0, 0);

    obj->name = "octomap_attached";
    obj->shapes.push_back(shapes::ShapeConstPtr(octomap_world));
    obj->shapes_trans.push_back(octomap_pose);
    env_->addAttachableObject(obj);

    gLogLevel = util::LevelInfo;
  }
};

TEST_F(CastOctomapTest, boxes) {
  ROS_DEBUG("CastTest, boxes");

  trajopt_scene::AttachedBodyInfo attached_body;
  attached_body.name = "attached_body";
  attached_body.object_name = "octomap_attached";
  attached_body.parent_link_name = "base_link";

  env_->attachBody(attached_body);
  env_->updateVisualization();

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/box_cast_test.json");

  std::map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env_->setState(ipos);

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  trajopt_scene::DistanceResultVector collisions;
  const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
  const std::vector<std::string>& link_names = prob->GetKin()->getLinkNames();

  env_->continuousCollisionCheckTrajectory(joint_names, link_names, prob->GetInitTraj(), collisions);
  ROS_DEBUG("Initial trajector number of continuous collisions: %lui\n", collisions.size());
  ASSERT_NE(collisions.size(), 0);

  BasicTrustRegionSQP opt(prob);
  if (plotting) opt.addCallback(PlotCallback(*prob));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting) prob->GetEnv()->plotClear();

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
