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

#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <tesseract_ros/bullet/bullet_env.h>

#include <ros/ros.h>
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

class CastTest : public testing::TestWithParam<const char*> {
public:
  ros::NodeHandle nh_;
  urdf::ModelInterfaceSharedPtr model_;  /**< URDF Model */
  srdf::ModelSharedPtr srdf_model_;      /**< SRDF Model */
  tesseract::BulletEnvPtr env_;   /**< Trajopt Basic Environment */

  virtual void SetUp()
  {
    std::string urdf_xml_string, srdf_xml_string;
    nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
    model_ = urdf::parseURDF(urdf_xml_string);

    srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model_->initString(*model_, srdf_xml_string);
    env_ = tesseract::BulletEnvPtr(new tesseract::BulletEnv);
    assert(model_ != nullptr);
    assert(env_ != nullptr);

    bool success = env_->init(model_, srdf_model_);
    assert(success);

    gLogLevel = util::LevelInfo;
  }
};

TEST_F(CastTest, boxes) {
  ROS_DEBUG("CastTest, boxes");

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/box_cast_test.json");

  std::map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env_->setState(ipos);
  env_->updateVisualization();

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  tesseract::DistanceResultVector collisions;
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
  ros::init(argc, argv, "trajopt_cast_cost_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
