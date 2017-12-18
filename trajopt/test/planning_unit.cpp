#include <gtest/gtest.h>
#include <trajopt/collision_checker.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt/common.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_osgviewer/osgviewer.hpp>
#include <ctime>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/clock.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_test_utils.hpp>

#include <ros/ros.h>
#include <constrained_ik/basic_kin.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>

using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;
using constrained_ik::basic_kin::BasicKin;


const std::string GROUP_NAME = "right_arm"; /**< Default group name for tests */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

class PlanningTest : public testing::TestWithParam<const char*> {
public:
//  static EnvironmentBasePtr env;
//  static OSGViewerPtr viewer;
  robot_model_loader::RobotModelLoaderPtr loader_;  /**< Used to load the robot model */
  moveit::core::RobotModelPtr robot_model_; /**< Robot model */
  planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene for the current robot model */
  BasicKin kin; /**< Basic Kinematic Model of the robot.  */

  virtual void SetUp()
  {
    loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
    robot_model_ = loader_->getModel();

    ASSERT_TRUE(robot_model_ != nullptr);
    ASSERT_TRUE(kin.init(robot_model_->getJointModelGroup(GROUP_NAME)));
    ASSERT_NO_THROW(planning_scene_.reset(new planning_scene::PlanningScene(robot_model_)));

    //Now assign collision detection plugin
    collision_detection::CollisionPluginLoader cd_loader;
    std::string class_name = "IndustrialFCL";
    ASSERT_TRUE(cd_loader.activate(class_name, planning_scene_, true));
  }
};


//TEST_F(PlanningTest, numerical_ik1) {
//  Json::Value root = readJsonFile(string(DATA_DIR) + "/numerical_ik1.json");
//  TrajOptProbPtr prob = ConstructProblem(root, env);
//  ASSERT_TRUE(!!prob);

//  BasicTrustRegionSQP opt(prob);
////  opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),*prob->GetRAD(), prob->GetVars(), _1));
//  opt.initialize(DblVec(prob->GetNumDOF(), 0));
//  double tStart = GetClock();
//  opt.optimize();
//  RAVELOG_INFO("planning time: %.3f\n", GetClock()-tStart);

//}

TEST_F(PlanningTest, arm_around_table)
{
  ROS_DEBUG("TEST\n");

//  RobotBasePtr pr2 = GetRobot(*env);

  ProblemConstructionInfo pci(planning_scene_);
//  Json::Value root = readJsonFile(string(DATA_DIR) + "/arm_around_table.json");
//  pci.fromJson(root);
//  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
//  TrajOptProbPtr prob = ConstructProblem(pci);
//  ASSERT_TRUE(!!prob);


//  BasicTrustRegionSQP opt(prob);
//  TrajPlotter plotter(env, pci.rad, prob->GetVars());
//  if (plotting) {
//    plotter.Add(prob->getCosts());
//    if (plotting) opt.addCallback(boost::bind(&TrajPlotter::OptimizerCallback, boost::ref(plotter), _1, _2));
//    plotter.AddLink(pr2->GetLink("r_gripper_tool_frame"));
//  }
//  opt.initialize(trajToDblVec(prob->GetInitTraj()));
//  double tStart = GetClock();
//  opt.optimize();
//  RAVELOG_INFO("planning time: %.3f\n", GetClock()-tStart);

//  vector<Collision> collisions;
//  CollisionChecker::GetOrCreate(*env)->ContinuousCheckTrajectory(getTraj(opt.x(), prob->GetVars()), *pci.rad, collisions);
//  RAVELOG_INFO("number of continuous collisions: %i\n", collisions.size());
//  ASSERT_EQ(collisions.size(), 0);


}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_planning_unit");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
