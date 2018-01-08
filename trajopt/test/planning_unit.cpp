#include <gtest/gtest.h>
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
#include <trajopt/ros_kin.h>
#include <trajopt/ros_env.h>
#include <trajopt/plot_callback.hpp>

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>

using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;


const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

class PlanningTest : public testing::TestWithParam<const char*> {
public:
  robot_model_loader::RobotModelLoaderPtr loader_;  /**< Used to load the robot model */
  moveit::core::RobotModelPtr robot_model_; /**< Robot model */
  planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene for the current robot model */
  ROSEnvPtr env_; /**< Trajopt Basic Environment */

  virtual void SetUp()
  {
    loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
    robot_model_ = loader_->getModel();
    env_ = ROSEnvPtr(new ROSEnv);
    ASSERT_TRUE(robot_model_ != nullptr);
    ASSERT_NO_THROW(planning_scene_.reset(new planning_scene::PlanningScene(robot_model_)));
    robot_state::RobotState &rs = planning_scene_->getCurrentStateNonConst();
    std::vector<double> val;
    val.push_back(0);
    rs.setJointPositions("torso_lift_joint", val);

//    //Now assign collision detection plugin
//    collision_detection::CollisionPluginLoader cd_loader;
//    std::string class_name = "IndustrialFCL";
//    ASSERT_TRUE(cd_loader.activate(class_name, planning_scene_, true));

    ASSERT_TRUE(env_->init(planning_scene_));
  }
};


//TEST_F(PlanningTest, numerical_ik1)
//{
//  Json::Value root = readJsonFile(string(DATA_DIR) + "/numerical_ik1.json");

//  TrajOptProbPtr prob = ConstructProblem(root, env_);
//  ASSERT_TRUE(!!prob);

//  BasicTrustRegionSQP opt(prob);
////  opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),*prob->GetRAD(), prob->GetVars(), _1));
//  ROS_ERROR_STREAM("DOF: " << prob->GetNumDOF());
//  opt.initialize(DblVec(prob->GetNumDOF(), 0));
//  double tStart = GetClock();
//  ROS_ERROR_STREAM("Size: " << opt.x().size());
//  ROS_ERROR_STREAM("Initial Vars: " << toVectorXd(opt.x()).transpose());
//  Eigen::Affine3d initial_pose, final_pose, change_base;
//  change_base = prob->GetEnv()->getLinkTransform(prob->GetKin()->getBaseLinkName());
//  prob->GetKin()->calcFwdKin(initial_pose, change_base, toVectorXd(opt.x()));

//  ROS_ERROR_STREAM("Initial Position: " << initial_pose.translation().transpose());
//  OptStatus status = opt.optimize();
//  ROS_ERROR_STREAM("Status: " << sco::statusToString(status));
//  prob->GetKin()->calcFwdKin(final_pose, change_base, toVectorXd(opt.x()));

//  ROS_ERROR_STREAM("Final Position: " << final_pose.translation().transpose());
//  ROS_ERROR_STREAM("Final Vars: " << toVectorXd(opt.x()).transpose());
//  ROS_ERROR("planning time: %.3f", GetClock()-tStart);
//}

TEST_F(PlanningTest, arm_around_table)
{
  ROS_DEBUG("TEST\n");

//  RobotBasePtr pr2 = GetRobot(*env);

//  ProblemConstructionInfo pci(planning_scene_);
//  Json::Value root = readJsonFile(string(DATA_DIR) + "/arm_around_table.json");
//  pci.fromJson(root);
//  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
//  TrajOptProbPtr prob = ConstructProblem(pci);
//  ASSERT_TRUE(!!prob);
  Json::Value root = readJsonFile(string(DATA_DIR) + "/arm_around_table.json");
  robot_state::RobotState &rs = planning_scene_->getCurrentStateNonConst();;
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

  BasicTrustRegionSQP opt(prob);
  ROS_ERROR_STREAM("DOF: " << prob->GetNumDOF());
  opt.addCallback(PlotCallback(*prob));

//  TrajPlotter plotter(env, pci.rad, prob->GetVars());
//  if (plotting) {
//    plotter.Add(prob->getCosts());
//    if (plotting) opt.addCallback(boost::bind(&TrajPlotter::OptimizerCallback, boost::ref(plotter), _1, _2));
//    plotter.AddLink(pr2->GetLink("r_gripper_tool_frame"));
//  }
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();
  opt.optimize();
  ROS_ERROR("planning time: %.3f", GetClock()-tStart);

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
