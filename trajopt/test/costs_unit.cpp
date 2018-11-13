#include <ctime>
#include <gtest/gtest.h>
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

#include <ros/package.h>
#include <ros/ros.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
bool plotting = false;                                                 /**< Enable plotting */

class CostsTest : public testing::TestWithParam<const char*>
{
public:
  ros::NodeHandle nh_;
  urdf::ModelInterfaceSharedPtr urdf_model_;   /**< URDF Model */
  srdf::ModelSharedPtr srdf_model_;            /**< SRDF Model */
  tesseract_ros::KDLEnvPtr env_;               /**< Trajopt Basic Environment */
  tesseract_ros::ROSBasicPlottingPtr plotter_; /**< Trajopt Plotter */

  virtual void SetUp()
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

    gLogLevel = util::LevelError;
  }
};

/**
 * @brief Tests the equality jointVel cost/constraints.
 *
 * Sets a cost targetting the velocity of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the velocity of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the cost
 * target
 */
TEST_F(CostsTest, equality_jointVel)
{
  ROS_DEBUG("CostsTest, equality_jointVel");

  const double cnt_targ = 0.0;
  const double cost_targ = 0.1;
  const int steps = 10;
  const double cost_tol = 0.01;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that first step velocity should be zero
  std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targs = std::vector<double>(7.0, cnt_targ);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_vel_single";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  std::shared_ptr<JointVelTermInfo> jv2 = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv2->coeffs = std::vector<double>(7, 10.0);
  jv2->targs = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_vel_all";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  OptStatus status = opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity constraint is satisfied
  double velocity;
  for (auto j = 0; j < output.cols(); ++j)
  {
    velocity = output(1, j) - output(0, j);
    EXPECT_NEAR(velocity, cnt_targ, cnt_tol);
  }
  // Check velocity cost is working
  for (auto i = 1; i < output.rows() - 1; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      velocity = output(i + 1, j) - output(i, j);
      EXPECT_NEAR(velocity, cost_targ, cost_tol);
    }
  }
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_costs_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
