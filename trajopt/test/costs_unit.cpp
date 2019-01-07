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
static bool plotting = false;                                          /**< Enable plotting */

class CostsTest : public testing::TestWithParam<const char*>
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

    gLogLevel = util::LevelError;
  }
};

/**
 * @brief Tests the equality jointPos cost/constraints.
 *
 * Sets a cost targetting the position of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the position of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the
 cost
 * target
 */
TEST_F(CostsTest, equality_jointPos)
{
  ROS_DEBUG("CostsTest, equality_jointPos");

  const double cnt_targ = 0.0;
  const double cost_targ = -0.1;
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
  std::shared_ptr<JointPosTermInfo> jv = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_pos_single";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  std::shared_ptr<JointPosTermInfo> jv2 = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jv2->coeffs = std::vector<double>(7, 10.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_pos_all";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check position constraint is satisfied
  double pos;
  for (auto j = 0; j < output.cols(); ++j)
  {
    pos = output(0, j);
    EXPECT_NEAR(pos, cnt_targ, cnt_tol);
  }
  // Check pos cost is working
  for (auto i = 1; i < output.rows(); ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      pos = output(i, j);
      EXPECT_NEAR(pos, cost_targ, cost_tol);
    }
  }
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointPos constraint
 *
 * Sets a cost targetting the position of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 rest.
 * Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1.
 * Checks to make sure that the constraint is met
 */
TEST_F(CostsTest, inequality_jointPos)
{
  ROS_DEBUG("CostsTest, inequality_cnt_jointPos");

  const double lower_tol = -0.1;
  const double upper_tol = 0.1;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that limits velocity
  std::shared_ptr<JointPosTermInfo> jv = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_pos_limits";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // Joint Velocities also have a cost to some non zero value
  std::shared_ptr<JointPosTermInfo> jv2 = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.01);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_pos_targ_1";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  std::shared_ptr<JointPosTermInfo> jv3 = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_pos_targ_2";
  jv3->term_type = TT_COST;
  pci.cost_infos.push_back(jv3);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check pos cost is working
  double pos;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      pos = output(i, j);
      EXPECT_TRUE(pos < upper_tol + cnt_tol);
      EXPECT_TRUE(pos > lower_tol - cnt_tol);
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows(); ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      pos = output(i, j);
      EXPECT_TRUE(pos < upper_tol + cnt_tol);
      EXPECT_TRUE(pos > lower_tol - cnt_tol);
    }
  }
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests the equality jointVel cost/constraints.
 *
 * Sets a cost targetting the velocity of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the velocity of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the
 cost
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
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that first step velocity should be zero
  std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_vel_single";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  std::shared_ptr<JointVelTermInfo> jv2 = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv2->coeffs = std::vector<double>(7, 10.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_vel_all";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();

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

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointVel constraint
 *
 * Sets a cost targetting the velocity of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 rest.
 * Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1.
 * Checks to make sure that the constraint is met
 */
TEST_F(CostsTest, inequality_jointVel)
{
  ROS_DEBUG("CostsTest, inequality_cnt_jointVel");

  const double lower_tol = -0.1;
  const double upper_tol = 0.1;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that limits velocity
  std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_vel_limits";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // Joint Velocities also have a cost to some non zero value
  std::shared_ptr<JointVelTermInfo> jv2 = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.0);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_vel_targ_1";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  std::shared_ptr<JointVelTermInfo> jv3 = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_vel_targ_2";
  jv3->term_type = TT_COST;
  pci.cost_infos.push_back(jv3);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity cost is working
  double velocity;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      velocity = output(i + 1, j) - output(i, j);
      EXPECT_TRUE(velocity < upper_tol + cnt_tol);
      EXPECT_TRUE(velocity > lower_tol - cnt_tol);
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows() - 1; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      velocity = output(i + 1, j) - output(i, j);
      EXPECT_TRUE(velocity < upper_tol + cnt_tol);
      EXPECT_TRUE(velocity > lower_tol - cnt_tol);
    }
  }
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests the equality jointVel cost/constraints with time parameterization.
 *
 * Sets a cost targetting the velocity of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the velocity of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the cost
 * target
 */
TEST_F(CostsTest, equality_jointVel_time)
{
  ROS_DEBUG("CostsTest, equality_jointVel_time");

  const double cnt_targ = 0.0;
  const double cost_targ = 0.1;
  const int steps = 10;
  const double cost_tol = 0.01;
  const double cnt_tol = 0.0001;
  const double dt_lower = 0.01234;
  const double dt_upper = 1.5678;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = true;
  pci.basic_info.dt_lower_lim = dt_lower;
  pci.basic_info.dt_upper_lim = dt_upper;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  pci.init_info.type = InitInfo::STATIONARY;

  // Constraint that first step velocity should be zero
  std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->upper_tols = std::vector<double>(7, 0.0);
  jv->lower_tols = std::vector<double>(7, 0.0);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_vel_single";
  jv->term_type = TT_CNT | TT_USE_TIME;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  std::shared_ptr<JointVelTermInfo> jv2 = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_vel_all";
  jv2->term_type = TT_COST | TT_USE_TIME;
  pci.cost_infos.push_back(jv2);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  sco::OptStatus status = opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity constraint is satisfied
  double velocity;
  for (auto j = 0; j < output.cols() - 1; ++j)
  {
    velocity = (output(1, j) - output(0, j)) * output(0 + 1, output.cols() - 1);
    EXPECT_NEAR(velocity, cnt_targ, cnt_tol);
  }
  // Check velocity cost is working
  for (auto i = 1; i < output.rows() - 1; ++i)
  {
    // dt limit is obeyed
    EXPECT_TRUE(output(i, output.cols() - 1) <= dt_upper);
    EXPECT_TRUE(output(i, output.cols() - 1) >= dt_lower);
    for (auto j = 0; j < output.cols() - 1; ++j)
    {
      velocity = (output(i + 1, j) - output(i, j)) * output(i + 1, output.cols() - 1);
      EXPECT_NEAR(velocity, cost_targ, cost_tol);
    }
  }
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointVel constraint with time parameterization
 *
 * Sets a cost targetting the velocity of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 rest.
 * Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1.
 * Checks to make sure that the constraint is met
 */
TEST_F(CostsTest, inequality_jointVel_time)
{
  ROS_DEBUG("CostsTest, inequality_cnt_jointVel_time");

  const double lower_tol = -0.1;
  const double upper_tol = 0.1;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;
  const double dt_lower = 0.01234;
  const double dt_upper = 3.5678;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = true;
  pci.basic_info.dt_lower_lim = dt_lower;
  pci.basic_info.dt_upper_lim = dt_upper;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.dt = dt_upper - dt_lower;

  // Constraint that limits velocity
  std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_vel_limits";
  jv->term_type = TT_CNT | TT_USE_TIME;
  pci.cnt_infos.push_back(jv);

  // Joint Velocities also have a cost to some non zero value
  std::shared_ptr<JointVelTermInfo> jv2 = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.01);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_vel_targ_1";
  jv2->term_type = TT_COST | TT_USE_TIME;
  pci.cost_infos.push_back(jv2);

  std::shared_ptr<JointVelTermInfo> jv3 = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_vel_targ_2";
  jv3->term_type = TT_COST | TT_USE_TIME;
  pci.cost_infos.push_back(jv3);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  sco::OptStatus status = opt.optimize();
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity cost is working
  double velocity;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols() - 1; ++j)
    {
      velocity = (output(i + 1, j) - output(i, j)) * output(i + 1, output.cols() - 1);
      EXPECT_TRUE(velocity < (upper_tol + cnt_tol));
      EXPECT_TRUE(velocity > (lower_tol - cnt_tol));
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows() - 1; ++i)
  {
    for (auto j = 0; j < output.cols() - 1; ++j)
    {
      velocity = (output(i + 1, j) - output(i, j)) * output(i + 1, output.cols() - 1);
      EXPECT_TRUE(velocity < (upper_tol + cnt_tol));
      EXPECT_TRUE(velocity > (lower_tol - cnt_tol));
    }
  }
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests the equality jointAcc cost/constraints.
 *
 * Sets a cost targetting the acceleration of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the velocity of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the
 cost
 * target
 */
TEST_F(CostsTest, equality_jointAcc)
{
  ROS_DEBUG("CostsTest, equality_jointAcc");

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
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that first step velocity should be zero
  std::shared_ptr<JointAccTermInfo> jv = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_acc_single";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  std::shared_ptr<JointAccTermInfo> jv2 = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  jv2->coeffs = std::vector<double>(7, 10.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_acc_all";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check acceleration constraint is satisfied
  double accel;
  for (auto j = 0; j < output.cols(); ++j)
  {
    int i = 0;
    accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
    EXPECT_NEAR(accel, cnt_targ, cnt_tol);
  }
  // Check acceleration cost is working
  for (auto i = 1; i < output.rows() - 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
      EXPECT_NEAR(accel, cost_targ, cost_tol);
    }
  }
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointAcc constraint
 *
 * Sets a cost targetting the acceleration of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 * rest. Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1. Checks to make sure
 that
 * the constraint is met
 */
TEST_F(CostsTest, inequality_jointAcc)
{
  ROS_DEBUG("CostsTest, inequality_cnt_jointVel");

  const double lower_tol = -0.1;
  const double upper_tol = 0.1;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that limits accel
  std::shared_ptr<JointAccTermInfo> jv = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_acc_limits";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // Joint accel also have a cost to some non zero value
  std::shared_ptr<JointAccTermInfo> jv2 = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.01);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_acc_targ_1";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  std::shared_ptr<JointAccTermInfo> jv3 = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_acc_targ_2";
  jv3->term_type = TT_COST;
  pci.cost_infos.push_back(jv3);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();
  ROS_DEBUG("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check accel cost is working
  double accel;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
      EXPECT_TRUE(accel < upper_tol + cnt_tol);
      EXPECT_TRUE(accel > lower_tol - cnt_tol);
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows() - 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
      EXPECT_TRUE(accel < upper_tol + cnt_tol);
      EXPECT_TRUE(accel > lower_tol - cnt_tol);
    }
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_costs_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
