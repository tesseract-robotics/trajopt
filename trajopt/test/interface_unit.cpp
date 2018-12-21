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
#include <trajopt_utils/logging.hpp>

#include <ros/package.h>
#include <ros/ros.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <bitset>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
static bool plotting = false;                                          /**< Enable plotting */

class InterfaceTest : public testing::TestWithParam<const char*>
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
 * @brief Tests the C++ interface for correct initial traj generation when not using time
 *
 * Also tests InitInfo::STATIONARY
 */
TEST_F(InterfaceTest, initial_trajectory_cpp_interface)
{
  ROS_DEBUG("InterfaceTest, initial_trajectory_cpp_interface");

  const int steps = 13;

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

  std::shared_ptr<JointPosTermInfo> jv = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_pos_all";
  jv->term_type = TT_COST;
  pci.cost_infos.push_back(jv);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  trajopt::TrajArray initial_trajectory = prob->GetInitTraj();
  unsigned int num_joints = prob->GetKin()->numJoints();

  // Check that the initial trajectory is the correct size
  EXPECT_EQ(initial_trajectory.cols(), num_joints);
  EXPECT_EQ(steps, initial_trajectory.rows());
}

/**
 * @brief Tests the C++ interface for correct initial traj generation when using time
 *
 * Also tests InitInfo::JOINT_INTERPOLATED
 */
TEST_F(InterfaceTest, initial_trajectory_time_cpp_interface)
{
  ROS_DEBUG("InterfaceTest, initial_trajectory_time_cpp_interface");

  const int steps = 13;
  const double dt = 0.12341234;
  const double end = 3;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = true;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  pci.init_info.data = Eigen::VectorXd::Constant(7, 1, end);
  pci.init_info.dt = dt;

  std::shared_ptr<JointPosTermInfo> jv = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_vel_all";
  jv->term_type = TT_COST | TT_USE_TIME;
  pci.cost_infos.push_back(jv);

  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  trajopt::TrajArray initial_trajectory = prob->GetInitTraj();
  unsigned int num_joints = prob->GetKin()->numJoints();

  // Check that the initial trajectory is the correct size
  EXPECT_EQ(initial_trajectory.cols(), num_joints + 1);
  EXPECT_EQ(steps, initial_trajectory.rows());

  std::cout << initial_trajectory << "\n";
  for (auto i = 0; i < initial_trajectory.rows(); ++i)
  {
    // Check that dt is set correctly
    EXPECT_EQ(initial_trajectory(i, num_joints), dt);
    for (auto j = 0; j < initial_trajectory.cols() - 1; ++j)
    {
      // Check that joint interpolation is working correctly
      double targ = 0 + i * pci.init_info.data(j, 0) / (steps - 1);
      EXPECT_EQ(targ, initial_trajectory(i, j));
    }
  }
}
/**
 * @brief Tests the json interface for correct initial traj generation when not using time
 */
TEST_F(InterfaceTest, initial_trajectory_json_interface)
{
  ROS_DEBUG("InterfaceTest, initial_trajectory_json_interface");

  // These must match the json file or the test will fail!!
  const int steps = 10;

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/arm_around_table.json");

  std::unordered_map<std::string, double> ipos;
  ipos["torso_lift_joint"] = 0;
  ipos["r_shoulder_pan_joint"] = 0;
  ipos["r_shoulder_lift_joint"] = 0;
  ipos["r_upper_arm_roll_joint"] = 0;
  ipos["r_elbow_flex_joint"] = 0;
  ipos["r_forearm_roll_joint"] = 0;
  ipos["r_wrist_flex_joint"] = 0;
  ipos["r_wrist_roll_joint"] = 0;
  env_->setState(ipos);

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  trajopt::TrajArray initial_trajectory = prob->GetInitTraj();
  unsigned int num_joints = prob->GetKin()->numJoints();

  // Check that the initial trajectory is the correct size
  EXPECT_EQ(initial_trajectory.cols(), num_joints);
  EXPECT_EQ(steps, initial_trajectory.rows());
}

/**
 * @brief Tests the json interface for correct initial traj generation when using time
 */
TEST_F(InterfaceTest, initial_trajectory_time_json_interface)
{
  ROS_DEBUG("InterfaceTest, initial_trajectory_time_json_interface");

  // These must match the json file or the test will fail!!
  const int steps = 10;
  const double dt = 0.12341234;

  std::string package_path = ros::package::getPath("trajopt_test_support");
  Json::Value root = readJsonFile(package_path + "/config/arm_around_table_time.json");

  std::unordered_map<std::string, double> ipos;
  ipos["torso_lift_joint"] = 0;
  ipos["r_shoulder_pan_joint"] = 0;
  ipos["r_shoulder_lift_joint"] = 0;
  ipos["r_upper_arm_roll_joint"] = 0;
  ipos["r_elbow_flex_joint"] = 0;
  ipos["r_forearm_roll_joint"] = 0;
  ipos["r_wrist_flex_joint"] = 0;
  ipos["r_wrist_roll_joint"] = 0;
  env_->setState(ipos);

  TrajOptProbPtr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  trajopt::TrajArray initial_trajectory = prob->GetInitTraj();
  unsigned int num_joints = prob->GetKin()->numJoints();

  // Check that the initial trajectory is the correct size
  EXPECT_EQ(initial_trajectory.cols(), num_joints + 1);
  EXPECT_EQ(steps, initial_trajectory.rows());

  std::cout << "Initial Trajectory: \n" << initial_trajectory << "\n";

  for (auto i = 0; i < initial_trajectory.rows(); ++i)
  {
    // Check that dt is set correctly
    EXPECT_EQ(initial_trajectory(i, num_joints), dt);
  }
}

/**
 * @brief Tests the bitmasks used to switch between cost types
 *
 * This should probably be replaced with a smarter test in the future that actually makes sure the right costs/cnts are
 * being set
 */
TEST_F(InterfaceTest, bitmask_test)
{
  ROS_DEBUG("InterfaceTest, bitmask_test");

  // Define all of the cases to test
  std::vector<int> types{ (TT_CNT), (TT_COST), (TT_CNT | TT_USE_TIME), (TT_COST | TT_USE_TIME) };
  std::vector<bool> cost{ false, true, false, true };
  std::vector<bool> cnt{ true, false, true, false };
  std::vector<bool> time{ false, false, true, true };

  for (size_t i = 0; i < types.size(); i++)
  {
    std::bitset<8> x(static_cast<long long unsigned>(types[i]));

    std::cout << "index: " << i << "  types= " << types[i] << "   bitmask: " << x << "\n";
    // It is a cost
    EXPECT_TRUE(static_cast<bool>(types[i] & TT_COST) == cost[i]);
    // It is a cnt
    EXPECT_TRUE(static_cast<bool>(types[i] & TT_CNT) == cnt[i]);
    // It does use time
    EXPECT_TRUE(static_cast<bool>(types[i] & TT_USE_TIME) == time[i]);
    // It does not use time
    EXPECT_TRUE(static_cast<bool>(~(types[i] | ~TT_USE_TIME)) == !time[i]);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajopt_interface_unit");
  ros::NodeHandle pnh("~");

  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
