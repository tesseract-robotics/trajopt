#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>

#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;

static bool plotting = false;                                          /**< Enable plotting */

class InterfaceTest : public testing::TestWithParam<const char*>
{
public:
  SceneGraphPtr scene_graph_;            /**< Scene Graph */
  SRDFModel srdf_model_;                 /**< SRDF Model */
  KDLEnvPtr env_;                        /**< Trajopt Basic Environment */
  AllowedCollisionMatrixPtr acm_;        /**< Allowed Collision Matrix */
  VisualizationPtr plotter_;             /**< Trajopt Plotter */
  ForwardKinematicsConstPtrMap kin_map_; /**< A map between manipulator name and kinematics object */
  void SetUp() override
  {
    std::string urdf_file = std::string(DATA_DIR) + "/data/arm_around_table.urdf";
    std::string srdf_file = std::string(DATA_DIR) + "/data/pr2.srdf";

    ResourceLocatorFn locator = locateResource;
    scene_graph_ = parseURDF(urdf_file, locator);
    EXPECT_TRUE(scene_graph_ != nullptr);
    EXPECT_TRUE(srdf_model_.initFile(*scene_graph_, srdf_file));

    env_ = KDLEnvPtr(new KDLEnv);
    EXPECT_TRUE(env_ != nullptr);
    EXPECT_TRUE(env_->init(scene_graph_));

    // Add collision detectors
    tesseract_collision_bullet::BulletDiscreteBVHManagerPtr dc(new tesseract_collision_bullet::BulletDiscreteBVHManager());
    tesseract_collision_bullet::BulletCastBVHManagerPtr cc(new tesseract_collision_bullet::BulletCastBVHManager());

    EXPECT_TRUE(env_->setDiscreteContactManager(dc));
    EXPECT_TRUE(env_->setContinuousContactManager(cc));

    // Add Allowed Collision Matrix
    acm_ = getAllowedCollisionMatrix(srdf_model_);
    IsContactAllowedFn fn = std::bind(&InterfaceTest::defaultIsContactAllowedFn, this, std::placeholders::_1, std::placeholders::_2);
    env_->setIsContactAllowedFn(fn);

    // Generate Kinematics Map
    kin_map_ = createKinematicsMap<KDLFwdKinChain, KDLFwdKinTree>(scene_graph_, srdf_model_);

    gLogLevel = util::LevelError;
  }

  bool defaultIsContactAllowedFn(const std::string& link_name1, const std::string& link_name2) const
  {
    if (acm_ != nullptr && acm_->isCollisionAllowed(link_name1, link_name2))
      return true;

    return false;
  }

};

/**
 * @brief Tests the C++ interface for correct initial traj generation when not using time
 *
 * Also tests InitInfo::STATIONARY
 */
TEST_F(InterfaceTest, initial_trajectory_cpp_interface)
{
  CONSOLE_BRIDGE_logDebug("InterfaceTest, initial_trajectory_cpp_interface");

  const int steps = 13;

  ProblemConstructionInfo pci(env_, kin_map_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = kin_map_[pci.basic_info.manip];

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
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
  CONSOLE_BRIDGE_logDebug("InterfaceTest, initial_trajectory_time_cpp_interface");

  const int steps = 13;
  const double dt = 0.12341234;
  const double end = 3;

  ProblemConstructionInfo pci(env_, kin_map_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = true;

  // Create Kinematic Object
  pci.kin = kin_map_[pci.basic_info.manip];

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
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
  CONSOLE_BRIDGE_logDebug("InterfaceTest, initial_trajectory_json_interface");

  // These must match the json file or the test will fail!!
  const int steps = 10;

  Json::Value root = readJsonFile(std::string(DATA_DIR)  + "/data/config/arm_around_table.json");

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

  TrajOptProbPtr prob = ConstructProblem(root, env_, kin_map_);
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
  CONSOLE_BRIDGE_logDebug("InterfaceTest, initial_trajectory_time_json_interface");

  // These must match the json file or the test will fail!!
  const int steps = 10;
  const double dt = 0.12341234;

  Json::Value root = readJsonFile(std::string(DATA_DIR)  + "/data/config/arm_around_table.json");

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

  TrajOptProbPtr prob = ConstructProblem(root, env_, kin_map_);
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
  CONSOLE_BRIDGE_logDebug("InterfaceTest, bitmask_test");

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

//  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
