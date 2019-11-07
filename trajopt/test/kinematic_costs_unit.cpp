#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <tesseract/tesseract.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

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

#include <trajopt/kinematic_terms.hpp>
#include <trajopt_sco/num_diff.hpp>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;

class KinematicCostsTest : public testing::Test
{
public:
  Tesseract::Ptr tesseract_ = std::make_shared<Tesseract>(); /**< Trajopt Basic Environment */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(tesseract_->init(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;
  }
};

static std::string toString(const Eigen::MatrixXd& mat)
{
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static void checkJacobian(const sco::VectorOfVector& f,
                          const sco::MatrixOfVector& dfdx,
                          const Eigen::VectorXd& values,
                          const double epsilon)
{
  Eigen::MatrixXd numerical = sco::calcForwardNumJac(f, values, epsilon);
  Eigen::MatrixXd analytical = dfdx(values);

  bool pass = numerical.isApprox(analytical, 1e-5);
  EXPECT_TRUE(pass);
  if (!pass)
  {
    CONSOLE_BRIDGE_logError("Numerical:\n %s", toString(numerical).c_str());
    CONSOLE_BRIDGE_logError("Analytical:\n %s", toString(analytical).c_str());
  }
}

TEST_F(KinematicCostsTest, CartPoseJacCalculator)
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, CartPoseJacCalculator");

  auto env = tesseract_->getEnvironment();
  auto kin = tesseract_->getFwdKinematicsManager()->getFwdKinematicSolver("right_arm");
  auto world_to_base = env->getCurrentState()->transforms.at(kin->getBaseLinkName());
  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->transforms);

  std::string link = "r_gripper_tool_frame";
  Eigen::Isometry3d input_pose = tesseract_->getEnvironment()->getCurrentState()->transforms.at(link);
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();

  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  CartPoseErrCalculator f(input_pose, kin, adjacency_map, world_to_base, link, tcp);
  CartPoseJacCalculator dfdx(input_pose, kin, adjacency_map, world_to_base, link, tcp);
  checkJacobian(f, dfdx, values, 1.0e-5);
}

TEST_F(KinematicCostsTest, DynamicCartPoseJacCalculator)
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, DynamicCartPoseJacCalculator");

  auto env = tesseract_->getEnvironment();
  auto kin = tesseract_->getFwdKinematicsManager()->getFwdKinematicSolver("full_body");
  std::unordered_map<std::string, double> j;
  j["l_elbow_flex_joint"] = -0.15;
  env->setState(j);
  auto world_to_base = env->getCurrentState()->transforms.at(kin->getBaseLinkName());
  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->transforms);

  std::string link = "r_gripper_tool_frame";
  std::string target = "l_gripper_tool_frame";
  Eigen::Isometry3d input_pose = tesseract_->getEnvironment()->getCurrentState()->transforms.at(link);
  Eigen::Isometry3d target_pose = tesseract_->getEnvironment()->getCurrentState()->transforms.at(target);
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();

  Eigen::VectorXd values(15);
  values.setZero();
  std::vector<std::string> joint_names = kin->getJointNames();
  for (size_t i = 0; i < 15; ++i)
  {
    if (joint_names[i] == "r_elbow_flex_joint" || joint_names[i] == "l_elbow_flex_joint")
      values(static_cast<long>(i)) = -0.15;

    if (joint_names[i] == "r_wrist_flex_joint" || joint_names[i] == "l_wrist_flex_joint")
      values(static_cast<long>(i)) = -0.1;
  }

  DynamicCartPoseErrCalculator f(target, kin, adjacency_map, world_to_base, link, tcp, tcp);
  DynamicCartPoseJacCalculator dfdx(target, kin, adjacency_map, world_to_base, link, tcp, tcp);
  checkJacobian(f, dfdx, values, 1.0e-5);
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
