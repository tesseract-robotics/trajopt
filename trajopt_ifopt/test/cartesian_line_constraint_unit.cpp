#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <console_bridge/console.h>
#include <ifopt/problem.h>
#include <tesseract_common/types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

using namespace trajopt_ifopt;
using namespace std;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

class CartesianLineConstraintUnit : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>();
  std::shared_ptr<ifopt::Composite> variables = std::make_shared<ifopt::Composite>("variable-sets", false);

  tesseract_kinematics::JointGroup::Ptr manip;
  CartLineInfo info;
  trajopt_ifopt::JointPosition::Ptr var;

  Eigen::Isometry3d source_tf;
  Eigen::Isometry3d line_start_pose;
  Eigen::Isometry3d line_end_pose;

  Eigen::Index n_dof{ 0 };

  void SetUp() override
  {
    // Initialize Tesseract
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    bool status = env->init(urdf_file, srdf_file, locator);
    EXPECT_TRUE(status);

    // Extract necessary kinematic information
    manip = env->getJointGroup("right_arm");
    n_dof = manip->numJoints();

    auto pos = Eigen::VectorXd::Ones(n_dof);
    var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_0");
    variables->AddComponent(var);

    // Add constraints
    Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);
    source_tf = manip->calcFwdKin(joint_position).at("r_gripper_tool_frame");

    line_start_pose = source_tf;
    line_start_pose.translation() = line_start_pose.translation() + Eigen::Vector3d(-0.5, 0.0, 0.0);
    line_end_pose = source_tf;
    line_end_pose.translation() = line_end_pose.translation() + Eigen::Vector3d(0.5, 0.0, 0.0);
  }
};

/** @brief Checks that the GetValue function is correct */
TEST_F(CartesianLineConstraintUnit, GetValue)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, GetValue");

  {
    Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);

    info = trajopt_ifopt::CartLineInfo(manip, "r_gripper_tool_frame", "base_link", line_start_pose, line_end_pose);
    Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
    auto constraint = std::make_shared<trajopt_ifopt::CartLineConstraint>(info, var, coeff);
    constraint->LinkWithVariables(variables);

    // Given a joint position at the target, the error should be 0
    {
      auto error = constraint->CalcValues(joint_position);
      EXPECT_LT(error.maxCoeff(), 1e-3) << error.maxCoeff();
      EXPECT_GT(error.minCoeff(), -1e-3) << error.minCoeff();
    }

    {
      auto error = constraint->GetValues();
      EXPECT_LT(error.maxCoeff(), 1e-3);
      EXPECT_GT(error.minCoeff(), -1e-3);
    }

    {  // Orientation test
      joint_position[6] += 0.707;
      auto error = constraint->CalcValues(joint_position);
      EXPECT_NEAR(error.norm(), 0.707, 1e-3);
    }
  }

  // distance error with a 3-4-5 triangle
  {
    Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);

    Eigen::Isometry3d start_pose_mod = line_start_pose;
    Eigen::Isometry3d end_pose_mod = line_end_pose;
    start_pose_mod.translation() = start_pose_mod.translation() + Eigen::Vector3d(0.0, 0.3, 0.4);
    end_pose_mod.translation() = end_pose_mod.translation() + Eigen::Vector3d(0.0, 0.3, 0.4);

    info = trajopt_ifopt::CartLineInfo(manip, "r_gripper_tool_frame", "base_link", start_pose_mod, end_pose_mod);
    Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
    auto constraint = std::make_shared<trajopt_ifopt::CartLineConstraint>(info, var, coeff);
    constraint->LinkWithVariables(variables);

    auto error = constraint->CalcValues(joint_position);
    EXPECT_NEAR(error.norm(), 0.5, 1e-2);
  }
}

///** @brief Checks that the FillJacobian function is correct */
TEST_F(CartesianLineConstraintUnit, FillJacobian)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, FillJacobian");

  // Run FK to get target pose
  Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);
  Eigen::Isometry3d source_tf = manip->calcFwdKin(joint_position).at("r_gripper_tool_frame");

  // Set the line endpoints st the target pose is on the line
  Eigen::Isometry3d start_pose_mod = source_tf.translate(Eigen::Vector3d(-1.0, 0, 0));
  Eigen::Isometry3d end_pose_mod = source_tf.translate(Eigen::Vector3d(1.0, 0, 0));

  info = trajopt_ifopt::CartLineInfo(manip, "r_gripper_tool_frame", "base_link", start_pose_mod, end_pose_mod);
  Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
  auto constraint = std::make_shared<trajopt_ifopt::CartLineConstraint>(info, var, coeff);
  constraint->LinkWithVariables(variables);

  // below here should match cartesian
  // Modify one joint at a time
  for (Eigen::Index i = 0; i < n_dof; i++)
  {
    // Set the joints
    Eigen::VectorXd joint_position_mod = joint_position;
    joint_position_mod[i] = 2.0;
    variables->SetVariables(joint_position_mod);

    // Calculate jacobian numerically
    auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return constraint->CalcValues(x); };
    ifopt::ConstraintSet::Jacobian num_jac_block =
        trajopt_ifopt::calcForwardNumJac(error_calculator, joint_position_mod, 1e-4);

    // Compare to constraint jacobian
    {
      ifopt::ConstraintSet::Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
      constraint->CalcJacobianBlock(joint_position_mod, jac_block);  // NOLINT
      EXPECT_TRUE(jac_block.isApprox(num_jac_block, 1e-3));
    }
    {
      ifopt::ConstraintSet::Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
      constraint->FillJacobianBlock("Joint_Position_0", jac_block);
      EXPECT_TRUE(jac_block.toDense().isApprox(num_jac_block.toDense(), 1e-3));
    }
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(CartesianLineConstraintUnit, GetSetBounds)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, GetSetBounds");

  // Check that setting bounds works
  info = trajopt_ifopt::CartLineInfo(manip, "r_gripper_tool_frame", "base_link", line_start_pose, line_end_pose);
  Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
  auto constraint = std::make_shared<trajopt_ifopt::CartLineConstraint>(info, var, coeff);
  constraint->LinkWithVariables(variables);

  ifopt::Bounds bounds(-0.1234, 0.5678);
  std::vector<ifopt::Bounds> bounds_vec = std::vector<ifopt::Bounds>(6, bounds);

  constraint->SetBounds(bounds_vec);
  std::vector<ifopt::Bounds> results_vec = constraint->GetBounds();
  for (size_t i = 0; i < bounds_vec.size(); i++)
  {
    EXPECT_EQ(bounds_vec[i].lower_, results_vec[i].lower_);
    EXPECT_EQ(bounds_vec[i].upper_, results_vec[i].upper_);
  }
}

///**
// * @brief Checks that the constraint doesn't change the jacobian when it shouldn't
// */
TEST_F(CartesianLineConstraintUnit, IgnoreVariables)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, IgnoreVariables");

  info = trajopt_ifopt::CartLineInfo(manip, "r_gripper_tool_frame", "base_link", line_start_pose, line_end_pose);
  Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
  auto constraint = std::make_shared<trajopt_ifopt::CartLineConstraint>(info, var, coeff);
  constraint->LinkWithVariables(variables);

  // Check that jacobian does not change for variables it shouldn't
  {
    ifopt::ConstraintSet::Jacobian jac_block_input;
    jac_block_input.resize(n_dof, n_dof);
    constraint->FillJacobianBlock("another_var", jac_block_input);
    EXPECT_EQ(jac_block_input.nonZeros(), 0);
  }
  // Check that it is fine with jac blocks the wrong size for this constraint
  {
    ifopt::ConstraintSet::Jacobian jac_block_input;
    jac_block_input.resize(3, 5);
    constraint->FillJacobianBlock("another_var2", jac_block_input);
    EXPECT_EQ(jac_block_input.nonZeros(), 0);
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
