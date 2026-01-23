#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <console_bridge/console.h>
#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/composite.h>
#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

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
  CompositeVariables::Ptr variables = std::make_shared<CompositeVariables>("variable-sets");

  tesseract_kinematics::JointGroup::ConstPtr manip;
  CartLineInfo info;
  std::shared_ptr<const Var> var;

  Eigen::Isometry3d source_tf;
  Eigen::Isometry3d line_start_pose;
  Eigen::Isometry3d line_end_pose;

  Eigen::Index n_dof{ 0 };

  void SetUp() override
  {
    // Initialize Tesseract
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    const bool status = env->init(urdf_file, srdf_file, locator);
    EXPECT_TRUE(status);

    // Extract necessary kinematic information
    manip = env->getJointGroup("right_arm");
    n_dof = manip->numJoints();

    std::vector<Bounds> bounds(static_cast<std::size_t>(manip->numJoints()), NoBound);
    auto node = std::make_unique<Node>("Joint_Position_0");
    auto pos = Eigen::VectorXd::Ones(n_dof);
    var = node->addVar("position", manip->getJointNames(), pos, bounds);
    std::vector<std::unique_ptr<Node>> nodes;
    nodes.push_back(std::move(node));
    variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

    // Add constraints
    const Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);
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

    info = CartLineInfo(manip, "r_gripper_tool_frame", "base_link", line_start_pose, line_end_pose);
    const Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
    auto constraint = std::make_shared<CartLineConstraint>(info, var, coeff);
    constraint->linkWithVariables(variables);

    // Given a joint position at the target, the error should be 0
    {
      auto error = constraint->calcValues(joint_position);
      EXPECT_LT(error.maxCoeff(), 1e-3) << error.maxCoeff();
      EXPECT_GT(error.minCoeff(), -1e-3) << error.minCoeff();
    }

    {
      auto error = constraint->getValues();
      EXPECT_LT(error.maxCoeff(), 1e-3);
      EXPECT_GT(error.minCoeff(), -1e-3);
    }

    {  // Orientation test
      joint_position[6] += 0.707;
      auto error = constraint->calcValues(joint_position);
      EXPECT_NEAR(error.norm(), 0.707, 1e-3);
    }
  }

  // distance error with a 3-4-5 triangle
  {
    const Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);

    Eigen::Isometry3d start_pose_mod = line_start_pose;
    Eigen::Isometry3d end_pose_mod = line_end_pose;
    start_pose_mod.translation() = start_pose_mod.translation() + Eigen::Vector3d(0.0, 0.3, 0.4);
    end_pose_mod.translation() = end_pose_mod.translation() + Eigen::Vector3d(0.0, 0.3, 0.4);

    info = CartLineInfo(manip, "r_gripper_tool_frame", "base_link", start_pose_mod, end_pose_mod);
    const Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
    auto constraint = std::make_shared<CartLineConstraint>(info, var, coeff);
    constraint->linkWithVariables(variables);

    auto error = constraint->calcValues(joint_position);
    EXPECT_NEAR(error.norm(), 0.5, 1e-2);
  }
}

///** @brief Checks that the FillJacobian function is correct */
TEST_F(CartesianLineConstraintUnit, FillJacobian)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, FillJacobian");

  // Run FK to get target pose
  const Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);
  Eigen::Isometry3d source_tf = manip->calcFwdKin(joint_position).at("r_gripper_tool_frame");

  // Set the line endpoints st the target pose is on the line
  const Eigen::Isometry3d start_pose_mod = source_tf.translate(Eigen::Vector3d(-1.0, 0, 0));
  const Eigen::Isometry3d end_pose_mod = source_tf.translate(Eigen::Vector3d(1.0, 0, 0));

  info = CartLineInfo(manip, "r_gripper_tool_frame", "base_link", start_pose_mod, end_pose_mod);
  const Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
  auto constraint = std::make_shared<CartLineConstraint>(info, var, coeff);
  constraint->linkWithVariables(variables);

  // below here should match cartesian
  // Modify one joint at a time
  for (Eigen::Index i = 0; i < n_dof; i++)
  {
    // Set the joints
    Eigen::VectorXd joint_position_mod = joint_position;
    joint_position_mod[i] = 2.0;
    variables->setVariables(joint_position_mod);

    // Calculate jacobian numerically
    auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return constraint->calcValues(x); };
    const Jacobian num_jac_block = calcForwardNumJac(error_calculator, joint_position_mod, 1e-4);

    // Compare to constraint jacobian
    {
      Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
      constraint->calcJacobianBlock(joint_position_mod, jac_block);  // NOLINT
      EXPECT_TRUE(jac_block.isApprox(num_jac_block, 1e-3));
    }
    {
      Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
      constraint->fillJacobianBlock("joint_trajectory", jac_block);
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
  info = CartLineInfo(manip, "r_gripper_tool_frame", "base_link", line_start_pose, line_end_pose);
  const Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
  auto constraint = std::make_shared<CartLineConstraint>(info, var, coeff);
  constraint->linkWithVariables(variables);

  const Bounds bounds(-0.1234, 0.5678);
  std::vector<Bounds> bounds_vec(6, bounds);

  constraint->setBounds(bounds_vec);
  std::vector<Bounds> results_vec = constraint->getBounds();
  for (std::size_t i = 0; i < bounds_vec.size(); i++)
  {
    EXPECT_EQ(bounds_vec[i].getLower(), results_vec[i].getLower());
    EXPECT_EQ(bounds_vec[i].getUpper(), results_vec[i].getUpper());
  }
}

///**
// * @brief Checks that the constraint doesn't change the jacobian when it shouldn't
// */
TEST_F(CartesianLineConstraintUnit, IgnoreVariables)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, IgnoreVariables");

  info = CartLineInfo(manip, "r_gripper_tool_frame", "base_link", line_start_pose, line_end_pose);
  const Eigen::VectorXd coeff = Eigen::VectorXd::Ones(info.indices.rows());
  auto constraint = std::make_shared<CartLineConstraint>(info, var, coeff);
  constraint->linkWithVariables(variables);

  // Check that jacobian does not change for variables it shouldn't
  {
    Jacobian jac_block_input;
    jac_block_input.resize(n_dof, n_dof);
    constraint->fillJacobianBlock("another_var", jac_block_input);
    EXPECT_EQ(jac_block_input.nonZeros(), 0);
  }
  // Check that it is fine with jac blocks the wrong size for this constraint
  {
    Jacobian jac_block_input;
    jac_block_input.resize(3, 5);
    constraint->fillJacobianBlock("another_var2", jac_block_input);
    EXPECT_EQ(jac_block_input.nonZeros(), 0);
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
