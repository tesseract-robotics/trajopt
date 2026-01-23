/**
 * @file inverse_kinematics_constraint_unit.cpp
 * @brief The inverse kinematics constraint unit test
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <console_bridge/console.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>
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
using namespace tesseract_common;

class InverseKinematicsConstraintUnit : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>();
  Problem nlp;

  tesseract_kinematics::KinematicGroup::ConstPtr kin_group;
  InverseKinematicsInfo::Ptr kinematic_info;
  InverseKinematicsConstraint::Ptr constraint;
  std::vector<Bounds> joint_bounds;

  Eigen::Index n_dof{ -1 };

  void SetUp() override
  {
    // Initialize Tesseract
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    const ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    const bool status = env->init(urdf_file, srdf_file, locator);
    EXPECT_TRUE(status);

    // Extract necessary kinematic information
    kin_group = env->getKinematicGroup("right_arm");
    n_dof = kin_group->numJoints();
    joint_bounds = toBounds(kin_group->getLimits().joint_limits);

    kinematic_info = std::make_shared<InverseKinematicsInfo>(kin_group, "base_footprint", "r_gripper_tool_frame");

    auto pos = Eigen::VectorXd::Ones(kin_group->numJoints());
    std::vector<std::unique_ptr<Node>> nodes;
    nodes.push_back(std::make_unique<Node>("Joint_Position_0"));
    auto var0 = nodes.back()->addVar("position", kin_group->getJointNames(), pos, joint_bounds);
    nodes.push_back(std::make_unique<Node>("Joint_Position_1"));
    auto var1 = nodes.back()->addVar("position", kin_group->getJointNames(), pos, joint_bounds);

    nlp.addVariableSet(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

    // Add constraints
    auto target_pose = Eigen::Isometry3d::Identity();
    constraint = std::make_shared<InverseKinematicsConstraint>(target_pose, kinematic_info, var0, var1);
    nlp.addConstraintSet(constraint);
  }
};

TEST_F(InverseKinematicsConstraintUnit, GetValue)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("InverseKinematicsConstraintUnit, GetValue");

  // Run FK to get target pose
  Eigen::VectorXd joint_position_single = Eigen::VectorXd::Zero(kin_group->numJoints());
  auto target_pose = kin_group->calcFwdKin(joint_position_single).at("r_gripper_tool_frame");
  constraint->setTargetPose(target_pose);

  // Set the joints to that joint position
  Eigen::VectorXd joint_position = Eigen::VectorXd::Zero(n_dof * 2);
  nlp.setVariables(joint_position.data());

  // Get the value (distance from IK position)
  const Eigen::VectorXd values = constraint->getValues();
  EXPECT_TRUE(almostEqualRelativeAndAbs(values, Eigen::VectorXd::Zero(n_dof)));

  // Check that jac wrt constraint_var is identity
  {
    Jacobian jac_block;
    jac_block.resize(n_dof, n_dof);
    constraint->fillJacobianBlock("joint_trajectory", jac_block);
    // Check that the size is correct
    EXPECT_EQ(jac_block.nonZeros(), n_dof);
    // Check that it is identity
    for (Eigen::Index i = 0; i < n_dof; i++)
      EXPECT_EQ(jac_block.coeff(i, i), -1.0);

    // Check against numeric differentiation
    auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) {
      return constraint->calcValues(x, joint_position_single);
    };
    const Jacobian num_jac_block = calcForwardNumJac(error_calculator, joint_position_single, 1e-4);
    EXPECT_TRUE(jac_block.isApprox(num_jac_block));
  }

  // Check that jac wrt seed_var is zero
  {
    Jacobian jac_block;
    jac_block.resize(n_dof, n_dof);
    constraint->fillJacobianBlock("invalid_var_set", jac_block);
    EXPECT_EQ(jac_block.nonZeros(), 0);
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(InverseKinematicsConstraintUnit, GetSetBounds)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("InverseKinematicsConstraintUnit, GetSetBounds");

  // Check that setting bounds works
  {
    const Eigen::VectorXd pos = Eigen::VectorXd::Ones(n_dof);
    std::vector<std::unique_ptr<Node>> nodes;
    nodes.push_back(std::make_unique<Node>("Joint_Position_0"));
    auto var0 = nodes.back()->addVar("position", kin_group->getJointNames(), pos, joint_bounds);
    nodes.push_back(std::make_unique<Node>("Joint_Position_1"));
    auto var1 = nodes.back()->addVar("position", kin_group->getJointNames(), pos, joint_bounds);

    auto target_pose = Eigen::Isometry3d::Identity();
    auto constraint_2 = std::make_shared<InverseKinematicsConstraint>(target_pose, kinematic_info, var0, var1);

    const Bounds bounds(-0.1234, 0.5678);
    std::vector<Bounds> bounds_vec = std::vector<Bounds>(static_cast<std::size_t>(n_dof), bounds);

    constraint_2->setBounds(bounds_vec);
    std::vector<Bounds> results_vec = constraint_2->getBounds();
    for (std::size_t i = 0; i < bounds_vec.size(); i++)
    {
      EXPECT_EQ(bounds_vec[i].getLower(), results_vec[i].getLower());
      EXPECT_EQ(bounds_vec[i].getUpper(), results_vec[i].getUpper());
    }
  }
}

/**
 * @brief Checks that the constraint doesn't change the jacobian when it shouldn't
 */
TEST_F(InverseKinematicsConstraintUnit, IgnoreVariables)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("InverseKinematicsConstraintUnit, IgnoreVariables");

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
