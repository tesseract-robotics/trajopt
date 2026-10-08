/**
 * @file cartesian_position_constraint_unit.h
 * @brief The cartesian position constraint unit tests
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
#include <filesystem>
#include <limits>
#include <tuple>
#include <gtest/gtest.h>
#include <tesseract/common/logging.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/common/types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/utils.hpp>
#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

#include "trajopt_ifopt_test_utils.h"

using namespace trajopt_ifopt;
using namespace std;
using namespace trajopt_common;
using namespace tesseract::environment;
using namespace tesseract::kinematics;
using namespace tesseract::collision;
using namespace tesseract::scene_graph;
using namespace tesseract::geometry;
using namespace tesseract::common;

class CartesianPositionConstraintUnit : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>();
  std::shared_ptr<Problem> nlp;

  JointGroup::ConstPtr kin_group;
  CartPosConstraint::Ptr constraint;

  Eigen::Index n_dof{ -1 };

  void SetUp() override
  {
    // Initialize Tesseract
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    const bool status = env->init(urdf_file, srdf_file, locator);
    EXPECT_TRUE(status);

    // Extract necessary kinematic information
    kin_group = env->getJointGroup("right_arm");
    n_dof = kin_group->numJoints();

    const std::vector<Bounds> bounds(static_cast<std::size_t>(n_dof), NoBound);
    auto pos = Eigen::VectorXd::Ones(n_dof);
    auto node = std::make_unique<Node>("Joint_Position_0");
    const std::vector<std::string> joint_names = tesseract::common::toNames(kin_group->getJointIds());
    auto var0 = node->addVar("position", joint_names, pos, bounds);

    std::vector<std::unique_ptr<Node>> nodes;
    nodes.push_back(std::move(node));
    auto variables = std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes));
    nlp = std::make_shared<Problem>(variables);

    // 4) Add constraints
    constraint = std::make_shared<CartPosConstraint>(var0, kin_group, "r_gripper_tool_frame", "base_footprint");
    nlp->addConstraintSet(constraint);
  }
};

/** @brief Checks that the GetValue function is correct */
TEST_F(CartesianPositionConstraintUnit, GetValue)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CartesianPositionConstraintUnit, GetValue");

  // Run FK to get target pose
  Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);
  const Eigen::Isometry3d target_pose = kin_group->calcFwdKin(joint_position).at("r_gripper_tool_frame");
  constraint->setTargetPose(target_pose);

  // Set the joints to the joint position that should satisfy it
  nlp->setVariables(joint_position.data());

  // Given a joint position at the target, the error should be 0
  {
    auto error = constraint->calcValues(joint_position);
    EXPECT_LT(error.maxCoeff(), 1e-3);
    EXPECT_GT(error.minCoeff(), -1e-3);
  }
  {
    auto error = constraint->getValues();
    EXPECT_LT(error.maxCoeff(), 1e-3);
    EXPECT_GT(error.minCoeff(), -1e-3);
  }

  // Given a joint position a small distance away, check the error for translation
  {
    Eigen::Isometry3d target_pose_mod = target_pose;
    target_pose_mod.translate(Eigen::Vector3d(0.1, 0.0, 0.0));
    constraint->setTargetPose(target_pose_mod);
    auto error = constraint->calcValues(joint_position);
    EXPECT_NEAR(error[0], -0.1, 1e-3);
  }
  {
    Eigen::Isometry3d target_pose_mod = target_pose;
    target_pose_mod.translate(Eigen::Vector3d(0.0, 0.2, 0.0));
    constraint->setTargetPose(target_pose_mod);
    auto error = constraint->calcValues(joint_position);
    EXPECT_NEAR(error[1], -0.2, 1e-3);
  }
  {
    Eigen::Isometry3d target_pose_mod = target_pose;
    target_pose_mod.translate(Eigen::Vector3d(0.0, 0.0, -0.3));
    constraint->setTargetPose(target_pose_mod);
    auto error = constraint->calcValues(joint_position);
    EXPECT_NEAR(error[2], 0.3, 1e-3);
  }

  // TODO: Check error for orientation
}

/** @brief Checks that the FillJacobian function is correct */
TEST_F(CartesianPositionConstraintUnit, FillJacobian)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CartesianPositionConstraintUnit, FillJacobian");

  // Run FK to get target pose
  const Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof);
  const Eigen::Isometry3d target_pose = kin_group->calcFwdKin(joint_position).at("r_gripper_tool_frame");
  constraint->setTargetPose(target_pose);

  // Modify one joint at a time
  for (Eigen::Index i = 0; i < n_dof; i++)
  {
    // Set the joints
    Eigen::VectorXd joint_position_mod = joint_position;
    joint_position_mod[i] = 2.0;
    nlp->setVariables(joint_position_mod.data());

    // Calculate jacobian numerically
    auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return constraint->calcValues(x); };
    const Jacobian num_jac_block = calcForwardNumJac(error_calculator, joint_position_mod, 1e-4);

    // Compare to constraint jacobian
    {
      Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
      constraint->calcJacobianBlock(jac_block, joint_position_mod);  // NOLINT
      EXPECT_TRUE(jac_block.isApprox(num_jac_block, 1e-3));
      //      std::cout << "Numeric:\n" << num_jac_block.toDense() << '\n';
      //      std::cout << "Analytic:\n" << jac_block.toDense() << '\n';
    }
    {
      Jacobian jac_block = constraint->getJacobian();
      EXPECT_TRUE(jac_block.toDense().isApprox(num_jac_block.toDense(), 1e-3));
      //      std::cout << "Numeric:\n" << num_jac_block.toDense() << '\n';
      //      std::cout << "Analytic:\n" << jac_block.toDense() << '\n';
    }
  }
}

/**
 * @brief Check the jacobian with the source moving, the target moving and both moving, for rotated frame offsets and
 * joint origins, with numeric and with analytic differentiation
 *
 * The analytic jacobian is exact. A numeric one is as good as its one-sided step.
 */
TEST_F(CartesianPositionConstraintUnit, FillJacobianMovingFrames)  // NOLINT
{
  // An arm whose joints have rotated origins
  const ResourceLocator::Ptr locator = std::make_shared<GeneralResourceLocator>();
  const auto urdf_resource = locator->locateResource("package://tesseract/support/urdf/iiwa7.urdf");
  ASSERT_NE(urdf_resource, nullptr);
  const std::filesystem::path urdf_file = urdf_resource->getFilePath();
  Environment iiwa_env;
  ASSERT_TRUE(iiwa_env.init(urdf_file, locator));
  const std::vector<JointId> iiwa_joints{ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" };

  // Each group with a link at its tip, a link only some of its joints move and a link none of its joints move. The full
  // body has a linear joint, and joints that leave the tool where it is; it is taken a second time with the tool of
  // its other arm, on another branch, as the link only some of its joints move.
  struct Arm
  {
    JointGroup::ConstPtr manip;
    LinkId tool_link;
    LinkId middle_link;
    LinkId fixed_link;
  };
  const std::vector<Arm> arms{
    { kin_group, "r_gripper_tool_frame", "r_forearm_link", "base_footprint" },
    { env->getJointGroup("full_body"), "r_gripper_tool_frame", "r_forearm_link", "base_footprint" },
    { env->getJointGroup("full_body"), "r_gripper_tool_frame", "l_gripper_tool_frame", "base_footprint" },
    { iiwa_env.getJointGroup("manipulator", iiwa_joints), "ikfast_tcp_link", "link_3", "base" }
  };

  // Every row, and two rows with a range bound beside a row without a coefficient
  Eigen::VectorXd some_coeffs = Eigen::VectorXd::Ones(6);
  some_coeffs[3] = 0.0;
  std::vector<Bounds> some_bounds(6, BoundZero);
  some_bounds[1] = Bounds(-0.1, 0.2);
  some_bounds[4] = Bounds(-0.3, 0.1);
  const std::vector<std::pair<Eigen::VectorXd, std::vector<Bounds>>> rows{
    { Eigen::VectorXd::Ones(6), std::vector<Bounds>(6, BoundZero) }, { some_coeffs, some_bounds }
  };

  // A turn alone leaves the tool point on the axis of the last joint of an arm
  const std::vector<Eigen::Isometry3d> tool_offsets{
    Eigen::Isometry3d(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())),
    Eigen::Translation3d(0.05, 0.1, -0.2) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())
  };

  // Where the other frame sits from the tool: on it, turned a little, shifted and turned, and turned nearly half a
  // turn
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0;
  const std::vector<Eigen::Isometry3d> from_tool{ Eigen::Isometry3d::Identity(),
                                                  Eigen::Isometry3d(Eigen::AngleAxisd(1e-3, axis)),
                                                  Eigen::Translation3d(0.03, -0.02, 0.04) *
                                                      Eigen::AngleAxisd(0.4, axis),
                                                  Eigen::Isometry3d(Eigen::AngleAxisd(3.0, axis)) };

  for (const Arm& arm : arms)
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Eigen::VectorXd reference_position = Eigen::VectorXd::Ones(dof);

    auto node = std::make_unique<Node>("Joint_Position_0");
    const auto var = node->addVar("position",
                                  toNames(arm.manip->getJointIds()),
                                  reference_position,
                                  std::vector<Bounds>(static_cast<std::size_t>(dof), NoBound));

    // The reference position, and three with every third joint elsewhere
    std::vector<Eigen::VectorXd> joint_positions{ reference_position };
    for (Eigen::Index first = 0; first < 3; ++first)
    {
      joint_positions.emplace_back(reference_position);
      for (Eigen::Index i = first; i < dof; i += 3)
        joint_positions.back()[i] = 2.0;
    }

    // A joint turns the origin of the tool link, on its axis, without moving it
    for (const Eigen::VectorXd& joint_position : joint_positions)
    {
      const Eigen::MatrixXd twists = arm.manip->calcJacobian(joint_position, arm.tool_link);
      int rolls{ 0 };
      for (Eigen::Index c = 0; c < dof; ++c)
        rolls += static_cast<int>(twists.col(c).head(3).norm() < 1e-6 && twists.col(c).tail(3).norm() > 0.5);
      EXPECT_GT(rolls, 0) << arm.manip->getName();
    }

    const auto reference_transforms = arm.manip->calcFwdKin(reference_position);
    for (const Eigen::Isometry3d& tool_offset : tool_offsets)
    {
      for (const Eigen::Isometry3d& other_from_tool : from_tool)
      {
        // The other frame, given on the middle link and on the fixed link
        const Eigen::Isometry3d other = reference_transforms.at(arm.tool_link) * tool_offset * other_from_tool;
        const Eigen::Isometry3d middle_offset = reference_transforms.at(arm.middle_link).inverse() * other;
        const Eigen::Isometry3d fixed_offset = reference_transforms.at(arm.fixed_link).inverse() * other;

        // The source moves, the target moves, and both move, with either one on the tool
        const std::vector<std::tuple<LinkId, Eigen::Isometry3d, LinkId, Eigen::Isometry3d>> frames{
          { arm.tool_link, tool_offset, arm.fixed_link, fixed_offset },
          { arm.fixed_link, fixed_offset, arm.tool_link, tool_offset },
          { arm.tool_link, tool_offset, arm.middle_link, middle_offset },
          { arm.middle_link, middle_offset, arm.tool_link, tool_offset }
        };
        for (const auto& [source_link, source_offset, target_link, target_offset] : frames)
        {
          for (const auto& [coeffs, bounds] : rows)
          {
            CartPosConstraint moving(
                var, coeffs, bounds, arm.manip, source_link, target_link, source_offset, target_offset);

            for (const Eigen::VectorXd& joint_position : joint_positions)
            {
              auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return moving.calcValues(x); };
              const Eigen::MatrixXd num_jac_block = calcCentralNumJac(error_calculator, joint_position);

              for (const bool numeric : { true, false })
              {
                moving.use_numeric_differentiation = numeric;
                Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
                moving.calcJacobianBlock(jac_block, joint_position);  // NOLINT
                EXPECT_LT(maxDifference(jac_block.toDense(), num_jac_block), numeric ? 2e-5 : 1e-9)
                    << arm.manip->getName() << ", source " << source_link.name() << ", target " << target_link.name()
                    << (numeric ? ", numeric" : ", analytic") << ", rows " << num_jac_block.rows()
                    << ", other frame turned " << Eigen::AngleAxisd(other_from_tool.linear()).angle()
                    << ", joint position " << joint_position.transpose();
              }
            }
          }
        }
      }
    }
  }
}

/** @brief Check the analytic jacobian where the two frames have the same orientation to the last bit */
TEST_F(CartesianPositionConstraintUnit, FillJacobianNoRotationError)  // NOLINT
{
  // With every joint at zero the tool has the orientation of the fixed link, and the rotation error is zero exactly
  const Eigen::VectorXd joint_position = Eigen::VectorXd::Zero(n_dof);
  ASSERT_EQ(constraint->calcValues(joint_position).tail(3).norm(), 0.0);

  auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return constraint->calcValues(x); };
  const Eigen::MatrixXd num_jac_block = calcCentralNumJac(error_calculator, joint_position);

  constraint->use_numeric_differentiation = false;
  Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
  constraint->calcJacobianBlock(jac_block, joint_position);  // NOLINT
  EXPECT_LT(maxDifference(jac_block.toDense(), num_jac_block), 1e-9);
}

/**
 * @brief Check the jacobian of the translation rows alone and of one rotation row alone, with numeric and with
 * analytic differentiation
 */
TEST_F(CartesianPositionConstraintUnit, FillJacobianSomeRows)  // NOLINT
{
  auto node = std::make_unique<Node>("Joint_Position_0");
  const Eigen::VectorXd reference_position = Eigen::VectorXd::Ones(n_dof);
  const auto var = node->addVar("position",
                                toNames(kin_group->getJointIds()),
                                reference_position,
                                std::vector<Bounds>(static_cast<std::size_t>(n_dof), NoBound));

  std::vector<Eigen::VectorXd> joint_positions(2, reference_position);
  joint_positions.back()[1] = 2.0;
  joint_positions.back()[4] = 2.0;

  // The target sits shifted and turned from the tool, on a link none of the joints move and on one some of them move
  const auto reference_transforms = kin_group->calcFwdKin(reference_position);
  const Eigen::Isometry3d target = reference_transforms.at("r_gripper_tool_frame") *
                                   Eigen::Translation3d(0.03, -0.02, 0.04) *
                                   Eigen::AngleAxisd(0.4, Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0);

  const std::vector<Eigen::VectorXd> coeff_lists{ (Eigen::VectorXd(6) << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0).finished(),
                                                  (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished() };
  for (const LinkId target_link : { "base_footprint", "r_forearm_link" })
  {
    const Eigen::Isometry3d target_offset = reference_transforms.at(target_link).inverse() * target;
    for (const Eigen::VectorXd& coeffs : coeff_lists)
    {
      CartPosConstraint some_rows(var,
                                  coeffs,
                                  std::vector<Bounds>(6, BoundZero),
                                  kin_group,
                                  "r_gripper_tool_frame",
                                  target_link,
                                  Eigen::Isometry3d::Identity(),
                                  target_offset);

      for (const Eigen::VectorXd& joint_position : joint_positions)
      {
        auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return some_rows.calcValues(x); };
        const Eigen::MatrixXd num_jac_block = calcCentralNumJac(error_calculator, joint_position);
        ASSERT_EQ(num_jac_block.rows(), (coeffs.array() != 0.0).count());

        for (const bool numeric : { true, false })
        {
          some_rows.use_numeric_differentiation = numeric;
          Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
          some_rows.calcJacobianBlock(jac_block, joint_position);  // NOLINT
          EXPECT_LT(maxDifference(jac_block.toDense(), num_jac_block), numeric ? 2e-5 : 1e-9)
              << "target " << target_link.name() << (numeric ? ", numeric" : ", analytic") << ", rows "
              << num_jac_block.rows() << ", joint position " << joint_position.transpose();
        }
      }
    }
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(CartesianPositionConstraintUnit, GetSetBounds)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CartesianPositionConstraintUnit, GetSetBounds");

  // Check that setting bounds works
  {
    std::vector<Bounds> bounds_vec(static_cast<std::size_t>(n_dof), NoBound);
    auto node = std::make_unique<Node>("Joint_Position_0");
    const Eigen::VectorXd pos = Eigen::VectorXd::Ones(kin_group->numJoints());
    const std::vector<std::string> joint_names = tesseract::common::toNames(kin_group->getJointIds());
    auto var0 = node->addVar("position", joint_names, pos, bounds_vec);

    auto constraint_2 = std::make_shared<CartPosConstraint>(var0, kin_group, "r_gripper_tool_frame", "base_footprint");

    const Eigen::VectorXd coeffs = 10 * Eigen::VectorXd::Ones(6);
    const Bounds bounds(-0.1234, 0.5678);
    bounds_vec = std::vector<Bounds>(6, bounds);

    auto constraint_3 = std::make_shared<CartPosConstraint>(var0,
                                                            coeffs,
                                                            bounds_vec,
                                                            kin_group,
                                                            "r_gripper_tool_frame",
                                                            "base_footprint",
                                                            Eigen::Isometry3d::Identity(),
                                                            Eigen::Isometry3d::Identity(),
                                                            "test",
                                                            trajopt_ifopt::RangeBoundHandling::kKeepAsIs);
    const std::vector<Bounds> results_bounds = constraint_3->getBounds();
    const Eigen::VectorXd result_coeffs = constraint_3->getCoefficients();
    for (std::size_t i = 0; i < bounds_vec.size(); i++)
    {
      EXPECT_DOUBLE_EQ(coeffs(static_cast<Eigen::Index>(i)), result_coeffs(static_cast<Eigen::Index>(i)));
      EXPECT_DOUBLE_EQ(bounds_vec[i].getLower(), results_bounds[i].getLower());
      EXPECT_DOUBLE_EQ(bounds_vec[i].getUpper(), results_bounds[i].getUpper());
    }
  }
}

////////////////////////////////////////////////////////////////////

/** @brief Coefficients must be finite and non-negative */
TEST_F(CartesianPositionConstraintUnit, RejectsInvalidCoeffs)  // NOLINT
{
  auto node = std::make_unique<Node>("Joint_Position_0");
  const std::vector<std::string> joint_names = tesseract::common::toNames(kin_group->getJointIds());
  auto var0 = node->addVar("position",
                           joint_names,
                           Eigen::VectorXd::Ones(n_dof),
                           std::vector<Bounds>(static_cast<std::size_t>(n_dof), NoBound));

  for (const double bad : { -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
  {
    Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(6);
    coeffs(2) = bad;
    EXPECT_THROW(
        std::make_shared<CartPosConstraint>(
            var0, coeffs, std::vector<Bounds>(6, BoundZero), kin_group, "r_gripper_tool_frame", "base_footprint"),
        std::runtime_error);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
