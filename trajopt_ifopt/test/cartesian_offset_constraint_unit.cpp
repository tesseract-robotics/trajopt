/**
 * @file cartesian_offset_constraint_unit.cpp
 * @brief The cartesian offset constraint unit tests
 *
 * @author Roelof Oomen
 * @date October 13, 2026
 *
 * @copyright Copyright (c) 2026
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
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_offset_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>

#include "trajopt_ifopt_test_utils.h"

using namespace trajopt_ifopt;
using tesseract::common::LinkId;
using tesseract::kinematics::JointGroup;

namespace
{
/** Agreement of the jacobian with the central difference of the values: its step of 1e-5 leaves an error near 1e-10 */
constexpr double JAC_TOL = 1e-9;
/** Agreement of values with ones built from forward kinematics: rounding of a few chained transforms */
constexpr double VALUE_TOL = 1e-10;
/** Agreement with CartPosConstraint's forward difference: its step of 1e-5 leaves an error near 5e-6 */
constexpr double NUMERIC_JAC_TOL = 2e-5;

/** A joint group with the link the tied frame is on and a link none of its joints move */
struct Arm
{
  JointGroup::ConstPtr manip;
  LinkId tool;
  LinkId fixed_link;
};

/** A variable set of states of one joint group, and the variable of each state */
struct Trajectory
{
  std::shared_ptr<NodesVariables> variables;
  std::vector<std::shared_ptr<const Var>> vars;
};

Trajectory makeTrajectory(const JointGroup& manip, std::size_t state_count)
{
  Trajectory trajectory;
  const Eigen::Index dof = manip.numJoints();
  const std::vector<std::string> joint_names = tesseract::common::toNames(manip.getJointIds());
  std::vector<std::unique_ptr<Node>> nodes;
  for (std::size_t i = 0; i < state_count; ++i)
  {
    auto node = std::make_unique<Node>("Joint_Position_" + std::to_string(i));
    trajectory.vars.push_back(node->addVar("position",
                                           joint_names,
                                           Eigen::VectorXd::Zero(dof),
                                           std::vector<Bounds>(static_cast<std::size_t>(dof), NoBound)));
    nodes.push_back(std::move(node));
  }
  trajectory.variables = std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes));
  return trajectory;
}

/** The values of every variable of the set: the two states as given, any other state at 0.3 */
Eigen::VectorXd stack(const Trajectory& trajectory,
                      const Var& reference,
                      const Eigen::VectorXd& reference_state,
                      const Var& follower,
                      const Eigen::VectorXd& follower_state)
{
  Eigen::VectorXd x = Eigen::VectorXd::Constant(trajectory.variables->getRows(), 0.3);
  x.segment(reference.getIndex(), reference_state.size()) = reference_state;
  x.segment(follower.getIndex(), follower_state.size()) = follower_state;
  return x;
}

/** States of the reference [rad] */
std::vector<Eigen::VectorXd> referenceStates(Eigen::Index dof)
{
  return { Eigen::VectorXd::Ones(dof),
           Eigen::VectorXd::LinSpaced(dof, -1.2, 1.5),
           Eigen::VectorXd::LinSpaced(dof, 0.9, -0.7) };
}

/**
 * Follower state minus reference state [rad]. The angles of each add up to less than half a turn, which bounds the
 * rotation of the follower frame from the reference frame.
 */
std::vector<Eigen::VectorXd> followerOffsets(Eigen::Index dof)
{
  Eigen::VectorXd alternating = Eigen::VectorXd::Constant(dof, 2.1 / static_cast<double>(dof));
  for (Eigen::Index i = 1; i < dof; i += 2)
    alternating[i] = -alternating[i];
  return { Eigen::VectorXd::Zero(dof), Eigen::VectorXd::Constant(dof, 1.4 / static_cast<double>(dof)), alternating };
}

/** The frame on the link itself, and shifted and turned from it */
std::vector<Eigen::Isometry3d> frameOffsets()
{
  return { Eigen::Isometry3d::Identity(),
           Eigen::Translation3d(0.05, 0.1, -0.2) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()) };
}

/** The pose of the follower frame in the offset reference frame, from forward kinematics alone */
Eigen::VectorXd expectedValues(const Arm& arm,
                               const Eigen::Isometry3d& frame_offset,
                               const Eigen::Vector3d& unit_axis,
                               double distance,
                               const Eigen::VectorXd& reference_state,
                               const Eigen::VectorXd& follower_state)
{
  const Eigen::Isometry3d reference =
      arm.manip->calcFwdKin(reference_state).at(arm.tool) * frame_offset * Eigen::Translation3d(distance * unit_axis);
  const Eigen::Isometry3d follower = arm.manip->calcFwdKin(follower_state).at(arm.tool) * frame_offset;
  const Eigen::AngleAxisd turn(reference.linear().transpose() * follower.linear());
  Eigen::VectorXd values(6);
  values.head(3) = reference.linear().transpose() * (follower.translation() - reference.translation());
  values.tail(3) = turn.angle() * turn.axis();
  return values;
}

void expectBounds(const std::vector<Bounds>& bounds, const std::vector<std::pair<double, double>>& expected)
{
  ASSERT_EQ(bounds.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(bounds[i].getLower(), expected[i].first) << "row " << i;
    EXPECT_EQ(bounds[i].getUpper(), expected[i].second) << "row " << i;
  }
}

/** Compare the jacobian at the two states with the central difference of the values over every variable of the set */
void expectJacobian(CartOffsetConstraint& constraint,
                    const Trajectory& trajectory,
                    const Var& reference,
                    const Eigen::VectorXd& reference_state,
                    const Var& follower,
                    const Eigen::VectorXd& follower_state,
                    const std::string& what)
{
  const Eigen::VectorXd x = stack(trajectory, reference, reference_state, follower, follower_state);
  constraint.linkWithVariables(trajectory.variables);
  trajectory.variables->setVariables(x);

  const Eigen::Index dof = reference.size();
  const auto values = [&](const Eigen::Ref<const Eigen::VectorXd>& v) -> Eigen::VectorXd {
    return constraint.calcValues(v.segment(reference.getIndex(), dof), v.segment(follower.getIndex(), dof));
  };
  const Eigen::MatrixXd numeric = calcCentralNumJac(values, x);
  const Jacobian analytic = constraint.getJacobian();

  ASSERT_EQ(analytic.rows(), constraint.getRows()) << what;
  ASSERT_EQ(analytic.cols(), x.size()) << what;
  EXPECT_EQ(analytic.nonZeros(), constraint.getNonZeros()) << what;
  EXPECT_LT(maxDifference(analytic.toDense(), numeric), JAC_TOL) << what;
}
}  // namespace

/** The arms the tests run on. An environment is loaded when its arm is first asked for and kept for the suite. */
class CartesianOffsetConstraintUnit : public testing::Test
{
public:
  static void TearDownTestSuite()
  {
    pr2_env.reset();
    iiwa_env.reset();
  }

  static Arm pr2()
  {
    if (pr2_env == nullptr)
    {
      const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
      const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
      pr2_env = std::make_shared<tesseract::environment::Environment>();
      EXPECT_TRUE(pr2_env->init(urdf_file, srdf_file, std::make_shared<tesseract::common::GeneralResourceLocator>()));
    }
    return { pr2_env->getJointGroup("right_arm"), "r_gripper_tool_frame", "base_footprint" };
  }

  /** An arm whose joints have rotated origins */
  static Arm iiwa()
  {
    if (iiwa_env == nullptr)
    {
      const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
      const auto resource = locator->locateResource("package://tesseract/support/urdf/iiwa7.urdf");
      EXPECT_NE(resource, nullptr);
      iiwa_env = std::make_shared<tesseract::environment::Environment>();
      EXPECT_TRUE(iiwa_env->init(std::filesystem::path(resource->getFilePath()), locator));
    }
    const std::vector<tesseract::common::JointId> joints{ "joint_1", "joint_2", "joint_3", "joint_4",
                                                          "joint_5", "joint_6", "joint_7" };
    return { iiwa_env->getJointGroup("manipulator", joints), "ikfast_tcp_link", "base" };
  }

  static std::vector<Arm> arms() { return { pr2(), iiwa() }; }

private:
  // NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables): loaded once per test suite, reset after it
  static inline tesseract::environment::Environment::Ptr pr2_env;
  static inline tesseract::environment::Environment::Ptr iiwa_env;
  // NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
};

/** @brief With both states equal the follower frame sits at minus the offset in the reference frame, unturned */
TEST_F(CartesianOffsetConstraintUnit, ValuesAtEqualStates)  // NOLINT
{
  const std::vector<Eigen::Vector3d> axes{
    Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0
  };
  for (const Arm& arm : arms())
  {
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    for (const Eigen::Isometry3d& frame_offset : frameOffsets())
    {
      for (const Eigen::Vector3d& axis : axes)
      {
        for (const double distance : { 0.02, -0.05 })
        {
          const CartOffsetConstraint constraint(
              trajectory.vars[1], trajectory.vars[0], arm.manip, arm.tool, frame_offset, axis, distance);
          Eigen::VectorXd expected = Eigen::VectorXd::Zero(6);
          expected.head(3) = -distance * axis;
          for (const Eigen::VectorXd& state : referenceStates(arm.manip->numJoints()))
            EXPECT_LT(maxDifference(constraint.calcValues(state, state), expected), VALUE_TOL)
                << arm.manip->getName() << ", axis " << axis.transpose() << ", distance " << distance;
        }
      }
    }
  }
}

/** @brief The values are the pose of the follower frame in the offset reference frame */
TEST_F(CartesianOffsetConstraintUnit, ValuesAgainstForwardKinematics)  // NOLINT
{
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0;
  constexpr double distance = -0.02;
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const Var& reference = *trajectory.vars[1];
    const Var& follower = *trajectory.vars[0];
    for (const Eigen::Isometry3d& frame_offset : frameOffsets())
    {
      const CartOffsetConstraint constraint(
          trajectory.vars[1], trajectory.vars[0], arm.manip, arm.tool, frame_offset, axis, distance);
      for (const Eigen::VectorXd& reference_state : referenceStates(dof))
      {
        for (const Eigen::VectorXd& offset : followerOffsets(dof))
        {
          const Eigen::VectorXd follower_state = reference_state + offset;
          const Eigen::VectorXd expected =
              expectedValues(arm, frame_offset, axis, distance, reference_state, follower_state);
          EXPECT_LT(maxDifference(constraint.calcValues(reference_state, follower_state), expected), VALUE_TOL)
              << arm.manip->getName();

          // The values of the set are those of the states its variables hold
          trajectory.variables->setVariables(stack(trajectory, reference, reference_state, follower, follower_state));
          EXPECT_LT(maxDifference(constraint.getValues(), expected), VALUE_TOL) << arm.manip->getName();
        }
      }
    }
  }
}

/**
 * @brief With the reference state held, the values and the follower columns are those of a CartPosConstraint on the
 * follower whose target is the offset reference frame, given on a fixed link
 */
TEST_F(CartesianOffsetConstraintUnit, AgreesWithCartPosAtFixedReference)  // NOLINT
{
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0;
  constexpr double distance = -0.02;
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& reference = trajectory.vars[1];
    const std::shared_ptr<const Var>& follower = trajectory.vars[0];
    for (const Eigen::Isometry3d& frame_offset : frameOffsets())
    {
      CartOffsetConstraint offset_constraint(reference, follower, arm.manip, arm.tool, frame_offset, axis, distance);
      offset_constraint.linkWithVariables(trajectory.variables);
      for (const Eigen::VectorXd& reference_state : referenceStates(dof))
      {
        const auto reference_transforms = arm.manip->calcFwdKin(reference_state);
        const Eigen::Isometry3d target_offset = reference_transforms.at(arm.fixed_link).inverse() *
                                                reference_transforms.at(arm.tool) * frame_offset *
                                                Eigen::Translation3d(distance * axis);
        CartPosConstraint pose_constraint(follower, arm.manip, arm.tool, arm.fixed_link, frame_offset, target_offset);
        pose_constraint.linkWithVariables(trajectory.variables);

        for (const Eigen::VectorXd& offset : followerOffsets(dof))
        {
          trajectory.variables->setVariables(
              stack(trajectory, *reference, reference_state, *follower, reference_state + offset));
          EXPECT_LT(maxDifference(offset_constraint.getValues(), pose_constraint.getValues()), VALUE_TOL)
              << arm.manip->getName();

          const Eigen::MatrixXd follower_columns =
              offset_constraint.getJacobian().toDense().middleCols(follower->getIndex(), dof);
          for (const bool numeric : { true, false })
          {
            pose_constraint.use_numeric_differentiation = numeric;
            const Eigen::MatrixXd pose_columns =
                pose_constraint.getJacobian().toDense().middleCols(follower->getIndex(), dof);
            EXPECT_LT(maxDifference(follower_columns, pose_columns), numeric ? NUMERIC_JAC_TOL : VALUE_TOL)
                << arm.manip->getName() << (numeric ? ", numeric" : ", analytic");
          }
        }
      }
    }
  }
}

/** @brief An axis of any length gives an offset as long as the distance */
TEST_F(CartesianOffsetConstraintUnit, AxisIsNormalized)  // NOLINT
{
  const Arm arm = pr2();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  const CartOffsetConstraint constraint(trajectory.vars[1],
                                        trajectory.vars[0],
                                        arm.manip,
                                        arm.tool,
                                        Eigen::Isometry3d::Identity(),
                                        Eigen::Vector3d(0.0, 0.0, -3.0),
                                        0.02);
  EXPECT_LT((constraint.getAxis() - Eigen::Vector3d(0.0, 0.0, -1.0)).norm(), VALUE_TOL);
  EXPECT_EQ(constraint.getDistance(), 0.02);

  const Eigen::VectorXd state = Eigen::VectorXd::Ones(arm.manip->numJoints());
  Eigen::VectorXd expected = Eigen::VectorXd::Zero(6);
  expected[2] = 0.02;
  EXPECT_LT(maxDifference(constraint.calcValues(state, state), expected), VALUE_TOL);
}

/** @brief Without coefficients and bounds: six rows, unit coefficients, equalities */
TEST_F(CartesianOffsetConstraintUnit, DefaultRows)  // NOLINT
{
  const Arm arm = pr2();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  CartOffsetConstraint constraint(trajectory.vars[1],
                                  trajectory.vars[0],
                                  arm.manip,
                                  arm.tool,
                                  Eigen::Isometry3d::Identity(),
                                  Eigen::Vector3d::UnitZ(),
                                  -0.02);
  EXPECT_EQ(constraint.getName(), "CartOffset");
  EXPECT_EQ(constraint.getRows(), 6);
  EXPECT_EQ(constraint.update(), 6);
  EXPECT_EQ(constraint.getNonZeros(), 12 * arm.manip->numJoints());
  EXPECT_TRUE(constraint.getCoefficients().isApprox(Eigen::VectorXd::Ones(6)));
  expectBounds(constraint.getBounds(), std::vector<std::pair<double, double>>(6, { 0.0, 0.0 }));
}

/** @brief A row without a coefficient is dropped; a range bound is kept as one row or split in two */
TEST_F(CartesianOffsetConstraintUnit, RowsCoefficientsBounds)  // NOLINT
{
  const double inf = std::numeric_limits<double>::infinity();
  const Arm arm = pr2();
  const Eigen::Index dof = arm.manip->numJoints();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);

  Eigen::VectorXd coeffs(6);
  coeffs << 2, 3, 0, 5, 7, 11;
  std::vector<Bounds> bounds(6, BoundZero);
  bounds[1] = Bounds(-0.1, 0.2);
  bounds[2] = Bounds(-1.0, 1.0);
  bounds[3] = Bounds(0.5, inf);
  bounds[4] = Bounds(-0.3, 0.1);

  const Eigen::VectorXd reference_state = referenceStates(dof)[1];
  const Eigen::VectorXd follower_state = reference_state + followerOffsets(dof)[2];
  const Eigen::VectorXd all_values = expectedValues(
      arm, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), -0.02, reference_state, follower_state);

  const auto make = [&](RangeBoundHandling handling) {
    return CartOffsetConstraint(trajectory.vars[1],
                                trajectory.vars[0],
                                arm.manip,
                                arm.tool,
                                Eigen::Isometry3d::Identity(),
                                Eigen::Vector3d::UnitZ(),
                                -0.02,
                                coeffs,
                                bounds,
                                "Rows",
                                handling);
  };
  const auto expectRows = [&](CartOffsetConstraint& constraint, const std::vector<int>& rows) {
    EXPECT_EQ(constraint.getRows(), static_cast<int>(rows.size()));
    EXPECT_EQ(constraint.update(), static_cast<int>(rows.size()));
    EXPECT_EQ(constraint.getNonZeros(), 2 * dof * static_cast<Eigen::Index>(rows.size()));
    const Eigen::VectorXd values = constraint.calcValues(reference_state, follower_state);
    const Eigen::VectorXd kept_coeffs = constraint.getCoefficients();
    ASSERT_EQ(values.size(), static_cast<Eigen::Index>(rows.size()));
    ASSERT_EQ(kept_coeffs.size(), static_cast<Eigen::Index>(rows.size()));
    for (std::size_t i = 0; i < rows.size(); ++i)
    {
      EXPECT_NEAR(values[static_cast<Eigen::Index>(i)], all_values[rows[i]], VALUE_TOL) << "row " << i;
      EXPECT_EQ(kept_coeffs[static_cast<Eigen::Index>(i)], coeffs[rows[i]]) << "row " << i;
    }
  };

  CartOffsetConstraint kept = make(RangeBoundHandling::kKeepAsIs);
  EXPECT_EQ(kept.getName(), "Rows");
  expectRows(kept, { 0, 1, 3, 4, 5 });
  expectBounds(kept.getBounds(), { { 0, 0 }, { -0.1, 0.2 }, { 0.5, inf }, { -0.3, 0.1 }, { 0, 0 } });

  CartOffsetConstraint split = make(RangeBoundHandling::kSplitToTwoInequalities);
  expectRows(split, { 0, 1, 1, 3, 4, 4, 5 });
  expectBounds(split.getBounds(),
               { { 0, 0 }, { -0.1, inf }, { -inf, 0.2 }, { 0.5, inf }, { -0.3, inf }, { -inf, 0.1 }, { 0, 0 } });
}

/** @brief Without a coefficient on any row the set has no rows */
TEST_F(CartesianOffsetConstraintUnit, NoRows)  // NOLINT
{
  const Arm arm = pr2();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  CartOffsetConstraint constraint(trajectory.vars[1],
                                  trajectory.vars[0],
                                  arm.manip,
                                  arm.tool,
                                  Eigen::Isometry3d::Identity(),
                                  Eigen::Vector3d::UnitZ(),
                                  -0.02,
                                  Eigen::VectorXd::Zero(6),
                                  std::vector<Bounds>(6, BoundZero));
  constraint.linkWithVariables(trajectory.variables);

  EXPECT_EQ(constraint.getRows(), 0);
  EXPECT_EQ(constraint.getNonZeros(), 0);
  EXPECT_EQ(constraint.getValues().size(), 0);
  EXPECT_EQ(constraint.getCoefficients().size(), 0);
  EXPECT_TRUE(constraint.getBounds().empty());
  const Jacobian jacobian = constraint.getJacobian();
  EXPECT_EQ(jacobian.rows(), 0);
  EXPECT_EQ(jacobian.cols(), trajectory.variables->getRows());
}

/** @brief Arguments that describe no constraint are refused */
TEST_F(CartesianOffsetConstraintUnit, RejectsInvalidArguments)  // NOLINT
{
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Arm arm = pr2();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  const std::shared_ptr<const Var>& reference = trajectory.vars[1];
  const std::shared_ptr<const Var>& follower = trajectory.vars[0];
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  const auto make = [&](const std::shared_ptr<const Var>& reference_var,
                        const std::shared_ptr<const Var>& follower_var,
                        const LinkId& link,
                        const Eigen::Vector3d& offset_axis,
                        double distance,
                        const Eigen::VectorXd& coeffs = Eigen::VectorXd::Ones(6),
                        const std::vector<Bounds>& bounds = std::vector<Bounds>(6, BoundZero)) {
    return CartOffsetConstraint(reference_var,
                                follower_var,
                                arm.manip,
                                link,
                                Eigen::Isometry3d::Identity(),
                                offset_axis,
                                distance,
                                coeffs,
                                bounds);
  };

  EXPECT_NO_THROW(make(reference, follower, arm.tool, axis, -0.02));

  // A state tied to itself
  EXPECT_THROW(make(reference, reference, arm.tool, axis, -0.02), std::runtime_error);

  // A variable that does not hold the joint values of the group
  auto short_node = std::make_unique<Node>("Short");
  const auto short_var = short_node->addVar(
      "position", std::vector<std::string>{ "a", "b" }, Eigen::VectorXd::Zero(2), std::vector<Bounds>(2, NoBound));
  EXPECT_THROW(make(short_var, follower, arm.tool, axis, -0.02), std::runtime_error);
  EXPECT_THROW(make(reference, short_var, arm.tool, axis, -0.02), std::runtime_error);

  // A link the group does not have, and one its joints do not move
  EXPECT_THROW(make(reference, follower, LinkId("no_such_link"), axis, -0.02), std::runtime_error);
  EXPECT_THROW(make(reference, follower, arm.fixed_link, axis, -0.02), std::runtime_error);

  // An axis without a direction
  for (const Eigen::Vector3d& bad_axis : { Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, nan, 1.0) })
    EXPECT_THROW(make(reference, follower, arm.tool, bad_axis, -0.02), std::runtime_error);

  // A distance that is not a number
  for (const double bad_distance : { inf, nan })
    EXPECT_THROW(make(reference, follower, arm.tool, axis, bad_distance), std::runtime_error);

  // Coefficients and bounds for other than six rows
  EXPECT_THROW(make(reference, follower, arm.tool, axis, -0.02, Eigen::VectorXd::Ones(5)), std::runtime_error);
  EXPECT_THROW(
      make(reference, follower, arm.tool, axis, -0.02, Eigen::VectorXd::Ones(6), std::vector<Bounds>(5, BoundZero)),
      std::runtime_error);

  // Coefficients must be finite and non-negative
  for (const double bad : { -1.0, inf, nan })
  {
    Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(6);
    coeffs[2] = bad;
    EXPECT_THROW(make(reference, follower, arm.tool, axis, -0.02, coeffs), std::runtime_error);
  }
}

/** @brief Check the jacobian with either state first, on the link and off it, on both arms */
TEST_F(CartesianOffsetConstraintUnit, Jacobian)  // NOLINT
{
  const std::vector<std::pair<Eigen::Vector3d, double>> offsets{ { Eigen::Vector3d::UnitZ(), -0.02 },
                                                                 { Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0, 0.3 } };
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    for (const bool reference_first : { true, false })
    {
      const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
      const std::shared_ptr<const Var>& reference = trajectory.vars[reference_first ? 0 : 1];
      const std::shared_ptr<const Var>& follower = trajectory.vars[reference_first ? 1 : 0];
      const std::string what = arm.manip->getName() + (reference_first ? ", reference first" : ", follower first");
      for (const Eigen::Isometry3d& frame_offset : frameOffsets())
      {
        for (const auto& [axis, distance] : offsets)
        {
          CartOffsetConstraint constraint(reference, follower, arm.manip, arm.tool, frame_offset, axis, distance);
          for (const Eigen::VectorXd& reference_state : referenceStates(dof))
            for (const Eigen::VectorXd& offset : followerOffsets(dof))
              expectJacobian(
                  constraint, trajectory, *reference, reference_state, *follower, reference_state + offset, what);
        }
      }
    }
  }
}

/** @brief Check the jacobian of translation rows alone, of one rotation row, and of rows with range bounds */
TEST_F(CartesianOffsetConstraintUnit, JacobianSomeRows)  // NOLINT
{
  struct Rows
  {
    Eigen::VectorXd coeffs;
    std::vector<Bounds> bounds;
    RangeBoundHandling handling;
    int count;
  };
  const std::vector<Bounds> equalities(6, BoundZero);
  std::vector<Bounds> ranges(6, BoundZero);
  ranges[1] = Bounds(-0.1, 0.2);
  ranges[4] = Bounds(-0.3, 0.1);
  const Eigen::VectorXd translation = (Eigen::VectorXd(6) << 1, 1, 1, 0, 0, 0).finished();
  const Eigen::VectorXd one_rotation = (Eigen::VectorXd(6) << 0, 0, 0, 1, 0, 0).finished();
  const Eigen::VectorXd all_but_one = (Eigen::VectorXd(6) << 1, 1, 1, 0, 1, 1).finished();
  const std::vector<Rows> cases{ { translation, equalities, RangeBoundHandling::kSplitToTwoInequalities, 3 },
                                 { one_rotation, equalities, RangeBoundHandling::kSplitToTwoInequalities, 1 },
                                 { all_but_one, ranges, RangeBoundHandling::kKeepAsIs, 5 },
                                 { all_but_one, ranges, RangeBoundHandling::kSplitToTwoInequalities, 7 } };

  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& reference = trajectory.vars[1];
    const std::shared_ptr<const Var>& follower = trajectory.vars[0];
    const Eigen::VectorXd reference_state = referenceStates(dof)[1];
    const Eigen::VectorXd follower_state = reference_state + followerOffsets(dof)[2];
    for (const Rows& rows : cases)
    {
      CartOffsetConstraint constraint(reference,
                                      follower,
                                      arm.manip,
                                      arm.tool,
                                      frameOffsets()[1],
                                      Eigen::Vector3d::UnitZ(),
                                      -0.02,
                                      rows.coeffs,
                                      rows.bounds,
                                      "CartOffset",
                                      rows.handling);
      ASSERT_EQ(constraint.getRows(), rows.count);
      expectJacobian(constraint,
                     trajectory,
                     *reference,
                     reference_state,
                     *follower,
                     follower_state,
                     arm.manip->getName() + ", rows " + std::to_string(rows.count));
    }
  }
}

/** @brief Check the values and the jacobian where the two frames coincide */
TEST_F(CartesianOffsetConstraintUnit, JacobianZeroDistanceAtEqualStates)  // NOLINT
{
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& reference = trajectory.vars[1];
    const std::shared_ptr<const Var>& follower = trajectory.vars[0];
    CartOffsetConstraint constraint(
        reference, follower, arm.manip, arm.tool, frameOffsets()[1], Eigen::Vector3d::UnitZ(), 0.0);
    for (const Eigen::VectorXd& state : referenceStates(dof))
    {
      EXPECT_LT(constraint.calcValues(state, state).cwiseAbs().maxCoeff(), VALUE_TOL) << arm.manip->getName();
      expectJacobian(constraint, trajectory, *reference, state, *follower, state, arm.manip->getName());
    }
  }
}

/** @brief Check that only the columns of the two states are filled when other states lie between them */
TEST_F(CartesianOffsetConstraintUnit, JacobianWithOtherStates)  // NOLINT
{
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 4);
    const std::shared_ptr<const Var>& reference = trajectory.vars[3];
    const std::shared_ptr<const Var>& follower = trajectory.vars[1];
    CartOffsetConstraint constraint(
        reference, follower, arm.manip, arm.tool, frameOffsets()[1], Eigen::Vector3d::UnitZ(), -0.02);
    const Eigen::VectorXd reference_state = referenceStates(dof)[2];
    expectJacobian(constraint,
                   trajectory,
                   *reference,
                   reference_state,
                   *follower,
                   reference_state + followerOffsets(dof)[1],
                   arm.manip->getName());

    const Eigen::MatrixXd jacobian = constraint.getJacobian().toDense();
    for (const std::size_t other : { std::size_t{ 0 }, std::size_t{ 2 } })
      EXPECT_EQ(jacobian.middleCols(trajectory.vars[other]->getIndex(), dof).cwiseAbs().maxCoeff(), 0.0)
          << arm.manip->getName() << ", state " << other;
  }
}

/** @brief Check the rotation rows and the jacobian with the follower frame turned 3.1 rad from the reference frame */
TEST_F(CartesianOffsetConstraintUnit, JacobianNearHalfTurn)  // NOLINT
{
  constexpr double turn = 3.1;
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& reference = trajectory.vars[1];
    const std::shared_ptr<const Var>& follower = trajectory.vars[0];
    CartOffsetConstraint constraint(
        reference, follower, arm.manip, arm.tool, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), -0.02);

    // The last joint turns the link of the frame about its own axis
    const Eigen::VectorXd reference_state = referenceStates(dof)[0];
    Eigen::VectorXd follower_state = reference_state;
    follower_state[dof - 1] += turn;

    const Eigen::VectorXd values = constraint.calcValues(reference_state, follower_state);
    ASSERT_TRUE(values.allFinite());
    EXPECT_NEAR(values.tail(3).norm(), turn, VALUE_TOL) << arm.manip->getName();
    expectJacobian(
        constraint, trajectory, *reference, reference_state, *follower, follower_state, arm.manip->getName());
  }
}

/** @brief Check the jacobian for a frame on a link that only some of the joints move */
TEST_F(CartesianOffsetConstraintUnit, JacobianMidChainLink)  // NOLINT
{
  const Arm arm = pr2();
  const Eigen::Index dof = arm.manip->numJoints();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  const std::shared_ptr<const Var>& reference = trajectory.vars[1];
  const std::shared_ptr<const Var>& follower = trajectory.vars[0];
  CartOffsetConstraint constraint(
      reference, follower, arm.manip, LinkId("r_forearm_link"), frameOffsets()[1], Eigen::Vector3d::UnitZ(), -0.02);
  const Eigen::VectorXd reference_state = referenceStates(dof)[1];
  expectJacobian(constraint,
                 trajectory,
                 *reference,
                 reference_state,
                 *follower,
                 reference_state + followerOffsets(dof)[2],
                 "r_forearm_link");

  // The two wrist joints do not move the link
  const Eigen::MatrixXd jacobian = constraint.getJacobian().toDense();
  for (const std::shared_ptr<const Var>& var : { reference, follower })
    EXPECT_EQ(jacobian.middleCols(var->getIndex() + dof - 2, 2).cwiseAbs().maxCoeff(), 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
