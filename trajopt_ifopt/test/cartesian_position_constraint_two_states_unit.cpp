/**
 * @file cartesian_position_constraint_two_states_unit.cpp
 * @brief The cartesian position constraint unit tests with the two frames in two states
 *
 * @author Roelof Oomen
 * @date October 17, 2026
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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

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
/** The step of the numeric jacobian, which is the forward difference of the values with it */
constexpr double NUMERIC_STEP = 1e-5;

/** A joint group with the link the tied frame is on, a link only some of its joints move and one none of them move */
struct Arm
{
  JointGroup::ConstPtr manip;
  LinkId tool;
  LinkId middle_link;
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

/** The frame on the link itself, and shifted and turned from it about each axis and about an oblique one */
std::vector<Eigen::Isometry3d> frameOffsets()
{
  return { Eigen::Isometry3d::Identity(),
           Eigen::Translation3d(0.05, 0.1, -0.2) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()),
           Eigen::Translation3d(-0.03, 0.02, 0.1) * Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()),
           Eigen::Translation3d(0.02, -0.04, 0.03) * Eigen::AngleAxisd(-0.6, Eigen::Vector3d::UnitZ()),
           Eigen::Translation3d(0.04, 0.01, -0.05) * Eigen::AngleAxisd(1.1, Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0) };
}

/** The frame offset followed by a translation along an axis of the frame */
Eigen::Isometry3d axisOffset(const Eigen::Isometry3d& frame_offset, const Eigen::Vector3d& unit_axis, double distance)
{
  return frame_offset * Eigen::Translation3d(distance * unit_axis);
}

/** The follower frame held at a distance along an axis of the frame the reference reaches */
CartPosConstraint makeTie(const Arm& arm,
                          const std::shared_ptr<const Var>& follower,
                          const std::shared_ptr<const Var>& reference,
                          const Eigen::Isometry3d& frame_offset,
                          const Eigen::Vector3d& unit_axis,
                          double distance)
{
  return {
    follower, reference, arm.manip, arm.tool, arm.tool, frame_offset, axisOffset(frame_offset, unit_axis, distance)
  };
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
      arm.manip->calcFwdKin(reference_state).at(arm.tool) * axisOffset(frame_offset, unit_axis, distance);
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

/**
 * Compare the analytic jacobian at the two states with the central difference of the values over every variable of the
 * set, and the numeric jacobian with their forward difference at its step
 */
void expectJacobian(CartPosConstraint& constraint,
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
    return constraint.calcValues(v.segment(follower.getIndex(), dof), v.segment(reference.getIndex(), dof));
  };
  const Eigen::MatrixXd central = calcCentralNumJac(values, x);
  const Eigen::MatrixXd forward = calcForwardNumJac(values, x, NUMERIC_STEP).toDense();

  for (const bool numeric : { false, true })
  {
    constraint.use_numeric_differentiation = numeric;
    const Jacobian jacobian = constraint.getJacobian();
    ASSERT_EQ(jacobian.rows(), constraint.getRows()) << what;
    ASSERT_EQ(jacobian.cols(), x.size()) << what;
    EXPECT_EQ(jacobian.nonZeros(), constraint.getNonZeros()) << what;

    // Each row lists its columns in ascending order, which Eigen checks only with assertions on
    for (Eigen::Index row = 0; row < jacobian.outerSize(); ++row)
    {
      Eigen::Index previous = -1;
      for (Jacobian::InnerIterator it(jacobian, row); it; ++it)
      {
        EXPECT_GT(it.col(), previous) << what << ", row " << row;
        previous = it.col();
      }
    }

    EXPECT_LT(maxDifference(jacobian.toDense(), numeric ? forward : central), JAC_TOL)
        << what << (numeric ? ", numeric" : ", analytic");
  }
}
}  // namespace

/**
 * The arms the tests run on. An environment is loaded when its arm is first asked for and kept for the suite. One that
 * fails to load throws, which fails the test that asked for it, and is not kept.
 */
class CartesianPositionTwoStatesUnit : public testing::Test
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
      auto env = std::make_shared<tesseract::environment::Environment>();
      if (!env->init(urdf_file, srdf_file, std::make_shared<tesseract::common::GeneralResourceLocator>()))
        throw std::runtime_error("Failed to load the PR2");
      pr2_env = std::move(env);
    }
    return { pr2_env->getJointGroup("right_arm"), "r_gripper_tool_frame", "r_forearm_link", "base_footprint" };
  }

  /** An arm whose joints have rotated origins */
  static Arm iiwa()
  {
    if (iiwa_env == nullptr)
    {
      const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
      const auto resource = locator->locateResource("package://tesseract/support/urdf/iiwa7.urdf");
      if (resource == nullptr)
        throw std::runtime_error("Failed to find the iiwa");
      auto env = std::make_shared<tesseract::environment::Environment>();
      if (!env->init(std::filesystem::path(resource->getFilePath()), locator))
        throw std::runtime_error("Failed to load the iiwa");
      iiwa_env = std::move(env);
    }
    const std::vector<tesseract::common::JointId> joints{ "joint_1", "joint_2", "joint_3", "joint_4",
                                                          "joint_5", "joint_6", "joint_7" };
    return { iiwa_env->getJointGroup("manipulator", joints), "ikfast_tcp_link", "link_3", "base" };
  }

  static std::vector<Arm> arms() { return { pr2(), iiwa() }; }

private:
  // NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables): loaded once per test suite, reset after it
  static inline tesseract::environment::Environment::Ptr pr2_env;
  static inline tesseract::environment::Environment::Ptr iiwa_env;
  // NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
};

/** @brief With both states equal the follower frame sits at minus the offset in the reference frame, unturned */
TEST_F(CartesianPositionTwoStatesUnit, ValuesAtEqualStates)  // NOLINT
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
          const CartPosConstraint constraint =
              makeTie(arm, trajectory.vars[0], trajectory.vars[1], frame_offset, axis, distance);
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
TEST_F(CartesianPositionTwoStatesUnit, ValuesAgainstForwardKinematics)  // NOLINT
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
      const CartPosConstraint constraint =
          makeTie(arm, trajectory.vars[0], trajectory.vars[1], frame_offset, axis, distance);
      for (const Eigen::VectorXd& reference_state : referenceStates(dof))
      {
        for (const Eigen::VectorXd& offset : followerOffsets(dof))
        {
          const Eigen::VectorXd follower_state = reference_state + offset;
          const Eigen::VectorXd expected =
              expectedValues(arm, frame_offset, axis, distance, reference_state, follower_state);
          EXPECT_LT(maxDifference(constraint.calcValues(follower_state, reference_state), expected), VALUE_TOL)
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
 * @brief With the reference state held, the values and the follower columns are those of the one-state form on the
 * follower whose target is the offset reference frame, given on a fixed link
 */
TEST_F(CartesianPositionTwoStatesUnit, AgreesWithOneStateAtFixedReference)  // NOLINT
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
      CartPosConstraint tie = makeTie(arm, follower, reference, frame_offset, axis, distance);
      tie.linkWithVariables(trajectory.variables);
      for (const Eigen::VectorXd& reference_state : referenceStates(dof))
      {
        const auto reference_transforms = arm.manip->calcFwdKin(reference_state);
        const Eigen::Isometry3d target_offset = reference_transforms.at(arm.fixed_link).inverse() *
                                                reference_transforms.at(arm.tool) *
                                                axisOffset(frame_offset, axis, distance);
        CartPosConstraint pose_constraint(follower, arm.manip, arm.tool, arm.fixed_link, frame_offset, target_offset);
        pose_constraint.linkWithVariables(trajectory.variables);

        for (const Eigen::VectorXd& offset : followerOffsets(dof))
        {
          trajectory.variables->setVariables(
              stack(trajectory, *reference, reference_state, *follower, reference_state + offset));
          EXPECT_LT(maxDifference(tie.getValues(), pose_constraint.getValues()), VALUE_TOL) << arm.manip->getName();

          for (const bool numeric : { false, true })
          {
            tie.use_numeric_differentiation = numeric;
            pose_constraint.use_numeric_differentiation = numeric;
            const Eigen::MatrixXd follower_columns = tie.getJacobian().toDense().middleCols(follower->getIndex(), dof);
            const Eigen::MatrixXd pose_columns =
                pose_constraint.getJacobian().toDense().middleCols(follower->getIndex(), dof);
            EXPECT_LT(maxDifference(follower_columns, pose_columns), JAC_TOL)
                << arm.manip->getName() << (numeric ? ", numeric" : ", analytic");
          }
        }
      }
    }
  }
}

/** @brief One variable given for both frames is the one-state form */
TEST_F(CartesianPositionTwoStatesUnit, OneVariableTwiceIsOneState)  // NOLINT
{
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& var = trajectory.vars[1];
    const Eigen::Isometry3d target_offset = axisOffset(frameOffsets()[1], Eigen::Vector3d::UnitZ(), -0.02);
    // Both frames move, and either one alone
    const std::vector<std::pair<LinkId, LinkId>> links{
      { arm.tool, arm.tool }, { arm.tool, arm.middle_link }, { arm.tool, arm.fixed_link }, { arm.fixed_link, arm.tool }
    };
    for (const auto& [source_link, target_link] : links)
    {
      CartPosConstraint twice(var, var, arm.manip, source_link, target_link, frameOffsets()[1], target_offset);
      CartPosConstraint once(var, arm.manip, source_link, target_link, frameOffsets()[1], target_offset);
      twice.linkWithVariables(trajectory.variables);
      once.linkWithVariables(trajectory.variables);
      EXPECT_EQ(twice.getNonZeros(), once.getNonZeros());
      for (const Eigen::VectorXd& state : referenceStates(dof))
      {
        trajectory.variables->setVariables(stack(trajectory, *var, state, *trajectory.vars[0], state));
        EXPECT_EQ(twice.getValues(), once.getValues())
            << arm.manip->getName() << ", " << source_link.name() << " to " << target_link.name();
        for (const bool numeric : { false, true })
        {
          twice.use_numeric_differentiation = numeric;
          once.use_numeric_differentiation = numeric;
          EXPECT_EQ(twice.getJacobian().toDense(), once.getJacobian().toDense())
              << arm.manip->getName() << ", " << source_link.name() << " to " << target_link.name()
              << (numeric ? ", numeric" : ", analytic");
        }
      }
    }
  }
}

/** @brief A frame on a link the joints do not move fills no columns of its state */
TEST_F(CartesianPositionTwoStatesUnit, StaticFrameFillsNoColumns)  // NOLINT
{
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& reference = trajectory.vars[1];
    const std::shared_ptr<const Var>& follower = trajectory.vars[0];
    const Eigen::Isometry3d offset = frameOffsets()[1];

    // The static frame is the target in the reference state, then the source in the follower state
    for (const bool static_target : { true, false })
    {
      const LinkId& source_link = static_target ? arm.tool : arm.fixed_link;
      const LinkId& target_link = static_target ? arm.fixed_link : arm.tool;
      const Var& static_var = static_target ? *reference : *follower;
      CartPosConstraint two_states(follower, reference, arm.manip, source_link, target_link, offset, offset);
      CartPosConstraint one_state(
          static_target ? follower : reference, arm.manip, source_link, target_link, offset, offset);
      two_states.linkWithVariables(trajectory.variables);
      one_state.linkWithVariables(trajectory.variables);
      const std::string what = arm.manip->getName() + (static_target ? ", static target" : ", static source");
      EXPECT_EQ(two_states.getNonZeros(), 6 * dof) << what;

      const Eigen::VectorXd reference_state = referenceStates(dof)[1];
      const Eigen::VectorXd follower_state = reference_state + followerOffsets(dof)[2];
      trajectory.variables->setVariables(stack(trajectory, *reference, reference_state, *follower, follower_state));
      EXPECT_EQ(two_states.getValues(), one_state.getValues()) << what;
      for (const bool numeric : { false, true })
      {
        two_states.use_numeric_differentiation = numeric;
        one_state.use_numeric_differentiation = numeric;
        const Jacobian jacobian = two_states.getJacobian();
        EXPECT_EQ(jacobian.nonZeros(), 6 * dof) << what;
        EXPECT_EQ(jacobian.toDense(), one_state.getJacobian().toDense()) << what;
        EXPECT_EQ(jacobian.toDense().middleCols(static_var.getIndex(), dof).cwiseAbs().maxCoeff(), 0.0) << what;
      }
      expectJacobian(two_states, trajectory, *reference, reference_state, *follower, follower_state, what);
    }
  }
}

/** @brief Without coefficients and bounds: six rows, unit coefficients, equalities */
TEST_F(CartesianPositionTwoStatesUnit, DefaultRows)  // NOLINT
{
  const Arm arm = pr2();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  CartPosConstraint constraint = makeTie(
      arm, trajectory.vars[0], trajectory.vars[1], Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), -0.02);
  EXPECT_EQ(constraint.getName(), "CartPos");
  EXPECT_EQ(constraint.getRows(), 6);
  EXPECT_EQ(constraint.update(), 6);
  EXPECT_EQ(constraint.getNonZeros(), 12 * arm.manip->numJoints());
  EXPECT_TRUE(constraint.getCoefficients().isApprox(Eigen::VectorXd::Ones(6)));
  expectBounds(constraint.getBounds(), std::vector<std::pair<double, double>>(6, { 0.0, 0.0 }));
  EXPECT_TRUE(constraint.getTargetPose().isApprox(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, -0.02))));
}

/** @brief A row without a coefficient is dropped; a range bound is kept as one row or split in two */
TEST_F(CartesianPositionTwoStatesUnit, RowsCoefficientsBounds)  // NOLINT
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
    return CartPosConstraint(trajectory.vars[0],
                             trajectory.vars[1],
                             coeffs,
                             bounds,
                             arm.manip,
                             arm.tool,
                             arm.tool,
                             Eigen::Isometry3d::Identity(),
                             axisOffset(Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), -0.02),
                             "Rows",
                             handling);
  };
  const auto expectRows = [&](CartPosConstraint& constraint, const std::vector<int>& rows) {
    EXPECT_EQ(constraint.getRows(), static_cast<int>(rows.size()));
    EXPECT_EQ(constraint.update(), static_cast<int>(rows.size()));
    EXPECT_EQ(constraint.getNonZeros(), 2 * dof * static_cast<Eigen::Index>(rows.size()));
    const Eigen::VectorXd values = constraint.calcValues(follower_state, reference_state);
    const Eigen::VectorXd kept_coeffs = constraint.getCoefficients();
    ASSERT_EQ(values.size(), static_cast<Eigen::Index>(rows.size()));
    ASSERT_EQ(kept_coeffs.size(), static_cast<Eigen::Index>(rows.size()));
    for (std::size_t i = 0; i < rows.size(); ++i)
    {
      EXPECT_NEAR(values[static_cast<Eigen::Index>(i)], all_values[rows[i]], VALUE_TOL) << "row " << i;
      EXPECT_EQ(kept_coeffs[static_cast<Eigen::Index>(i)], coeffs[rows[i]]) << "row " << i;
    }
  };

  CartPosConstraint kept = make(RangeBoundHandling::kKeepAsIs);
  EXPECT_EQ(kept.getName(), "Rows");
  expectRows(kept, { 0, 1, 3, 4, 5 });
  expectBounds(kept.getBounds(), { { 0, 0 }, { -0.1, 0.2 }, { 0.5, inf }, { -0.3, 0.1 }, { 0, 0 } });

  CartPosConstraint split = make(RangeBoundHandling::kSplitToTwoInequalities);
  expectRows(split, { 0, 1, 1, 3, 4, 4, 5 });
  expectBounds(split.getBounds(),
               { { 0, 0 }, { -0.1, inf }, { -inf, 0.2 }, { 0.5, inf }, { -0.3, inf }, { -inf, 0.1 }, { 0, 0 } });
}

/** @brief Without a coefficient on any row the set has no rows */
TEST_F(CartesianPositionTwoStatesUnit, NoRows)  // NOLINT
{
  const Arm arm = pr2();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  CartPosConstraint constraint(trajectory.vars[0],
                               trajectory.vars[1],
                               Eigen::VectorXd::Zero(6),
                               std::vector<Bounds>(6, BoundZero),
                               arm.manip,
                               arm.tool,
                               arm.tool);
  constraint.linkWithVariables(trajectory.variables);

  EXPECT_EQ(constraint.getRows(), 0);
  EXPECT_EQ(constraint.getNonZeros(), 0);
  EXPECT_EQ(constraint.getValues().size(), 0);
  EXPECT_EQ(constraint.getCoefficients().size(), 0);
  EXPECT_TRUE(constraint.getBounds().empty());
  for (const bool numeric : { false, true })
  {
    constraint.use_numeric_differentiation = numeric;
    const Jacobian jacobian = constraint.getJacobian();
    EXPECT_EQ(jacobian.rows(), 0);
    EXPECT_EQ(jacobian.cols(), trajectory.variables->getRows());
  }
}

/** @brief Arguments that describe no constraint are refused */
TEST_F(CartesianPositionTwoStatesUnit, RejectsInvalidArguments)  // NOLINT
{
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Arm arm = pr2();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  const auto make = [&](const LinkId& source_link,
                        const LinkId& target_link,
                        const Eigen::VectorXd& coeffs = Eigen::VectorXd::Ones(6),
                        const std::vector<Bounds>& bounds = std::vector<Bounds>(6, BoundZero)) {
    return CartPosConstraint(
        trajectory.vars[0], trajectory.vars[1], coeffs, bounds, arm.manip, source_link, target_link);
  };

  EXPECT_NO_THROW(make(arm.tool, arm.tool));

  // A link the group does not have, and two links its joints do not move
  EXPECT_THROW(make(LinkId("no_such_link"), arm.tool), std::runtime_error);
  EXPECT_THROW(make(arm.tool, LinkId("no_such_link")), std::runtime_error);
  EXPECT_THROW(make(arm.fixed_link, arm.fixed_link), std::runtime_error);

  // Coefficients and bounds for other than six rows
  EXPECT_THROW(make(arm.tool, arm.tool, Eigen::VectorXd::Ones(5)), std::runtime_error);
  EXPECT_THROW(make(arm.tool, arm.tool, Eigen::VectorXd::Ones(6), std::vector<Bounds>(5, BoundZero)),
               std::runtime_error);

  // Coefficients must be finite and non-negative
  for (const double bad : { -1.0, inf, nan })
  {
    Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(6);
    coeffs[2] = bad;
    EXPECT_THROW(make(arm.tool, arm.tool, coeffs), std::runtime_error);
  }
}

/** @brief Check the jacobian with either state first, on the link and off it, on both arms */
TEST_F(CartesianPositionTwoStatesUnit, Jacobian)  // NOLINT
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
          CartPosConstraint constraint = makeTie(arm, follower, reference, frame_offset, axis, distance);
          for (const Eigen::VectorXd& reference_state : referenceStates(dof))
            for (const Eigen::VectorXd& offset : followerOffsets(dof))
              expectJacobian(
                  constraint, trajectory, *reference, reference_state, *follower, reference_state + offset, what);
        }
      }
    }
  }
}

/** @brief Check the jacobian with the source frame on the tool and the target frame on a link nearer the base */
TEST_F(CartesianPositionTwoStatesUnit, JacobianTwoLinks)  // NOLINT
{
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    for (const bool reference_first : { true, false })
    {
      const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
      const std::shared_ptr<const Var>& reference = trajectory.vars[reference_first ? 0 : 1];
      const std::shared_ptr<const Var>& follower = trajectory.vars[reference_first ? 1 : 0];

      // The target frame sits on the middle link where the source frame is at the reference state
      const Eigen::VectorXd reference_state = referenceStates(dof)[1];
      const auto transforms = arm.manip->calcFwdKin(reference_state);
      const Eigen::Isometry3d target_offset =
          transforms.at(arm.middle_link).inverse() * transforms.at(arm.tool) * frameOffsets()[1];
      CartPosConstraint constraint(
          follower, reference, arm.manip, arm.tool, arm.middle_link, frameOffsets()[1], target_offset);
      for (const Eigen::VectorXd& offset : followerOffsets(dof))
        expectJacobian(constraint,
                       trajectory,
                       *reference,
                       reference_state,
                       *follower,
                       reference_state + offset,
                       arm.manip->getName() + (reference_first ? ", reference first" : ", follower first"));
    }
  }
}

/** @brief Check the jacobian of translation rows alone, of one rotation row, and of rows with range bounds */
TEST_F(CartesianPositionTwoStatesUnit, JacobianSomeRows)  // NOLINT
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
      CartPosConstraint constraint(follower,
                                   reference,
                                   rows.coeffs,
                                   rows.bounds,
                                   arm.manip,
                                   arm.tool,
                                   arm.tool,
                                   frameOffsets()[1],
                                   axisOffset(frameOffsets()[1], Eigen::Vector3d::UnitZ(), -0.02),
                                   "CartPos",
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
TEST_F(CartesianPositionTwoStatesUnit, JacobianZeroDistanceAtEqualStates)  // NOLINT
{
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& reference = trajectory.vars[1];
    const std::shared_ptr<const Var>& follower = trajectory.vars[0];
    CartPosConstraint constraint = makeTie(arm, follower, reference, frameOffsets()[1], Eigen::Vector3d::UnitZ(), 0.0);
    for (const Eigen::VectorXd& state : referenceStates(dof))
    {
      EXPECT_LT(constraint.calcValues(state, state).cwiseAbs().maxCoeff(), VALUE_TOL) << arm.manip->getName();
      expectJacobian(constraint, trajectory, *reference, state, *follower, state, arm.manip->getName());
    }
  }
}

/** @brief Check that only the columns of the two states are filled when other states lie between them */
TEST_F(CartesianPositionTwoStatesUnit, JacobianWithOtherStates)  // NOLINT
{
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 4);
    const std::shared_ptr<const Var>& reference = trajectory.vars[3];
    const std::shared_ptr<const Var>& follower = trajectory.vars[1];
    CartPosConstraint constraint =
        makeTie(arm, follower, reference, frameOffsets()[1], Eigen::Vector3d::UnitZ(), -0.02);
    const Eigen::VectorXd reference_state = referenceStates(dof)[2];
    expectJacobian(constraint,
                   trajectory,
                   *reference,
                   reference_state,
                   *follower,
                   reference_state + followerOffsets(dof)[1],
                   arm.manip->getName());

    for (const bool numeric : { false, true })
    {
      constraint.use_numeric_differentiation = numeric;
      const Eigen::MatrixXd jacobian = constraint.getJacobian().toDense();
      for (const std::size_t other : { std::size_t{ 0 }, std::size_t{ 2 } })
        EXPECT_EQ(jacobian.middleCols(trajectory.vars[other]->getIndex(), dof).cwiseAbs().maxCoeff(), 0.0)
            << arm.manip->getName() << ", state " << other << (numeric ? ", numeric" : ", analytic");
    }
  }
}

/** @brief Check the rotation rows and the jacobian with the follower frame turned 3.1 rad from the reference frame */
TEST_F(CartesianPositionTwoStatesUnit, JacobianNearHalfTurn)  // NOLINT
{
  constexpr double turn = 3.1;
  for (const Arm& arm : arms())
  {
    const Eigen::Index dof = arm.manip->numJoints();
    const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
    const std::shared_ptr<const Var>& reference = trajectory.vars[1];
    const std::shared_ptr<const Var>& follower = trajectory.vars[0];
    CartPosConstraint constraint =
        makeTie(arm, follower, reference, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), -0.02);

    // The last joint turns the link of the frame about its own axis
    const Eigen::VectorXd reference_state = referenceStates(dof)[0];
    Eigen::VectorXd follower_state = reference_state;
    follower_state[dof - 1] += turn;

    const Eigen::VectorXd values = constraint.calcValues(follower_state, reference_state);
    ASSERT_TRUE(values.allFinite());
    EXPECT_NEAR(values.tail(3).norm(), turn, VALUE_TOL) << arm.manip->getName();
    expectJacobian(
        constraint, trajectory, *reference, reference_state, *follower, follower_state, arm.manip->getName());
  }
}

/** @brief Check the jacobian for a frame on a link that only some of the joints move */
TEST_F(CartesianPositionTwoStatesUnit, JacobianMidChainLink)  // NOLINT
{
  const Arm arm = pr2();
  const Eigen::Index dof = arm.manip->numJoints();
  const Trajectory trajectory = makeTrajectory(*arm.manip, 2);
  const std::shared_ptr<const Var>& reference = trajectory.vars[1];
  const std::shared_ptr<const Var>& follower = trajectory.vars[0];
  CartPosConstraint constraint(follower,
                               reference,
                               arm.manip,
                               arm.middle_link,
                               arm.middle_link,
                               frameOffsets()[1],
                               axisOffset(frameOffsets()[1], Eigen::Vector3d::UnitZ(), -0.02));
  const Eigen::VectorXd reference_state = referenceStates(dof)[1];
  expectJacobian(constraint,
                 trajectory,
                 *reference,
                 reference_state,
                 *follower,
                 reference_state + followerOffsets(dof)[2],
                 arm.middle_link.name());

  // The two wrist joints do not move the link
  for (const bool numeric : { false, true })
  {
    constraint.use_numeric_differentiation = numeric;
    const Eigen::MatrixXd jacobian = constraint.getJacobian().toDense();
    for (const std::shared_ptr<const Var>& var : { reference, follower })
      EXPECT_EQ(jacobian.middleCols(var->getIndex() + dof - 2, 2).cwiseAbs().maxCoeff(), 0.0)
          << (numeric ? "numeric" : "analytic");
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
