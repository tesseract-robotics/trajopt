/**
 * @file cartesian_axis_constraint_unit.cpp
 * @brief Unit tests for the cartesian axis cone and align constraints
 *
 * @author Jelle Feringa
 * @date October 1, 2026
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
#include <algorithm>
#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_axis_align_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_axis_cone_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>

using namespace trajopt_ifopt;

namespace
{
const std::string TOOL{ "r_gripper_tool_frame" };  // active, tip of right_arm
const std::string ELBOW{ "r_elbow_flex_link" };    // active, mid-chain
const std::string BASE{ "base_footprint" };        // static
const std::string TORSO{ "torso_lift_link" };      // static, chain base of right_arm

/** Central difference step [rad]: truncation O(h^2) ~ 1e-12 and rounding O(eps/h) ~ 1e-10 balance near 1e-6. */
constexpr double FD_STEP = 1e-6;
/** Agreement of analytic and central-difference jacobians, a few orders above their ~1e-10 error. */
constexpr double JAC_TOL = 1e-7;
/** Agreement of computed angles and projections with closed-form values, double rounding on unit vectors. */
constexpr double VALUE_TOL = 1e-12;

const double INF = std::numeric_limits<double>::infinity();
const double NAN_D = std::numeric_limits<double>::quiet_NaN();

/** Joint configurations away from singularities; values in rad within the right_arm limits. */
std::vector<Eigen::VectorXd> testConfigurations()
{
  Eigen::VectorXd q0(7), q1(7), q2(7);
  q0 << 1.0, 0.2, 1.0, -1.0, 1.0, -1.0, 1.0;
  q1 << -0.5, 0.3, -1.2, -0.6, 2.0, -0.4, -2.5;
  q2 << 0.3, -0.2, 0.7, -1.5, -1.3, -1.1, 0.4;
  return { q0, q1, q2 };
}

Eigen::MatrixXd centralDifference(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
                                  const Eigen::VectorXd& q)
{
  const Eigen::Index rows = f(q).size();
  Eigen::MatrixXd jac(rows, q.size());
  for (Eigen::Index i = 0; i < q.size(); ++i)
  {
    Eigen::VectorXd qp = q;
    Eigen::VectorXd qm = q;
    qp(i) += FD_STEP;
    qm(i) -= FD_STEP;
    jac.col(i) = (f(qp) - f(qm)) / (2 * FD_STEP);
  }
  return jac;
}

/** A unit vector perpendicular to @p v, rotated by @p turn about @p v so tests do not depend on one choice. */
Eigen::Vector3d perpendicular(const Eigen::Vector3d& v, double turn)
{
  return Eigen::AngleAxisd(turn, v) * v.unitOrthogonal();
}

struct LinkPair
{
  std::string source;
  std::string target;
};

/** The three active cases: source active, target active, both active. */
std::vector<LinkPair> linkPairs() { return { { TOOL, BASE }, { BASE, TOOL }, { TOOL, ELBOW } }; }
}  // namespace

class CartesianAxisConstraintUnit : public testing::Test
{
public:
  tesseract::kinematics::JointGroup::ConstPtr kin_group;
  std::shared_ptr<const Var> var;
  std::shared_ptr<Problem> nlp;
  Eigen::Index n_dof{ -1 };

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract::environment::Environment>();
    ASSERT_TRUE(env->init(urdf_file, srdf_file, locator));

    kin_group = env->getJointGroup("right_arm");
    n_dof = kin_group->numJoints();

    auto node = std::make_unique<Node>("Joint_Position_0");
    const std::vector<std::string> joint_names = tesseract::common::toNames(kin_group->getJointIds());
    var = node->addVar("position",
                       joint_names,
                       testConfigurations().front(),
                       std::vector<Bounds>(static_cast<std::size_t>(n_dof), NoBound));
    std::vector<std::unique_ptr<Node>> nodes;
    nodes.push_back(std::move(node));
    nlp = std::make_shared<Problem>(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));
  }

  /** World orientation of @p link at @p q */
  Eigen::Matrix3d rotation(const std::string& link, const Eigen::VectorXd& q) const
  {
    return kin_group->calcFwdKin(q).at(link).linear();
  }

  /** The source axis expressed in target link coordinates at @p q */
  Eigen::Vector3d relativeAxis(const LinkPair& links,
                               const Eigen::Vector3d& source_axis,
                               const Eigen::VectorXd& q) const
  {
    return rotation(links.target, q).transpose() * rotation(links.source, q) * source_axis.normalized();
  }

  /** dv/dq by central differences of relativeAxis: an oracle independent of CartAxisKinematics */
  Eigen::MatrixXd numericAxisJacobian(const LinkPair& links,
                                      const Eigen::Vector3d& source_axis,
                                      const Eigen::VectorXd& q) const
  {
    return centralDifference([&](const Eigen::VectorXd& x) { return relativeAxis(links, source_axis, x); }, q);
  }

  /** Largest rate at which the joints can move v perpendicular to v: top singular value of T dv/dq */
  double largestTangentRate(const LinkPair& links, const Eigen::Vector3d& source_axis, const Eigen::VectorXd& q) const
  {
    const Eigen::Vector3d v = relativeAxis(links, source_axis, q);
    Eigen::Matrix<double, 2, 3> tangent;
    tangent.row(0) = v.unitOrthogonal().transpose();
    tangent.row(1) = v.cross(v.unitOrthogonal()).transpose();
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(tangent * numericAxisJacobian(links, source_axis, q));
    return svd.singularValues()(0);
  }

  Eigen::MatrixXd analyticJacobian(const std::function<void(Jacobian&, const Eigen::VectorXd&)>& fill,
                                   Eigen::Index rows,
                                   const Eigen::VectorXd& q) const
  {
    Jacobian jac(rows, n_dof);
    fill(jac, q);
    return jac.toDense();
  }
};

////////////////////////////////////////////////////////////////////
// Cone
////////////////////////////////////////////////////////////////////

/** @brief The value is the angle between the axes in radians, over the whole range [0, pi] */
TEST_F(CartesianAxisConstraintUnit, ConeValueIsAngleBetweenAxes)  // NOLINT
{
  const Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
  for (const auto& links : linkPairs())
  {
    for (const auto& q : testConfigurations())
    {
      const Eigen::Vector3d v = relativeAxis(links, source_axis, q);
      for (const double angle : { 0.0, 1e-9, 0.3, M_PI / 2, 3.0, M_PI - 1e-9, M_PI })
      {
        const Eigen::Vector3d target_axis = Eigen::AngleAxisd(angle, perpendicular(v, 0.7)) * v;
        const CartAxisConeConstraint cnt(var, kin_group, links.source, source_axis, links.target, target_axis, 0.1);
        const Eigen::VectorXd value = cnt.calcValues(q);
        ASSERT_EQ(value.size(), 1);
        EXPECT_NEAR(value(0), angle, VALUE_TOL) << links.source << " -> " << links.target << " angle " << angle;
      }
    }
  }
}

/** @brief getValues reads the variable; one row bounded by (-inf, theta], weighted by coeff */
TEST_F(CartesianAxisConstraintUnit, ConeBoundsCoefficientsAndValues)  // NOLINT
{
  const CartAxisConeConstraint cnt(
      var, kin_group, TOOL, Eigen::Vector3d(0, 0, 2), BASE, Eigen::Vector3d(1, 0, 0), 0.25, 3.5);

  EXPECT_EQ(cnt.getRows(), 1);
  const std::vector<Bounds> bounds = cnt.getBounds();
  ASSERT_EQ(bounds.size(), 1);
  EXPECT_EQ(bounds[0].getLower(), -INF);
  EXPECT_DOUBLE_EQ(bounds[0].getUpper(), 0.25);
  EXPECT_DOUBLE_EQ(cnt.getHalfAngle(), 0.25);
  ASSERT_EQ(cnt.getCoefficients().size(), 1);
  EXPECT_DOUBLE_EQ(cnt.getCoefficients()(0), 3.5);
  EXPECT_TRUE(cnt.getTargetAxis().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_DOUBLE_EQ(cnt.getValues()(0), cnt.calcValues(var->value())(0));
}

/** @brief The analytic jacobian matches central differences for every active case */
TEST_F(CartesianAxisConstraintUnit, ConeJacobianMatchesCentralDifferences)  // NOLINT
{
  const Eigen::Vector3d source_axis(0.2, -0.3, 1.0);
  for (const auto& links : linkPairs())
  {
    for (const auto& q : testConfigurations())
    {
      const Eigen::Vector3d v = relativeAxis(links, source_axis, q);
      for (const double angle : { 0.05, 0.4, 1.5, 2.8 })
      {
        const Eigen::Vector3d target_axis = Eigen::AngleAxisd(angle, perpendicular(v, 1.9)) * v;
        const CartAxisConeConstraint cnt(var, kin_group, links.source, source_axis, links.target, target_axis, 0.1);

        const Eigen::MatrixXd numeric =
            centralDifference([&](const Eigen::VectorXd& x) { return cnt.calcValues(x); }, q);
        const Eigen::MatrixXd analytic =
            analyticJacobian([&](Jacobian& jac, const Eigen::VectorXd& x) { cnt.calcJacobianBlock(jac, x); }, 1, q);

        EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), JAC_TOL)
            << links.source << " -> " << links.target << " angle " << angle << "\nanalytic " << analytic
            << "\nnumeric  " << numeric;
      }
    }
  }
}

/** @brief getJacobian evaluates calcJacobianBlock at the variable values */
TEST_F(CartesianAxisConstraintUnit, ConeGetJacobianUsesVariableValues)  // NOLINT
{
  auto cnt = std::make_shared<CartAxisConeConstraint>(
      var, kin_group, TOOL, Eigen::Vector3d::UnitZ(), BASE, Eigen::Vector3d::UnitX(), 0.1);
  nlp->addConstraintSet(cnt);
  const Eigen::MatrixXd analytic = analyticJacobian(
      [&](Jacobian& jac, const Eigen::VectorXd& x) { cnt->calcJacobianBlock(jac, x); }, 1, var->value());
  EXPECT_TRUE(cnt->getJacobian().toDense().isApprox(analytic));
}

/** @brief At phi = 0 (strictly feasible) the jacobian is zero and finite */
TEST_F(CartesianAxisConstraintUnit, ConeJacobianAtAlignmentIsZero)  // NOLINT
{
  const Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
  for (const auto& links : linkPairs())
  {
    const Eigen::VectorXd q = testConfigurations().front();
    const CartAxisConeConstraint cnt(
        var, kin_group, links.source, source_axis, links.target, relativeAxis(links, source_axis, q), 0.1);
    const Eigen::MatrixXd analytic =
        analyticJacobian([&](Jacobian& jac, const Eigen::VectorXd& x) { cnt.calcJacobianBlock(jac, x); }, 1, q);
    EXPECT_TRUE(analytic.allFinite());
    EXPECT_EQ(analytic.cwiseAbs().maxCoeff(), 0.0);
  }
}

/** @brief At phi = pi the jacobian is a descent direction, so a solver can leave the antiparallel configuration */
TEST_F(CartesianAxisConstraintUnit, ConeJacobianAtAntiparallelIsDescentDirection)  // NOLINT
{
  // Step along -grad [rad]; small enough to stay in the linear regime, large enough to clear rounding at phi = pi.
  constexpr double step = 1e-4;
  const Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
  for (const auto& links : linkPairs())
  {
    const Eigen::VectorXd q = testConfigurations().front();
    const CartAxisConeConstraint cnt(
        var, kin_group, links.source, source_axis, links.target, -relativeAxis(links, source_axis, q), 0.1);
    ASSERT_NEAR(cnt.calcValues(q)(0), M_PI, VALUE_TOL);

    const Eigen::MatrixXd analytic =
        analyticJacobian([&](Jacobian& jac, const Eigen::VectorXd& x) { cnt.calcJacobianBlock(jac, x); }, 1, q);
    ASSERT_GT(analytic.norm(), 0.0);
    const Eigen::VectorXd q_step = q - step * analytic.row(0).transpose().normalized();
    EXPECT_LT(cnt.calcValues(q_step)(0), M_PI - step * 1e-3) << links.source << " -> " << links.target;

    // The chosen direction is the one the joints move v along fastest
    EXPECT_NEAR(analytic.norm(), largestTangentRate(links, source_axis, q), JAC_TOL);
  }
}

/**
 * @brief The angle row keeps its gradient as the tilt shrinks; a cosine row a_tgt . v >= cos(theta) loses a factor
 * sin(phi).
 *
 * d(cos phi)/dq = -sin(phi) dphi/dq, checked against central differences of cos(phi(q)), so at the boundary of a cone
 * of half angle theta the cosine form's gradient is sin(theta) times smaller.
 */
TEST_F(CartesianAxisConstraintUnit, ConeGradientDoesNotDegenerateForSmallAngles)  // NOLINT
{
  const Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
  const LinkPair links{ TOOL, BASE };
  const Eigen::VectorXd q = testConfigurations().front();
  const Eigen::Vector3d v = relativeAxis(links, source_axis, q);
  const Eigen::Vector3d tilt_axis = perpendicular(v, 0.3);

  std::vector<double> angle_gradient_norms;
  for (const double angle : { 1e-3, 1e-2, 1e-1, 0.5 })
  {
    const Eigen::Vector3d target_axis = Eigen::AngleAxisd(angle, tilt_axis) * v;
    const CartAxisConeConstraint cnt(var, kin_group, links.source, source_axis, links.target, target_axis, 0.1);
    const Eigen::MatrixXd angle_gradient =
        analyticJacobian([&](Jacobian& jac, const Eigen::VectorXd& x) { cnt.calcJacobianBlock(jac, x); }, 1, q);
    const Eigen::MatrixXd cosine_gradient =
        centralDifference([&](const Eigen::VectorXd& x) { return cnt.calcValues(x).array().cos().matrix().eval(); }, q);

    EXPECT_LT((cosine_gradient + std::sin(angle) * angle_gradient).cwiseAbs().maxCoeff(), JAC_TOL) << angle;
    angle_gradient_norms.push_back(angle_gradient.norm());
  }

  // Same tilt direction, so the angle gradient barely changes over three decades of angle
  const auto [min_norm, max_norm] = std::minmax_element(angle_gradient_norms.begin(), angle_gradient_norms.end());
  EXPECT_LT(*max_norm / *min_norm, 1.5);
}

/** @brief Invalid half angles, axes, coefficients and links are rejected at construction */
TEST_F(CartesianAxisConstraintUnit, ConeRejectsInvalidArguments)  // NOLINT
{
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  auto make = [&](const std::string& source,
                  const Eigen::Vector3d& source_axis,
                  const std::string& target,
                  const Eigen::Vector3d& target_axis,
                  double half_angle,
                  double coeff) {
    return CartAxisConeConstraint(var, kin_group, source, source_axis, target, target_axis, half_angle, coeff);
  };

  for (const double bad : { 0.0, -0.1, M_PI, 4.0, INF, NAN_D })
    EXPECT_THROW(make(TOOL, z, BASE, z, bad, 1.0), std::runtime_error) << "half angle " << bad;

  for (const Eigen::Vector3d& bad :
       { Eigen::Vector3d::Zero().eval(), Eigen::Vector3d(NAN_D, 0, 1), Eigen::Vector3d(INF, 0, 0) })
  {
    EXPECT_THROW(make(TOOL, bad, BASE, z, 0.1, 1.0), std::runtime_error);
    EXPECT_THROW(make(TOOL, z, BASE, bad, 0.1, 1.0), std::runtime_error);
  }

  for (const double bad : { -1.0, INF, NAN_D })
    EXPECT_THROW(make(TOOL, z, BASE, z, 0.1, bad), std::runtime_error) << "coeff " << bad;

  EXPECT_THROW(make("no_such_link", z, BASE, z, 0.1, 1.0), std::runtime_error);
  EXPECT_THROW(make(TOOL, z, "no_such_link", z, 0.1, 1.0), std::runtime_error);
  EXPECT_THROW(make(BASE, z, TORSO, z, 0.1, 1.0), std::runtime_error);

  CartAxisConeConstraint cnt = make(TOOL, z, BASE, z, 0.1, 1.0);
  EXPECT_THROW(cnt.setTargetAxis(Eigen::Vector3d::Zero()), std::runtime_error);
  cnt.setTargetAxis(Eigen::Vector3d(0, 3, 0));
  EXPECT_TRUE(cnt.getTargetAxis().isApprox(Eigen::Vector3d::UnitY()));
}

/**
 * @brief No box on CartPosConstraint's rotation error equals the cone.
 *
 * The inscribed square |r_x|, |r_y| <= theta / sqrt(2) rejects a tilt inside the cone; the circumscribing square
 * |r_x|, |r_y| <= theta accepts a diagonal tilt outside it. The cone constraint classifies both correctly.
 */
TEST_F(CartesianAxisConstraintUnit, ConeIsRoundWhereCartPosBoxIsSquare)  // NOLINT
{
  constexpr double theta = 0.4;
  const Eigen::VectorXd q = testConfigurations().front();
  const Eigen::Isometry3d tool_pose = kin_group->calcFwdKin(q).at(TOOL);
  const Eigen::Isometry3d base_pose = kin_group->calcFwdKin(q).at(BASE);

  struct Case
  {
    Eigen::Vector3d tilt_axis;  // in tool coordinates, perpendicular to the tool z axis
    double tilt;                // [rad]
    bool in_cone;
    double box_half_width;  // [rad]
  };
  const std::vector<Case> cases{
    { Eigen::Vector3d::UnitX(), 0.35, true, theta / std::sqrt(2.0) },  // inscribed box rejects
    { Eigen::Vector3d(1, 1, 0).normalized(), 0.5, false, theta },      // circumscribed box accepts
  };

  for (const auto& c : cases)
  {
    // Target pose: the current tool pose tilted about tilt_axis, expressed relative to the base
    const Eigen::Isometry3d target_world = tool_pose * Eigen::AngleAxisd(c.tilt, c.tilt_axis);
    const Eigen::Isometry3d target_in_base = base_pose.inverse() * target_world;

    const CartPosConstraint cart_pos(var, kin_group, TOOL, BASE, Eigen::Isometry3d::Identity(), target_in_base);
    const Eigen::Vector3d rotation_error = cart_pos.calcValues(q).tail<3>();
    const bool in_box =
        std::abs(rotation_error.x()) <= c.box_half_width && std::abs(rotation_error.y()) <= c.box_half_width;

    const Eigen::Vector3d cone_axis = target_in_base.linear() * Eigen::Vector3d::UnitZ();
    const CartAxisConeConstraint cone(var, kin_group, TOOL, Eigen::Vector3d::UnitZ(), BASE, cone_axis, theta);
    const bool in_cone = cone.calcValues(q)(0) <= theta;

    EXPECT_NEAR(cone.calcValues(q)(0), c.tilt, VALUE_TOL);
    EXPECT_EQ(in_cone, c.in_cone);
    EXPECT_NE(in_box, c.in_cone) << "box half width " << c.box_half_width << " rotation error "
                                 << rotation_error.transpose();
  }
}

////////////////////////////////////////////////////////////////////
// Align
////////////////////////////////////////////////////////////////////

/** @brief Rows are the logarithm map: norm is the tilt angle, direction is the tilt direction P v / |P v| */
TEST_F(CartesianAxisConstraintUnit, AlignValuesAreLogarithmMap)  // NOLINT
{
  const Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
  for (const auto& links : linkPairs())
  {
    for (const auto& q : testConfigurations())
    {
      const Eigen::Vector3d v = relativeAxis(links, source_axis, q);
      for (const double angle : { 0.0, 1e-9, 0.3, M_PI / 2, 3.0, M_PI })
      {
        const Eigen::Vector3d target_axis = Eigen::AngleAxisd(angle, perpendicular(v, 0.4)) * v;
        const CartAxisAlignConstraint cnt(var, kin_group, links.source, source_axis, links.target, target_axis);
        const Eigen::VectorXd value = cnt.calcValues(q);
        ASSERT_EQ(value.size(), 2);
        EXPECT_NEAR(value.norm(), angle, VALUE_TOL) << links.source << " -> " << links.target << " angle " << angle;

        if (angle > 1e-6 && angle < M_PI - 1e-6)
        {
          // Direction of the tilt in the target's perpendicular basis
          const Eigen::Vector3d p1 = cnt.getTargetAxis().unitOrthogonal();
          const Eigen::Vector3d p2 = cnt.getTargetAxis().cross(p1);
          const Eigen::Vector2d tilt(p1.dot(v), p2.dot(v));
          EXPECT_LT((value.normalized() - tilt.normalized()).norm(), VALUE_TOL * 1e3);
        }
      }
    }
  }
}

/** @brief Two equality rows, both weighted by coeff */
TEST_F(CartesianAxisConstraintUnit, AlignBoundsCoefficientsAndValues)  // NOLINT
{
  const CartAxisAlignConstraint cnt(
      var, kin_group, TOOL, Eigen::Vector3d(0, 0, 2), BASE, Eigen::Vector3d(0, -4, 0), 2.5);

  EXPECT_EQ(cnt.getRows(), 2);
  const std::vector<Bounds> bounds = cnt.getBounds();
  ASSERT_EQ(bounds.size(), 2);
  for (const auto& bound : bounds)
  {
    EXPECT_EQ(bound.getLower(), 0.0);
    EXPECT_EQ(bound.getUpper(), 0.0);
  }
  EXPECT_TRUE(cnt.getCoefficients().isApprox(Eigen::Vector2d::Constant(2.5)));
  EXPECT_TRUE(cnt.getTargetAxis().isApprox(-Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(cnt.getValues().isApprox(cnt.calcValues(var->value())));
}

/** @brief The analytic jacobian matches central differences, including at alignment (pi is the cut point) */
TEST_F(CartesianAxisConstraintUnit, AlignJacobianMatchesCentralDifferences)  // NOLINT
{
  const Eigen::Vector3d source_axis(-0.4, 0.1, 1.0);
  for (const auto& links : linkPairs())
  {
    for (const auto& q : testConfigurations())
    {
      const Eigen::Vector3d v = relativeAxis(links, source_axis, q);
      for (const double angle : { 0.0, 0.4, 1.5, 2.8 })
      {
        const Eigen::Vector3d target_axis = Eigen::AngleAxisd(angle, perpendicular(v, 2.3)) * v;
        const CartAxisAlignConstraint cnt(var, kin_group, links.source, source_axis, links.target, target_axis);

        const Eigen::MatrixXd numeric =
            centralDifference([&](const Eigen::VectorXd& x) { return cnt.calcValues(x); }, q);
        const Eigen::MatrixXd analytic =
            analyticJacobian([&](Jacobian& jac, const Eigen::VectorXd& x) { cnt.calcJacobianBlock(jac, x); }, 2, q);

        EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), JAC_TOL)
            << links.source << " -> " << links.target << " angle " << angle << "\nanalytic\n"
            << analytic << "\nnumeric\n"
            << numeric;
      }
    }
  }
}

/** @brief getJacobian evaluates calcJacobianBlock at the variable values */
TEST_F(CartesianAxisConstraintUnit, AlignGetJacobianUsesVariableValues)  // NOLINT
{
  auto cnt = std::make_shared<CartAxisAlignConstraint>(
      var, kin_group, TOOL, Eigen::Vector3d::UnitZ(), BASE, Eigen::Vector3d::UnitX());
  nlp->addConstraintSet(cnt);
  const Eigen::MatrixXd analytic = analyticJacobian(
      [&](Jacobian& jac, const Eigen::VectorXd& x) { cnt->calcJacobianBlock(jac, x); }, 2, var->value());
  EXPECT_TRUE(cnt->getJacobian().toDense().isApprox(analytic));
}

/** @brief At the solution the two rows have independent gradients */
TEST_F(CartesianAxisConstraintUnit, AlignEqualityJacobianHasFullRankAtSolution)  // NOLINT
{
  const Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
  for (const auto& links : linkPairs())
  {
    for (const auto& q : testConfigurations())
    {
      const CartAxisAlignConstraint cnt(
          var, kin_group, links.source, source_axis, links.target, relativeAxis(links, source_axis, q));
      const Eigen::MatrixXd analytic =
          analyticJacobian([&](Jacobian& jac, const Eigen::VectorXd& x) { cnt.calcJacobianBlock(jac, x); }, 2, q);
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(analytic);
      // A 7-dof arm can tilt its tool in both directions; the smaller singular value is O(link length) in rad/rad.
      EXPECT_GT(svd.singularValues()(1), 1e-2) << links.source << " -> " << links.target;
    }
  }
}

/**
 * @brief At v = -a_tgt the rows have norm pi and the jacobian descends along the most reachable direction.
 *
 * The rows P v = 0 alone would be satisfied here; the logarithm map is not, so the solver is pushed away.
 */
TEST_F(CartesianAxisConstraintUnit, AlignAtAntiparallelIsInfeasibleWithDescentDirection)  // NOLINT
{
  // Step along the descent direction [rad]; small enough to stay in the linear regime
  constexpr double step = 1e-4;
  const Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
  for (const auto& links : linkPairs())
  {
    const Eigen::VectorXd q = testConfigurations().front();
    const CartAxisAlignConstraint cnt(
        var, kin_group, links.source, source_axis, links.target, -relativeAxis(links, source_axis, q));
    const Eigen::VectorXd value = cnt.calcValues(q);
    ASSERT_NEAR(value.norm(), M_PI, VALUE_TOL);

    const Eigen::MatrixXd analytic =
        analyticJacobian([&](Jacobian& jac, const Eigen::VectorXd& x) { cnt.calcJacobianBlock(jac, x); }, 2, q);
    EXPECT_NEAR(analytic.norm(), largestTangentRate(links, source_axis, q), JAC_TOL);

    // Gauss-Newton direction on |r|^2 decreases |r|
    const Eigen::VectorXd descent = -(analytic.transpose() * value).normalized();
    EXPECT_LT(cnt.calcValues(q + step * descent).norm(), M_PI - step * 1e-3) << links.source << " -> " << links.target;
  }
}

/** @brief Invalid axes, coefficients and links are rejected at construction */
TEST_F(CartesianAxisConstraintUnit, AlignRejectsInvalidArguments)  // NOLINT
{
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  auto make = [&](const std::string& source,
                  const Eigen::Vector3d& source_axis,
                  const std::string& target,
                  const Eigen::Vector3d& target_axis,
                  double coeff) {
    return CartAxisAlignConstraint(var, kin_group, source, source_axis, target, target_axis, coeff);
  };

  for (const Eigen::Vector3d& bad :
       { Eigen::Vector3d::Zero().eval(), Eigen::Vector3d(NAN_D, 0, 1), Eigen::Vector3d(INF, 0, 0) })
  {
    EXPECT_THROW(make(TOOL, bad, BASE, z, 1.0), std::runtime_error);
    EXPECT_THROW(make(TOOL, z, BASE, bad, 1.0), std::runtime_error);
  }

  for (const double bad : { -1.0, INF, NAN_D })
    EXPECT_THROW(make(TOOL, z, BASE, z, bad), std::runtime_error) << "coeff " << bad;

  EXPECT_THROW(make("no_such_link", z, BASE, z, 1.0), std::runtime_error);
  EXPECT_THROW(make(TOOL, z, "no_such_link", z, 1.0), std::runtime_error);
  EXPECT_THROW(make(BASE, z, TORSO, z, 1.0), std::runtime_error);

  CartAxisAlignConstraint cnt = make(TOOL, z, BASE, z, 1.0);
  EXPECT_THROW(cnt.setTargetAxis(Eigen::Vector3d(NAN_D, 0, 0)), std::runtime_error);
  cnt.setTargetAxis(Eigen::Vector3d(-2, 0, 0));
  EXPECT_TRUE(cnt.getTargetAxis().isApprox(-Eigen::Vector3d::UnitX()));
}

/**
 * @brief The align rows are continuous where CartPosConstraint's rotation rows jump.
 *
 * Fix a tilt epsilon of the tool z axis and sweep the free rotation gamma about it through pi. The rotation vector
 * wraps at gamma = pi, so the r_x, r_y rows of CartPosConstraint (with r_z dropped, the usual way to free that axis)
 * change sign; the align rows move by O(delta).
 */
TEST_F(CartesianAxisConstraintUnit, AlignIsContinuousWhereCartPosWrapsAtHalfTurn)  // NOLINT
{
  constexpr double epsilon = 0.1;  // tilt [rad]
  constexpr double delta = 1e-6;   // distance of the two samples from gamma = pi [rad]
  const Eigen::VectorXd q = testConfigurations().front();
  const Eigen::Isometry3d tool_pose = kin_group->calcFwdKin(q).at(TOOL);
  const Eigen::Isometry3d base_pose = kin_group->calcFwdKin(q).at(BASE);

  Eigen::VectorXd cart_pos_coeffs(6);
  cart_pos_coeffs << 1, 1, 1, 1, 1, 0;

  std::vector<Eigen::Vector2d> cart_pos_rows;
  std::vector<Eigen::Vector2d> align_rows;
  for (const double gamma : { M_PI - delta, M_PI + delta })
  {
    const Eigen::Isometry3d target_world = tool_pose * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ()) *
                                           Eigen::AngleAxisd(epsilon, Eigen::Vector3d::UnitX());
    const Eigen::Isometry3d target_in_base = base_pose.inverse() * target_world;

    const CartPosConstraint cart_pos(var,
                                     cart_pos_coeffs,
                                     std::vector<Bounds>(6, BoundZero),
                                     kin_group,
                                     TOOL,
                                     BASE,
                                     Eigen::Isometry3d::Identity(),
                                     target_in_base);
    const Eigen::VectorXd cart_pos_values = cart_pos.calcValues(q);
    ASSERT_EQ(cart_pos_values.size(), 5);
    cart_pos_rows.emplace_back(cart_pos_values.tail<2>());

    const CartAxisAlignConstraint align(
        var, kin_group, TOOL, Eigen::Vector3d::UnitZ(), BASE, target_in_base.linear() * Eigen::Vector3d::UnitZ());
    align_rows.emplace_back(align.calcValues(q));
  }

  EXPECT_GT((cart_pos_rows[1] - cart_pos_rows[0]).norm(), epsilon);
  EXPECT_LT((align_rows[1] - align_rows[0]).norm(), 10 * delta);
  EXPECT_NEAR(align_rows[0].norm(), epsilon, VALUE_TOL);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
