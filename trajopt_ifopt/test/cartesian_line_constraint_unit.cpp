#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <ctime>
#include <limits>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <tesseract/common/logging.h>
#include <tesseract/common/types.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/utils.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
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
using namespace tesseract::environment;
using namespace tesseract::kinematics;
using namespace tesseract::collision;
using namespace tesseract::scene_graph;
using namespace tesseract::geometry;

namespace
{
/** @brief The angle the slanted line turns between its start and its end, about SLANTED_LINE_AXIS */
const double SLANTED_LINE_ANGLE = 0.6;
const Eigen::Vector3d SLANTED_LINE_AXIS = Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0;

/** @brief The fraction of the way from a to b of the point on that segment nearest to c */
double nearestFraction(const Eigen::Vector3d& c, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  return std::clamp((c - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
}

/** @brief The distance from c to the segment from a to b, without finding the nearest point */
double distanceToSegment(const Eigen::Vector3d& c, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  if ((c - a).dot(ab) <= 0.0)
    return (c - a).norm();
  if ((c - b).dot(ab) >= 0.0)
    return (c - b).norm();
  return ab.cross(c - a).norm() / ab.norm();
}
}  // namespace

class CartesianLineConstraintUnit : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>();
  Variables::Ptr variables;

  JointGroup::ConstPtr manip;
  CartLineInfo info;
  std::shared_ptr<const Var> var;

  /** @brief The joint position at which source_tf and the frames of the lines' target links are taken */
  Eigen::VectorXd reference_position;
  Eigen::Isometry3d source_tf;
  Eigen::Isometry3d line_start_pose;
  Eigen::Isometry3d line_end_pose;

  Eigen::Index n_dof{ 0 };

  /** @brief Every row, every row in another order, the position rows only, and two rows in another order */
  const std::vector<Eigen::VectorXi> index_lists{ (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished(),
                                                  (Eigen::VectorXi(6) << 3, 4, 5, 0, 1, 2).finished(),
                                                  (Eigen::VectorXi(3) << 0, 1, 2).finished(),
                                                  (Eigen::VectorXi(2) << 4, 0).finished() };

  void SetUp() override
  {
    // Initialize Tesseract
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    const bool status = env->init(urdf_file, srdf_file, locator);
    EXPECT_TRUE(status);

    // Extract necessary kinematic information
    manip = env->getJointGroup("right_arm");
    n_dof = manip->numJoints();

    std::vector<Bounds> bounds(static_cast<std::size_t>(manip->numJoints()), NoBound);
    auto node = std::make_unique<Node>("Joint_Position_0");
    auto pos = Eigen::VectorXd::Ones(n_dof);
    const std::vector<std::string> joint_names = tesseract::common::toNames(manip->getJointIds());
    var = node->addVar("position", joint_names, pos, bounds);
    std::vector<std::unique_ptr<Node>> nodes;
    nodes.push_back(std::move(node));
    variables = std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes));

    // Add constraints
    reference_position = Eigen::VectorXd::Ones(n_dof);
    source_tf = manip->calcFwdKin(reference_position).at("r_gripper_tool_frame");

    // The line runs half a metre either side of the tool, given in the frame of base_link
    line_start_pose = source_tf;
    line_start_pose.translation() = line_start_pose.translation() + Eigen::Vector3d(-0.5, 0.0, 0.0);
    line_start_pose = inLinkFrame("base_link", line_start_pose);
    line_end_pose = source_tf;
    line_end_pose.translation() = line_end_pose.translation() + Eigen::Vector3d(0.5, 0.0, 0.0);
    line_end_pose = inLinkFrame("base_link", line_end_pose);
  }

  /** @brief Express a world pose in the frame of a link, taken at reference_position */
  Eigen::Isometry3d inLinkFrame(const tesseract::common::LinkId& link, const Eigen::Isometry3d& world_pose) const
  {
    return manip->calcFwdKin(reference_position).at(link).inverse() * world_pose;
  }

  /**
   * @brief Describe a slanted line near the tool, fixed to the target link, with a tool offset on the source
   *
   * The line turns by SLANTED_LINE_ANGLE about SLANTED_LINE_AXIS from its start to its end.
   */
  CartLineInfo slantedLineInfo(const tesseract::common::LinkId& target_frame,
                               double length,
                               const Eigen::VectorXi& indices) const
  {
    Eigen::Isometry3d start = source_tf * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ());
    start.translation() = source_tf.translation() + Eigen::Vector3d(-0.02, 0.18, -0.07);
    Eigen::Isometry3d end = start * Eigen::AngleAxisd(SLANTED_LINE_ANGLE, SLANTED_LINE_AXIS);
    end.translation() = start.translation() + length * Eigen::Vector3d(2.0, -1.0, 2.0) / 3.0;

    const Eigen::Isometry3d source_frame_offset =
        Eigen::Translation3d(0.05, 0.1, -0.2) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY());

    return { manip,
             "r_gripper_tool_frame",
             target_frame,
             inLinkFrame(target_frame, start),
             inLinkFrame(target_frame, end),
             source_frame_offset,
             indices };
  }

  /** @brief Joint positions that put the tool beside the slanted line, before its start and past its end */
  std::vector<Eigen::VectorXd> jointPositions() const
  {
    std::vector<Eigen::VectorXd> joint_positions{ reference_position };
    for (Eigen::Index i = 0; i < n_dof; ++i)
    {
      joint_positions.emplace_back(reference_position);
      joint_positions.back()[i] = 2.0;
    }
    return joint_positions;
  }
};

/** @brief Checks that the GetValue function is correct */
TEST_F(CartesianLineConstraintUnit, GetValue)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CartesianPositionConstraintUnit, GetValue");

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

/** @brief Check the nearest point on a slanted line for a source beside it, before its start and past its end */
TEST_F(CartesianLineConstraintUnit, GetLinePoint)  // NOLINT
{
  // A line of length 3 whose direction has a negative component
  Eigen::Isometry3d start = Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d end = start * Eigen::AngleAxisd(SLANTED_LINE_ANGLE, SLANTED_LINE_AXIS);
  end.translation() = Eigen::Vector3d(-1.0, 3.0, 5.0);
  const Eigen::Vector3d line = end.translation() - start.translation();

  // The source sits beside the line at a fraction of its length; the nearest point stays within the line
  const std::vector<std::pair<double, double>> cases{ { 0.25, 0.25 }, { 0.75, 0.75 }, { -0.3, 0.0 }, { 1.2, 1.0 } };
  for (const auto& [source_fraction, fraction] : cases)
  {
    Eigen::Isometry3d source = Eigen::Translation3d(start.translation() + source_fraction * line) *
                               Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitX());
    source.translation() += 0.3 * Eigen::Vector3d(1.0, 2.0, 0.0);

    Eigen::Isometry3d expected = start * Eigen::AngleAxisd(fraction * SLANTED_LINE_ANGLE, SLANTED_LINE_AXIS);
    expected.translation() = start.translation() + fraction * line;

    const Eigen::Isometry3d line_point = CartLineConstraint::getLinePoint(source, start, end);
    EXPECT_LT((line_point.matrix() - expected.matrix()).norm(), 1e-12) << "source fraction " << source_fraction;

    // Swapping the ends of the line leaves the nearest point where it is
    const Eigen::Isometry3d swapped = CartLineConstraint::getLinePoint(source, end, start);
    EXPECT_LT((swapped.matrix() - expected.matrix()).norm(), 1e-12) << "source fraction " << source_fraction;
  }

  // A line of zero length has only its start
  end.translation() = start.translation();
  const Eigen::Isometry3d line_point = CartLineConstraint::getLinePoint(Eigen::Isometry3d::Identity(), start, end);
  EXPECT_LT((line_point.matrix() - start.matrix()).norm(), 1e-12);
}

/** @brief Check the values against the nearest point on a slanted line, fixed to a rotated and to a moving link */
TEST_F(CartesianLineConstraintUnit, GetValueSlantedLine)  // NOLINT
{
  for (const auto& target_frame : { "imu_link", "r_upper_arm_roll_link" })
  {
    int beside{ 0 };
    int before{ 0 };
    int past{ 0 };
    for (const double length : { 0.3, 2.4 })
    {
      for (const Eigen::VectorXi& indices : index_lists)
      {
        info = slantedLineInfo(target_frame, length, indices);
        const CartLineConstraint constraint(info, var, Eigen::VectorXd::Ones(info.indices.rows()));

        for (const Eigen::VectorXd& joint_position : jointPositions())
        {
          const auto transforms = manip->calcFwdKin(joint_position);
          const Eigen::Isometry3d source = transforms.at(info.source_frame) * info.source_frame_offset;
          const Eigen::Isometry3d start = transforms.at(info.target_frame) * info.target_frame_offset1;
          const Eigen::Isometry3d end = transforms.at(info.target_frame) * info.target_frame_offset2;

          const double fraction = nearestFraction(source.translation(), start.translation(), end.translation());
          beside += static_cast<int>(fraction > 0.0 && fraction < 1.0);
          before += static_cast<int>(fraction == 0.0);
          past += static_cast<int>(fraction == 1.0);

          Eigen::Isometry3d line_point = start * Eigen::AngleAxisd(fraction * SLANTED_LINE_ANGLE, SLANTED_LINE_AXIS);
          line_point.translation() = start.translation() + fraction * (end.translation() - start.translation());

          // The translation terms span the distance from the source to the line
          const Eigen::VectorXd expected = tesseract::common::calcTransformError(line_point, source);
          const double distance = distanceToSegment(source.translation(), start.translation(), end.translation());
          EXPECT_NEAR(expected.head(3).norm(), distance, 1e-12) << target_frame << ", length " << length;

          // The values are the terms the indices name, in the order of the list
          const Eigen::VectorXd error = constraint.calcValues(joint_position);
          ASSERT_EQ(error.size(), indices.size());
          for (Eigen::Index i = 0; i < indices.size(); ++i)
            EXPECT_NEAR(error[i], expected[indices[i]], 1e-12)
                << target_frame << ", length " << length << ", indices " << indices.transpose();
        }
      }
    }

    // The joint positions cover the three places the tool can be along the line
    EXPECT_GT(beside, 0) << target_frame;
    EXPECT_GT(before, 0) << target_frame;
    EXPECT_GT(past, 0) << target_frame;
  }
}

///** @brief Checks that the FillJacobian function is correct */
TEST_F(CartesianLineConstraintUnit, FillJacobian)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CartesianPositionConstraintUnit, FillJacobian");

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
      constraint->calcJacobianBlock(jac_block, joint_position_mod);  // NOLINT
      EXPECT_TRUE(jac_block.isApprox(num_jac_block, 1e-3));
    }
    {
      Jacobian jac_block = constraint->getJacobian();
      EXPECT_TRUE(jac_block.toDense().isApprox(num_jac_block.toDense(), 1e-3));
    }
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(CartesianLineConstraintUnit, GetSetBounds)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CartesianPositionConstraintUnit, GetSetBounds");

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

////////////////////////////////////////////////////////////////////

/** @brief Coefficients must be finite and non-negative */
TEST_F(CartesianLineConstraintUnit, RejectsInvalidCoeffs)  // NOLINT
{
  info = CartLineInfo(manip, "r_gripper_tool_frame", "base_link", line_start_pose, line_end_pose);
  for (const double bad : { -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
  {
    Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(info.indices.rows());
    coeffs(0) = bad;
    EXPECT_THROW(std::make_shared<CartLineConstraint>(info, var, coeffs), std::runtime_error);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
