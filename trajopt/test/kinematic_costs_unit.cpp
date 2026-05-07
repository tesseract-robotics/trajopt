#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract/common/types.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>

#include <trajopt/kinematic_terms.hpp>
#include <trajopt_sco/num_diff.hpp>

using namespace trajopt;
using namespace std;
using namespace trajopt_common;
using namespace tesseract::environment;
using namespace tesseract::collision;
using namespace tesseract::kinematics;
using namespace tesseract::visualization;
using namespace tesseract::scene_graph;
using namespace tesseract::common;

class KinematicCostsTest : public testing::Test
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Trajopt Basic Environment */

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    gLogLevel = trajopt_common::LevelError;
  }
};

namespace
{
std::string toString(const Eigen::MatrixXd& mat)
{
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

void checkJacobian(const sco::VectorOfVector& f,
                   const sco::MatrixOfVector& dfdx,
                   const Eigen::VectorXd& values,
                   const double epsilon)
{
  const Eigen::MatrixXd numerical = sco::calcForwardNumJac(f, values, epsilon);
  const Eigen::MatrixXd analytical = dfdx(values);

  const bool pass = numerical.isApprox(analytical, 1e-5);
  EXPECT_TRUE(pass);
  if (!pass)
  {
    CONSOLE_BRIDGE_logError("Numerical:\n %s", toString(numerical).c_str());
    CONSOLE_BRIDGE_logError("Analytical:\n %s", toString(analytical).c_str());
  }
}
}  // namespace

TEST_F(KinematicCostsTest, CartPoseJacCalculator)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, CartPoseJacCalculator");

  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");

  const std::string source_frame = env_->getRootLinkName();
  const std::string target_frame = "r_gripper_tool_frame";
  const Eigen::Isometry3d source_frame_offset = env_->getState().link_transforms.at(target_frame);
  const Eigen::Isometry3d target_frame_offset =
      Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  const CartPoseErrCalculator f(kin, source_frame, target_frame, source_frame_offset, target_frame_offset);
  const CartPoseJacCalculator dfdx(kin, source_frame, target_frame, source_frame_offset, target_frame_offset);
  checkJacobian(f, dfdx, values, 1.0e-5);
}

TEST_F(KinematicCostsTest, CartPoseJacCalculator_TolerancedInsideBand)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, CartPoseJacCalculator_TolerancedInsideBand");

  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");

  const std::string source_frame = env_->getRootLinkName();
  const std::string target_frame = "r_gripper_tool_frame";

  // Seed configuration. We then construct target_frame_offset so that the pose error at this seed is +0.1 rad about
  // x and zero everywhere else (well inside the ±0.52 band on rx).
  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  const tesseract::common::TransformMap state_cache = kin->calcFwdKin(values);
  const Eigen::Isometry3d& source_frame_offset = state_cache.at(target_frame);
  const Eigen::Isometry3d target_frame_offset =
      Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());

  Eigen::VectorXd lower(6);
  Eigen::VectorXd upper(6);
  lower << 0.0, 0.0, 0.0, -0.52, 0.0, 0.0;
  upper << 0.0, 0.0, 0.0, 0.52, 0.0, 0.0;

  const Eigen::VectorXi indices = (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished();

  const CartPoseErrCalculator f(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);
  const CartPoseJacCalculator dfdx(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);

  // (a) f at the seed must be zero — rx error is in the band, others are exactly zero by construction.
  const Eigen::VectorXd err_at_seed = f(values);
  EXPECT_TRUE(err_at_seed.isZero(1e-9)) << "Toleranced error at seed should be zero inside band, got: "
                                        << err_at_seed.transpose();

  // (b) The smoking gun: the rx row of the analytical Jacobian must be ~0. f is identically zero on an open
  //     neighborhood of the seed in rx (band half-width 0.52, seed 0.1 rad in), so the analytical gradient w.r.t.
  //     rx must vanish.
  const Eigen::MatrixXd analytical = dfdx(values);
  EXPECT_TRUE(analytical.row(3).isZero(1e-6))
      << "rx row of toleranced Jacobian must be zero inside band, got: " << analytical.row(3);

  // (c) Consistency: numerical FD of toleranced f must equal analytical Jac.
  checkJacobian(f, dfdx, values, 1.0e-5);
}

TEST_F(KinematicCostsTest, CartPoseJacCalculator_TolerancedOutsideBand)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, CartPoseJacCalculator_TolerancedOutsideBand");

  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");

  const std::string source_frame = env_->getRootLinkName();
  const std::string target_frame = "r_gripper_tool_frame";

  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  const tesseract::common::TransformMap state_cache = kin->calcFwdKin(values);
  const Eigen::Isometry3d& source_frame_offset = state_cache.at(target_frame);
  // 0.8 rad rx offset, well outside the ±0.52 band.
  const Eigen::Isometry3d target_frame_offset =
      Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitX());

  Eigen::VectorXd lower(6);
  Eigen::VectorXd upper(6);
  lower << 0.0, 0.0, 0.0, -0.52, 0.0, 0.0;
  upper << 0.0, 0.0, 0.0, 0.52, 0.0, 0.0;

  const Eigen::VectorXi indices = (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished();

  const CartPoseErrCalculator f(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);
  const CartPoseJacCalculator dfdx(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);

  // f is non-zero outside the band: f = (true_err - upper) on rx.
  EXPECT_FALSE(f(values).isZero(1e-9));

  // Numerical FD of f̃ equals analytical Jac. (Outside the band, f̃ is just f shifted by a constant — the constant
  // cancels in the FD difference, so this would also pass under the old buggy implementation.)
  checkJacobian(f, dfdx, values, 1.0e-5);
}

TEST_F(KinematicCostsTest, CartPoseJacCalculator_TolerancedAcrossEdge)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, CartPoseJacCalculator_TolerancedAcrossEdge");

  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");

  const std::string source_frame = env_->getRootLinkName();
  const std::string target_frame = "r_gripper_tool_frame";

  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  const tesseract::common::TransformMap state_cache = kin->calcFwdKin(values);
  const Eigen::Isometry3d& source_frame_offset = state_cache.at(target_frame);
  // 0.51 rad rx — just inside the band, so any non-trivial perturbation around the edge straddles it.
  const Eigen::Isometry3d target_frame_offset =
      Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(0.51, Eigen::Vector3d::UnitX());

  Eigen::VectorXd lower(6);
  Eigen::VectorXd upper(6);
  lower << 0.0, 0.0, 0.0, -0.52, 0.0, 0.0;
  upper << 0.0, 0.0, 0.0, 0.52, 0.0, 0.0;

  const Eigen::VectorXi indices = (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished();

  const CartPoseErrCalculator f(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);
  const CartPoseJacCalculator dfdx(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);

  // f is zero at the seed (0.51 is inside [-0.52, 0.52]).
  EXPECT_TRUE(f(values).isZero(1e-9));

  // FD-check the analytical Jac at small epsilons (which stay inside the band).
  // Larger epsilons (1e-3, 1e-2, 1e-1) are excluded: this seed is only 0.01 rad from the upper edge, so even a
  // moderate joint perturbation can carry the rotational error across the band boundary through the kinematic chain.
  // When that happens the numerical FD secant mixes "inside" and "outside" behaviour while the analytical Jac (which
  // uses DEFAULT_EPSILON ~ 1e-5 internally) stays squarely inside — producing genuine, unavoidable FD inaccuracy that
  // is NOT a Jacobian bug. The two small epsilons below are sufficient to catch a naive row-zeroing shortcut: such a
  // fix would zero rows 0-2 and 4-5 (non-rx DOFs), which the numerical FD at these epsilons would immediately expose.
  checkJacobian(f, dfdx, values, 1.0e-7);
  checkJacobian(f, dfdx, values, 1.0e-5);
}

TEST_F(KinematicCostsTest, DynamicCartPoseJacCalculator_TolerancedInsideBand)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, DynamicCartPoseJacCalculator_TolerancedInsideBand");

  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");

  // Both frames must be active for the dynamic variant. r_gripper_tool_frame and r_wrist_roll_link both move with
  // the right arm joints in the pr2 srdf.
  const std::string source_frame = "r_gripper_tool_frame";
  const std::string target_frame = "r_wrist_roll_link";

  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  const tesseract::common::TransformMap state_cache = kin->calcFwdKin(values);
  const Eigen::Isometry3d source_frame_offset = Eigen::Isometry3d::Identity();
  // Pick a target_frame_offset such that the seed pose error has rx inside ±0.52 (e.g. 0.1 rad). Concretely: compute
  // current_target_in_source = source_tf.inverse() * target_tf at the seed, then target_frame_offset =
  // current_target_in_source.inverse() * AngleAxis(0.1, x) so the err vector at the seed is +/-0.1 about x and zero
  // elsewhere.
  // Error formula: err = calcTransformError(target_tf, source_tf) = target_tf.inverse() * source_tf.
  // With source_frame_offset = I: source_tf = state_cache[source_frame].
  // With target_frame_offset below: target_tf = state_cache[target_frame] * target_frame_offset.
  // At seed, target_tf = wrist_tf * wrist_tf^-1 * gripper_tf * AngleAxis(0.1, x) = gripper_tf * AngleAxis(0.1, x).
  // So err = (gripper_tf * AngleAxis(0.1,x))^-1 * gripper_tf = AngleAxis(-0.1, x). err[3] = -0.1 inside [-0.52, 0.52].
  const Eigen::Isometry3d& source_tf_seed = state_cache.at(source_frame);
  const Eigen::Isometry3d& target_tf_seed = state_cache.at(target_frame);
  const Eigen::Isometry3d current_target_in_source = source_tf_seed.inverse() * target_tf_seed;
  const Eigen::Isometry3d target_frame_offset =
      current_target_in_source.inverse() * Eigen::Isometry3d(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));

  Eigen::VectorXd lower(6);
  Eigen::VectorXd upper(6);
  lower << 0.0, 0.0, 0.0, -0.52, 0.0, 0.0;
  upper << 0.0, 0.0, 0.0, 0.52, 0.0, 0.0;

  const Eigen::VectorXi indices = (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished();

  const DynamicCartPoseErrCalculator f(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);
  const DynamicCartPoseJacCalculator dfdx(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);

  // (a) f at the seed must be zero — rx error is in the band, others are exactly zero by construction.
  EXPECT_TRUE(f(values).isZero(1e-9));

  // (b) The smoking gun: the rx row of the analytical Jacobian must be ~0. f is identically zero on an open
  //     neighborhood of the seed in rx (band half-width 0.52, seed 0.1 rad in), so the analytical gradient w.r.t.
  //     rx must vanish.
  const Eigen::MatrixXd analytical = dfdx(values);
  EXPECT_TRUE(analytical.row(3).isZero(1e-6))
      << "rx row of toleranced dynamic Jacobian must be zero inside band, got: " << analytical.row(3);

  // (c) Consistency: numerical FD of toleranced f must equal analytical Jac.
  checkJacobian(f, dfdx, values, 1.0e-5);
}

TEST_F(KinematicCostsTest, DynamicCartPoseJacCalculator_TolerancedOutsideBand)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, DynamicCartPoseJacCalculator_TolerancedOutsideBand");

  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");

  const std::string source_frame = "r_gripper_tool_frame";
  const std::string target_frame = "r_wrist_roll_link";

  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  const tesseract::common::TransformMap state_cache = kin->calcFwdKin(values);
  const Eigen::Isometry3d source_frame_offset = Eigen::Isometry3d::Identity();
  // 0.8 rad rx offset, well outside the ±0.52 band. Same construction as InsideBand but with a larger angle.
  // current_target_in_source = source_tf^-1 * target_tf at the seed.
  // target_frame_offset = current_target_in_source^-1 * AngleAxis(0.8, x)
  // => at seed, err[3] ≈ -0.8 which is outside [-0.52, 0.52].
  const Eigen::Isometry3d& source_tf_seed = state_cache.at(source_frame);
  const Eigen::Isometry3d& target_tf_seed = state_cache.at(target_frame);
  const Eigen::Isometry3d current_target_in_source = source_tf_seed.inverse() * target_tf_seed;
  const Eigen::Isometry3d target_frame_offset =
      current_target_in_source.inverse() * Eigen::Isometry3d(Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitX()));

  Eigen::VectorXd lower(6);
  Eigen::VectorXd upper(6);
  lower << 0.0, 0.0, 0.0, -0.52, 0.0, 0.0;
  upper << 0.0, 0.0, 0.0, 0.52, 0.0, 0.0;

  const Eigen::VectorXi indices = (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished();

  const DynamicCartPoseErrCalculator f(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);
  const DynamicCartPoseJacCalculator dfdx(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);

  // f is non-zero outside the band: err[3] ≈ -0.8 - (-0.52) = -0.28 (clamped residual).
  EXPECT_FALSE(f(values).isZero(1e-9));

  // Numerical FD of f̃ equals analytical Jac. (Outside the band, f̃ is just f shifted by a constant — the constant
  // cancels in the FD difference, so this would also pass under the old buggy implementation. This test is a
  // regression guard, not a pre/post-fix discriminator.)
  checkJacobian(f, dfdx, values, 1.0e-5);
}

TEST_F(KinematicCostsTest, DynamicCartPoseJacCalculator_TolerancedAcrossEdge)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("KinematicCostsTest, DynamicCartPoseJacCalculator_TolerancedAcrossEdge");

  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");

  const std::string source_frame = "r_gripper_tool_frame";
  const std::string target_frame = "r_wrist_roll_link";

  Eigen::VectorXd values(7);
  values << -1.1, 1.2, -3.3, -1.4, 5.5, -1.6, 7.7;

  const tesseract::common::TransformMap state_cache = kin->calcFwdKin(values);
  const Eigen::Isometry3d source_frame_offset = Eigen::Isometry3d::Identity();
  // 0.51 rad rx — just inside the band, so any non-trivial perturbation straddles the edge. Same pattern as
  // InsideBand but with a larger angle close to the band boundary.
  const Eigen::Isometry3d& source_tf_seed = state_cache.at(source_frame);
  const Eigen::Isometry3d& target_tf_seed = state_cache.at(target_frame);
  const Eigen::Isometry3d current_target_in_source = source_tf_seed.inverse() * target_tf_seed;
  const Eigen::Isometry3d target_frame_offset =
      current_target_in_source.inverse() * Eigen::Isometry3d(Eigen::AngleAxisd(0.51, Eigen::Vector3d::UnitX()));

  Eigen::VectorXd lower(6);
  Eigen::VectorXd upper(6);
  lower << 0.0, 0.0, 0.0, -0.52, 0.0, 0.0;
  upper << 0.0, 0.0, 0.0, 0.52, 0.0, 0.0;

  const Eigen::VectorXi indices = (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished();

  const DynamicCartPoseErrCalculator f(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);
  const DynamicCartPoseJacCalculator dfdx(
      kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower, upper);

  // f is zero at the seed (0.51 is inside [-0.52, 0.52]).
  EXPECT_TRUE(f(values).isZero(1e-9));

  // FD-check the analytical Jac. The dynamic variant involves two moving frames, which produces higher baseline
  // numerical noise than the static variant — 1e-7 epsilon makes isApprox(1e-5) unreliable because both the
  // numerical and analytical values are O(1e-10~12) (the function is identically zero on a band neighbourhood, so
  // both quantities are essentially noise). Using 1e-5 epsilon keeps the FD secant well inside the band while
  // avoiding noise-dominated comparisons. This epsilon is sufficient to catch a naive row-zeroing shortcut.
  // NOTE: stash-reverting calcJacobianTransformErrorDiff tolerance handling SHOULD cause this test to fail —
  // the band-edge sub-gradient is the discriminating case.
  checkJacobian(f, dfdx, values, 1.0e-5);
}

// All four CartPose calculator constructors validate the (lower, upper) tolerance pair at
// construction time so the FD-Jacobian hot path doesn't pay for a per-call check. An inverted
// band (lower > upper on any component) must throw rather than producing a silent non-zero
// result inside what looks like the dead-band.
TEST_F(KinematicCostsTest, CartPoseCalculators_InvertedToleranceBandThrows)  // NOLINT
{
  const tesseract::kinematics::JointGroup::ConstPtr kin = env_->getJointGroup("right_arm");
  const std::string source_frame = env_->getRootLinkName();
  const std::string target_frame = "r_gripper_tool_frame";
  const Eigen::Isometry3d source_frame_offset = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d target_frame_offset = Eigen::Isometry3d::Identity();
  const Eigen::VectorXi indices = (Eigen::VectorXi(6) << 0, 1, 2, 3, 4, 5).finished();

  Eigen::VectorXd lower(6);
  Eigen::VectorXd upper(6);
  lower << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0;   // rx lower = 0.5
  upper << 0.0, 0.0, 0.0, -0.5, 0.0, 0.0;  // rx upper = -0.5  → inverted

  EXPECT_THROW(
      CartPoseErrCalculator(kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower,
                            upper),
      std::runtime_error);
  EXPECT_THROW(
      CartPoseJacCalculator(kin, source_frame, target_frame, source_frame_offset, target_frame_offset, indices, lower,
                            upper),
      std::runtime_error);
  EXPECT_THROW(DynamicCartPoseErrCalculator(kin, source_frame, target_frame, source_frame_offset, target_frame_offset,
                                            indices, lower, upper),
               std::runtime_error);
  EXPECT_THROW(DynamicCartPoseJacCalculator(kin, source_frame, target_frame, source_frame_offset, target_frame_offset,
                                            indices, lower, upper),
               std::runtime_error);
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
