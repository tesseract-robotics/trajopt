#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <utility>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt_common/logging.hpp>

using namespace trajopt;
using namespace tesseract::collision;
using namespace tesseract::common;
using namespace tesseract::environment;
using namespace tesseract::kinematics;

namespace
{
/** @brief Exposes the base evaluator's gradient entry points with a settable cast count; the
 * collision entry points are unused here and are given the smallest bodies that satisfy the
 * interface. */
struct GradientOnlyEvaluator : public CollisionEvaluator
{
  GradientOnlyEvaluator(const JointGroup::ConstPtr& manip, Environment::ConstPtr env)
    : CollisionEvaluator(manip, std::move(env), false)
  {
  }

  void CalcDistExpressions(const DblVec& /*x*/,
                           sco::AffExprVector& /*exprs*/,
                           std::vector<double>& /*exprs_margin*/,
                           std::vector<double>& /*exprs_coeff*/) override
  {
  }
  void CalcCollisions(const DblVec& /*x*/, ContactResultMap& /*dist_results*/) override {}
  void Plot(const std::shared_ptr<tesseract::visualization::Visualization>& /*plotter*/, const DblVec& /*x*/) override
  {
  }
  sco::VarVector GetVars() override { return {}; }
  long GetCastCount(const Eigen::Ref<const Eigen::VectorXd>& /*dofvals0*/,
                    const Eigen::Ref<const Eigen::VectorXd>& /*dofvals1*/) const override
  {
    return cast_count;
  }

  /** @brief The cast count the two-state gradients place contacts by */
  long cast_count{ 0 };
};
}  // namespace

// The two endpoints of one trajectory segment checked as five equal casts, and the cast a contact
// is reported from. cc_time is global to the segment and falls inside that cast.
namespace
{
const std::string kActiveLink = "r_wrist_roll_link";
constexpr long kCastCount = 5;
constexpr double kSubStart = 0.4;
constexpr double kSubEnd = 0.6;
constexpr double kCcTime = 0.45;

Eigen::VectorXd segmentStart() { return (Eigen::VectorXd(7) << -1.1, 1.2, -1.5, -1.4, -1.1, -1.3, 0.2).finished(); }
Eigen::VectorXd segmentEnd() { return (Eigen::VectorXd(7) << -0.3, 0.7, -0.9, -0.8, -0.4, -0.7, 0.9).finished(); }
Eigen::VectorXd lerp(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double t) { return a + (b - a) * t; }

/**
 * @brief The derivatives of one link's contact distance under the cast model, with respect to the
 * segment start and end
 *
 * The link's contact point is the fixed blend, at the contact time's fraction of the interval
 * [start, end], of the witness point the link carries at the configurations interpolated to start
 * and end; the normal is held fixed. Taken by central differences of forward kinematics, so it
 * shares no code with the implementation. A point in time is the interval [time, time].
 * @return The derivative with respect to the segment start, then the one with respect to its end
 */
std::pair<Eigen::VectorXd, Eigen::VectorXd> intervalDerivatives(const JointGroup& manip,
                                                                const LinkId& link,
                                                                const Eigen::Vector3d& local_point,
                                                                const Eigen::Vector3d& signed_normal,
                                                                const Eigen::VectorXd& q0,
                                                                const Eigen::VectorXd& q1,
                                                                double start,
                                                                double end,
                                                                double time)
{
  const double tau = (end > start) ? (time - start) / (end - start) : 0.0;
  auto distance = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    const Eigen::Vector3d at_start = manip.calcFwdKin(lerp(a, b, start)).at(link) * local_point;
    const Eigen::Vector3d at_end = manip.calcFwdKin(lerp(a, b, end)).at(link) * local_point;
    return signed_normal.dot(((1.0 - tau) * at_start) + (tau * at_end));
  };

  constexpr double h = 1e-6;
  const Eigen::Index n = q0.size();
  Eigen::VectorXd d0(n);
  Eigen::VectorXd d1(n);
  for (Eigen::Index j = 0; j < n; ++j)
  {
    Eigen::VectorXd step = Eigen::VectorXd::Zero(n);
    step(j) = h;
    d0(j) = (distance(q0 + step, q1) - distance(q0 - step, q1)) / (2.0 * h);
    d1(j) = (distance(q0, q1 + step) - distance(q0, q1 - step)) / (2.0 * h);
  }
  return { d0, d1 };
}
}  // namespace

class CastGradientFrameTest : public testing::Test
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>();
  JointGroup::ConstPtr manip_;
  std::shared_ptr<GradientOnlyEvaluator> evaluator_;
  const LinkId link_{ kActiveLink };
  const Eigen::VectorXd q0_{ segmentStart() };
  const Eigen::VectorXd q1_{ segmentEnd() };

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<GeneralResourceLocator>();
    ASSERT_TRUE(env_->init(urdf_file, srdf_file, locator));

    manip_ = env_->getJointGroup("right_arm");
    ASSERT_TRUE(manip_ != nullptr);
    ASSERT_TRUE(manip_->isActiveLinkId(link_));
    evaluator_ = std::make_shared<GradientOnlyEvaluator>(manip_, env_);

    trajopt_common::gLogLevel = trajopt_common::LevelError;
  }

  /** @brief A contact on an active link, typed as occurring between two configurations, whose
   * stored poses are those of an arbitrary interval rather than of q_jac. */
  ContactResult makeContact(const Eigen::VectorXd& pose_source,
                            const Eigen::VectorXd& cc_pose_source,
                            double cc_time) const
  {
    ContactResult cr;
    cr.link_ids[0] = link_;
    cr.link_ids[1] = env_->getRootLinkId();
    cr.nearest_points_local[0] = Eigen::Vector3d(0.06, -0.04, 0.03);
    cr.nearest_points_local[1] = Eigen::Vector3d::Zero();
    cr.transform[0] = manip_->calcFwdKin(pose_source).at(link_);
    cr.cc_transform[0] = manip_->calcFwdKin(cc_pose_source).at(link_);
    cr.cc_time[0] = cc_time;
    cr.cc_type[0] = ContinuousCollisionType::CCType_Between;
    cr.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
    cr.distance = -0.01;
    return cr;
  }

  /** @brief The gradient a correct implementation must return for link A: the numerical derivative
   * of the witness point's world position at q_jac, contracted with the contact normal. */
  Eigen::VectorXd referenceGradient(const ContactResult& cr, const Eigen::VectorXd& q_jac) const
  {
    Eigen::MatrixXd num_jac(6, manip_->numJoints());
    numericalJacobian(
        num_jac, Eigen::Isometry3d::Identity(), *manip_, q_jac, cr.link_ids[0], cr.nearest_points_local[0]);
    return -1.0 * cr.normal.transpose() * num_jac.topRows(3);
  }

  /** @brief Compare against an expected value, refusing to compare two vanishing vectors.
   * isApprox is a relative test and is satisfied by any pair of near-zero vectors, so a
   * configuration that produced no usable gradient would satisfy every comparison in this file
   * whatever the implementation does. The floor is what makes a pass mean something. */
  static void expectMatches(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected)
  {
    EXPECT_GT(expected.norm(), 1e-2) << "the configuration produces no usable gradient, so the "
                                        "comparison below would hold for any implementation";
    EXPECT_TRUE(actual.isApprox(expected, 1e-4))
        << "got      " << actual.transpose() << "\nexpected " << expected.transpose();
  }

  void expectMatchesReference(const Eigen::VectorXd& actual,
                              const ContactResult& cr,
                              const Eigen::VectorXd& q_jac) const
  {
    expectMatches(actual, referenceGradient(cr, q_jac));
  }
};

TEST_F(CastGradientFrameTest, SubdividedContactGradientAtSegmentStart)  // NOLINT
{
  const ContactResult cr = makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime);

  const GradientResults results = evaluator_->GetGradient(q0_, cr, 0.025, 20.0, false);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  expectMatchesReference(results.gradients[0].gradient, cr, q0_);
}

TEST_F(CastGradientFrameTest, SubdividedContactGradientAtSegmentEnd)  // NOLINT
{
  const ContactResult cr = makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime);

  const GradientResults results = evaluator_->GetGradient(q1_, cr, 0.025, 20.0, true);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  expectMatchesReference(results.gradients[0].gradient, cr, q1_);
}

// With the stored poses equal to the poses at the evaluation configurations — the state of an
// unsubdivided check — the gradient must match the reference exactly: an unsubdivided contact
// carries no separate linearisation pose, so there is nothing for the reference frame to correct.
TEST_F(CastGradientFrameTest, UnsubdividedContactGradientIsUnchanged)  // NOLINT
{
  const ContactResult cr = makeContact(q0_, q1_, kCcTime);

  const GradientResults at_start = evaluator_->GetGradient(q0_, cr, 0.025, 20.0, false);
  const GradientResults at_end = evaluator_->GetGradient(q1_, cr, 0.025, 20.0, true);

  ASSERT_TRUE(at_start.gradients[0].has_gradient);
  ASSERT_TRUE(at_end.gradients[0].has_gradient);
  expectMatchesReference(at_start.gradients[0].gradient, cr, q0_);
  expectMatchesReference(at_end.gradients[0].gradient, cr, q1_);
}

// A contact found in one cast of a subdivided segment moves with the link at both ends of that
// cast. The two-state entry point places it in its cast from the evaluator's cast count and blends
// the gradients at the cast ends, each rotated by its own pose rather than by a stored one.
TEST_F(CastGradientFrameTest, TwoStateGradientBlendsItsCastEnds)  // NOLINT
{
  evaluator_->cast_count = kCastCount;
  const ContactResult cr = makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime);

  const GradientResults at_start = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, false);
  const GradientResults at_end = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, true);

  ASSERT_TRUE(at_start.gradients[0].has_gradient);
  ASSERT_TRUE(at_end.gradients[0].has_gradient);
  const auto [d0, d1] = intervalDerivatives(
      *manip_, link_, cr.nearest_points_local[0], -cr.normal, q0_, q1_, kSubStart, kSubEnd, kCcTime);
  expectMatches(at_start.gradients[0].scale * at_start.gradients[0].gradient, d0);
  expectMatches(at_end.gradients[0].scale * at_end.gradients[0].gradient, d1);
}

// Checked as one cast, the segment is the cast: each timestep's two-state gradient is the
// single-state gradient at that timestep's own state, weighted by 1 - cc_time and cc_time
// (Schulman et al. 2014, Eq. (20))
TEST_F(CastGradientFrameTest, SingleCastTwoStateGradientIsTheSingleStateGradientAtEachEnd)  // NOLINT
{
  evaluator_->cast_count = 1;
  const ContactResult cr = makeContact(q0_, q1_, kCcTime);

  const GradientResults two_state_start = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, false);
  const GradientResults two_state_end = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, true);
  const GradientResults at_start = evaluator_->GetGradient(q0_, cr, 0.025, 20.0, false);
  const GradientResults at_end = evaluator_->GetGradient(q1_, cr, 0.025, 20.0, true);

  ASSERT_TRUE(two_state_start.gradients[0].has_gradient);
  ASSERT_TRUE(two_state_end.gradients[0].has_gradient);
  expectMatchesReference(two_state_start.gradients[0].gradient, cr, q0_);
  expectMatchesReference(two_state_end.gradients[0].gradient, cr, q1_);
  EXPECT_TRUE(two_state_start.gradients[0].gradient.isApprox(at_start.gradients[0].gradient, 1e-12));
  EXPECT_TRUE(two_state_end.gradients[0].gradient.isApprox(at_end.gradients[0].gradient, 1e-12));
  EXPECT_NEAR(two_state_start.gradients[0].scale, at_start.gradients[0].scale, 1e-12);
  EXPECT_NEAR(two_state_end.gradients[0].scale, at_end.gradients[0].scale, 1e-12);
}

// A check at interpolated states finds a contact at a point in time, so both timesteps' gradients
// are taken at the state interpolated by cc_time, rotated by that state's pose rather than by
// either stored pose
TEST_F(CastGradientFrameTest, PointInTimeTwoStateGradientAtInterpolatedState)  // NOLINT
{
  evaluator_->cast_count = 0;
  const ContactResult cr = makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime);

  const GradientResults at_start = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, false);
  const GradientResults at_end = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, true);

  ASSERT_TRUE(at_start.gradients[0].has_gradient);
  ASSERT_TRUE(at_end.gradients[0].has_gradient);
  expectMatchesReference(at_start.gradients[0].gradient, cr, lerp(q0_, q1_, kCcTime));
  expectMatchesReference(at_end.gradients[0].gradient, cr, lerp(q0_, q1_, kCcTime));
}

// The time weighting is a separate quantity from the reference frame and must not move.
TEST_F(CastGradientFrameTest, TimeWeightingIsUnchanged)  // NOLINT
{
  const ContactResult cr = makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime);

  EXPECT_NEAR(evaluator_->GetGradient(q0_, cr, 0.025, 20.0, false).gradients[0].scale, 1.0 - kCcTime, 1e-12);
  EXPECT_NEAR(evaluator_->GetGradient(q1_, cr, 0.025, 20.0, true).gradients[0].scale, kCcTime, 1e-12);
}

// A link the check gave no interval carries a negative time, so there is no cast to place it in.
// Such a contact must be treated as occurring at the state being linearised, at full weight,
// rather than extrapolated outside the segment.
TEST_F(CastGradientFrameTest, UntimedContactLinearisesAtTheEndpoint)  // NOLINT
{
  evaluator_->cast_count = kCastCount;
  ContactResult cr = makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime);
  cr.cc_type[0] = ContinuousCollisionType::CCType_None;
  cr.cc_time[0] = -1.0;

  const GradientResults at_start = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, false);
  const GradientResults at_end = evaluator_->GetGradient(q0_, q1_, cr, 0.025, 20.0, true);

  ASSERT_TRUE(at_start.gradients[0].has_gradient);
  ASSERT_TRUE(at_end.gradients[0].has_gradient);
  expectMatchesReference(at_start.gradients[0].gradient, cr, q0_);
  expectMatchesReference(at_end.gradients[0].gradient, cr, q1_);
  EXPECT_NEAR(at_start.gradients[0].scale, 1.0, 1e-12);
  EXPECT_NEAR(at_end.gradients[0].scale, 1.0, 1e-12);
}
