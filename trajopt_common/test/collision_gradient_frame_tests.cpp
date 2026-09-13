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

#include <trajopt_common/collision_types.h>
#include <trajopt_common/collision_utils.h>

using namespace tesseract::collision;
using namespace tesseract::common;
using namespace tesseract::environment;
using namespace tesseract::kinematics;

// The two endpoints of one trajectory segment checked as five equal casts, and the cast a contact
// is reported from. cc_time is global to the segment and falls inside that cast.
namespace
{
const std::string kActiveLink = "r_wrist_roll_link";
const std::string kSecondActiveLink = "r_elbow_flex_link";
constexpr long kCastCount = 5;
constexpr double kSubStart = 0.4;
constexpr double kSubEnd = 0.6;
constexpr double kCcTime = 0.45;
constexpr double kMargin = 0.025;
constexpr double kMarginBuffer = 20.0;
const Eigen::Vector3d kWitnessLocal(0.06, -0.04, 0.03);

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

/** @brief A timestep's gradient of one link multiplied by its weight, which is what a consumer reads */
Eigen::VectorXd scaled(const trajopt_common::LinkGradientResults& g) { return g.scale * g.gradient; }
}  // namespace

class CollisionGradientFrameTest : public testing::Test
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>();
  JointGroup::ConstPtr manip_;
  const LinkId link_{ kActiveLink };
  const LinkId second_link_{ kSecondActiveLink };
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
    ASSERT_TRUE(manip_->isActiveLinkId(second_link_));
  }

  /** @brief A contact on an active link whose stored poses are those of the given configurations,
   * independent of the configurations the gradient will be linearised about. */
  ContactResult makeContact(const Eigen::VectorXd& pose_source,
                            const Eigen::VectorXd& cc_pose_source,
                            double cc_time,
                            ContinuousCollisionType cc_type) const
  {
    ContactResult cr;
    cr.link_ids[0] = link_;
    cr.link_ids[1] = env_->getRootLinkId();
    cr.nearest_points_local[0] = kWitnessLocal;
    cr.nearest_points_local[1] = Eigen::Vector3d::Zero();
    cr.transform[0] = manip_->calcFwdKin(pose_source).at(link_);
    cr.cc_transform[0] = manip_->calcFwdKin(cc_pose_source).at(link_);
    cr.cc_time[0] = cc_time;
    cr.cc_type[0] = cc_type;
    cr.cc_time[1] = cc_time;
    cr.cc_type[1] = cc_type;
    cr.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
    cr.distance = -0.01;
    cr.nearest_points[0] = cr.transform[0] * cr.nearest_points_local[0];
    return cr;
  }

  /**
   * @brief A contact a check pinned to one endpoint of the segment
   * @details Its witness lies on the link at @p witness_state, since that is where the check found
   * it. Both backends guarantee only that transform maps the stored local point to the reported
   * witness, and transform is the start of the cast whichever endpoint the contact is pinned to, so
   * the stored local point is that world point taken back through transform rather than the witness's
   * own local coordinates.
   */
  ContactResult makePinnedContact(const Eigen::VectorXd& pose_source,
                                  const Eigen::VectorXd& cc_pose_source,
                                  double cc_time,
                                  ContinuousCollisionType cc_type,
                                  const Eigen::VectorXd& witness_state) const
  {
    ContactResult cr = makeContact(pose_source, cc_pose_source, cc_time, cc_type);
    cr.nearest_points[0] = manip_->calcFwdKin(witness_state).at(link_) * kWitnessLocal;
    cr.nearest_points_local[0] = cr.transform[0].inverse() * cr.nearest_points[0];
    return cr;
  }

  /** @brief The gradient a correct implementation must return for a link linearised at q_jac: the
   * numerical derivative of the witness point's world position there, contracted with the normal. */
  Eigen::VectorXd referenceGradient(const ContactResult& cr, const Eigen::VectorXd& q_jac, std::size_t i = 0) const
  {
    return referenceGradientAt(cr, q_jac, cr.nearest_points_local[i], i);
  }

  /** @brief The reference gradient for a witness whose coordinates on the link are @p local_point */
  Eigen::VectorXd referenceGradientAt(const ContactResult& cr,
                                      const Eigen::VectorXd& q_jac,
                                      const Eigen::Vector3d& local_point,
                                      std::size_t i = 0) const
  {
    Eigen::MatrixXd num_jac(6, manip_->numJoints());
    numericalJacobian(num_jac, Eigen::Isometry3d::Identity(), *manip_, q_jac, cr.link_ids[i], local_point);
    return ((i == 0) ? -1.0 : 1.0) * cr.normal.transpose() * num_jac.topRows(3);
  }

  /** @brief The cast model's derivatives for link i of a contact found in the interval [start, end] */
  std::pair<Eigen::VectorXd, Eigen::VectorXd>
  linkDerivatives(const ContactResult& cr, std::size_t i, double start, double end) const
  {
    const Eigen::Vector3d signed_normal = ((i == 0) ? -1.0 : 1.0) * cr.normal;
    return intervalDerivatives(
        *manip_, cr.link_ids[i], cr.nearest_points_local[i], signed_normal, q0_, q1_, start, end, cr.cc_time[i]);
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
                              const Eigen::VectorXd& q_jac,
                              std::size_t i = 0) const
  {
    expectMatches(actual, referenceGradient(cr, q_jac, i));
  }
};

// A contact found in one cast of a subdivided segment moves with the link at both ends of that
// cast, so each timestep's gradient blends the jacobians at the two cast ends, each rotated by its
// own pose rather than by a pose the contact happens to carry
TEST_F(CollisionGradientFrameTest, BetweenContactBlendsItsCastEnds)  // NOLINT
{
  const ContactResult cr =
      makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime, ContinuousCollisionType::CCType_Between);

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, kCastCount);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  ASSERT_TRUE(results.cc_gradients[0].has_gradient);
  const auto [d0, d1] = linkDerivatives(cr, 0, kSubStart, kSubEnd);
  expectMatches(scaled(results.gradients[0]), d0);
  expectMatches(scaled(results.cc_gradients[0]), d1);
}

// Checked as one cast, the segment is the cast: the start timestep's gradient is the jacobian at
// the segment start and the end timestep's the jacobian at the segment end, weighted by 1 - cc_time
// and cc_time (Schulman et al. 2014, Eq. (20))
TEST_F(CollisionGradientFrameTest, SingleCastContactTakesEachTimestepAtItsOwnState)  // NOLINT
{
  const ContactResult cr = makeContact(q0_, q1_, kCcTime, ContinuousCollisionType::CCType_Between);

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, 1);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  ASSERT_TRUE(results.cc_gradients[0].has_gradient);
  expectMatchesReference(results.gradients[0].gradient, cr, q0_);
  expectMatchesReference(results.cc_gradients[0].gradient, cr, q1_);
  EXPECT_NEAR(results.gradients[0].scale, 1.0 - kCcTime, 1e-12);
  EXPECT_NEAR(results.cc_gradients[0].scale, kCcTime, 1e-12);
}

// A check at interpolated states finds a contact at a point in time, so both timesteps' gradients
// are taken at the state interpolated by cc_time and differ only in weight
TEST_F(CollisionGradientFrameTest, PointInTimeContactLinearisesAtItsState)  // NOLINT
{
  const Eigen::VectorXd q = lerp(q0_, q1_, kCcTime);
  const ContactResult cr = makeContact(q, q, kCcTime, ContinuousCollisionType::CCType_Between);

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, 0);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  ASSERT_TRUE(results.cc_gradients[0].has_gradient);
  expectMatchesReference(results.gradients[0].gradient, cr, q);
  expectMatchesReference(results.cc_gradients[0].gradient, cr, q);
  EXPECT_NEAR(results.gradients[0].scale, 1.0 - kCcTime, 1e-12);
  EXPECT_NEAR(results.cc_gradients[0].scale, kCcTime, 1e-12);
}

// An untyped contact on an active link is placed in its cast exactly as a typed one is. The absence
// of a continuous collision type leaves each timestep's gradient at full weight; it does not move
// the states the gradient is taken at.
TEST_F(CollisionGradientFrameTest, UntypedContactBlendsItsCastEndsAtFullWeight)  // NOLINT
{
  const ContactResult cr =
      makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime, ContinuousCollisionType::CCType_None);

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, kCastCount);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  ASSERT_TRUE(results.cc_gradients[0].has_gradient);
  const auto [d0, d1] = linkDerivatives(cr, 0, kSubStart, kSubEnd);
  expectMatches(results.gradients[0].gradient, d0 / (1.0 - kCcTime));
  expectMatches(results.cc_gradients[0].gradient, d1 / kCcTime);
  EXPECT_NEAR(results.gradients[0].scale, 1.0, 1e-12);
  EXPECT_NEAR(results.cc_gradients[0].scale, 1.0, 1e-12);
}

// A link the check gave no interval carries a negative cc_time. Both halves here are derived from
// one shared state, so unlike a per-half caller there is no separate endpoint to fall back to per
// timestep: the contact is linearised at the segment start for both halves rather than
// extrapolated outside the segment.
TEST_F(CollisionGradientFrameTest, UntimedContactLinearisesAtSegmentStart)  // NOLINT
{
  const ContactResult cr =
      makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), -1.0, ContinuousCollisionType::CCType_None);

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, kCastCount);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  ASSERT_TRUE(results.cc_gradients[0].has_gradient);
  expectMatchesReference(results.gradients[0].gradient, cr, q0_);
  expectMatchesReference(results.cc_gradients[0].gradient, cr, q0_);
}

// A contact pinned to a segment endpoint is a point in time there, so both halves take that
// endpoint's pose - including the timestep1 half of a Time0 contact, whose stored cc_transform is
// the far endpoint - and all the weight falls on that endpoint's timestep
TEST_F(CollisionGradientFrameTest, EndpointContactGradientsUseTheEndpointState)  // NOLINT
{
  const ContactResult at_t0 = makePinnedContact(q0_, q1_, 0.0, ContinuousCollisionType::CCType_Time0, q0_);
  const ContactResult at_t1 = makePinnedContact(q0_, q1_, 1.0, ContinuousCollisionType::CCType_Time1, q1_);

  trajopt_common::GradientResults t0;
  trajopt_common::GradientResults t1;
  trajopt_common::getGradient(t0, q0_, q1_, at_t0, kMargin, kMarginBuffer, *manip_, kCastCount);
  trajopt_common::getGradient(t1, q0_, q1_, at_t1, kMargin, kMarginBuffer, *manip_, kCastCount);

  ASSERT_TRUE(t0.gradients[0].has_gradient);
  ASSERT_TRUE(t1.gradients[0].has_gradient);
  expectMatches(t0.gradients[0].gradient, referenceGradientAt(at_t0, q0_, kWitnessLocal));
  expectMatches(t0.cc_gradients[0].gradient, referenceGradientAt(at_t0, q0_, kWitnessLocal));
  expectMatches(t1.gradients[0].gradient, referenceGradientAt(at_t1, q1_, kWitnessLocal));
  expectMatches(t1.cc_gradients[0].gradient, referenceGradientAt(at_t1, q1_, kWitnessLocal));
  EXPECT_NEAR(t0.gradients[0].scale, 1.0, 1e-12);
  EXPECT_NEAR(t0.cc_gradients[0].scale, 0.0, 1e-12);
  EXPECT_NEAR(t1.gradients[0].scale, 0.0, 1e-12);
  EXPECT_NEAR(t1.cc_gradients[0].scale, 1.0, 1e-12);
}

// Both links of a contact between two active links are placed in the same cast, and each is
// blended over that cast's ends with its own jacobians and poses
TEST_F(CollisionGradientFrameTest, ContactBetweenTwoActiveLinksBlendsBothOverTheirCast)  // NOLINT
{
  ContactResult cr =
      makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime, ContinuousCollisionType::CCType_Between);
  cr.link_ids[1] = second_link_;
  cr.nearest_points_local[1] = Eigen::Vector3d(-0.02, 0.05, 0.01);
  cr.transform[1] = manip_->calcFwdKin(lerp(q0_, q1_, kSubStart)).at(second_link_);
  cr.cc_transform[1] = manip_->calcFwdKin(lerp(q0_, q1_, kSubEnd)).at(second_link_);
  cr.nearest_points[1] = cr.transform[1] * cr.nearest_points_local[1];

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, kCastCount);

  ASSERT_TRUE(results.gradients[1].has_gradient);
  ASSERT_TRUE(results.cc_gradients[1].has_gradient);
  const auto [a0, a1] = linkDerivatives(cr, 0, kSubStart, kSubEnd);
  const auto [b0, b1] = linkDerivatives(cr, 1, kSubStart, kSubEnd);
  expectMatches(scaled(results.gradients[0]), a0);
  expectMatches(scaled(results.cc_gradients[0]), a1);
  expectMatches(scaled(results.gradients[1]), b0);
  expectMatches(scaled(results.cc_gradients[1]), b1);
}

// Two active links need not share a cast: a swept check times each link's contact independently,
// so each is placed in the cast holding its own cc_time
TEST_F(CollisionGradientFrameTest, ContactBetweenTwoActiveLinksWithDistinctTimes)  // NOLINT
{
  constexpr double kOtherSubStart = 0.6;
  constexpr double kOtherSubEnd = 0.8;
  constexpr double kOtherCcTime = 0.75;
  ContactResult cr =
      makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime, ContinuousCollisionType::CCType_Between);
  cr.link_ids[1] = second_link_;
  cr.nearest_points_local[1] = Eigen::Vector3d(-0.02, 0.05, 0.01);
  cr.transform[1] = manip_->calcFwdKin(lerp(q0_, q1_, kOtherSubStart)).at(second_link_);
  cr.cc_transform[1] = manip_->calcFwdKin(lerp(q0_, q1_, kOtherSubEnd)).at(second_link_);
  cr.nearest_points[1] = cr.transform[1] * cr.nearest_points_local[1];
  cr.cc_time[1] = kOtherCcTime;

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, kCastCount);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  ASSERT_TRUE(results.gradients[1].has_gradient);
  const auto [a0, a1] = linkDerivatives(cr, 0, kSubStart, kSubEnd);
  const auto [b0, b1] = linkDerivatives(cr, 1, kOtherSubStart, kOtherSubEnd);
  expectMatches(scaled(results.gradients[0]), a0);
  expectMatches(scaled(results.cc_gradients[0]), a1);
  expectMatches(scaled(results.gradients[1]), b0);
  expectMatches(scaled(results.cc_gradients[1]), b1);
}

// A discrete contact stores the pose of the state it was found at, so its gradient is already
// rotated by the configuration it is linearised about and must stay exactly where it is.
TEST_F(CollisionGradientFrameTest, DiscreteContactGradientIsUnchanged)  // NOLINT
{
  const Eigen::VectorXd q = lerp(q0_, q1_, kCcTime);
  const ContactResult cr = makeContact(q, q, 0.0, ContinuousCollisionType::CCType_None);

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q, cr, kMargin, kMarginBuffer, *manip_);

  ASSERT_TRUE(results.gradients[0].has_gradient);
  EXPECT_FALSE(results.cc_gradients[0].has_gradient);
  expectMatchesReference(results.gradients[0].gradient, cr, q);
}

// Splitting a timestep's gradient between the ends of its cast leaves its total weight, the time
// weighting, where it was
TEST_F(CollisionGradientFrameTest, TimeWeightingIsUnchanged)  // NOLINT
{
  const ContactResult cr =
      makeContact(lerp(q0_, q1_, kSubStart), lerp(q0_, q1_, kSubEnd), kCcTime, ContinuousCollisionType::CCType_Between);

  trajopt_common::GradientResults results;
  trajopt_common::getGradient(results, q0_, q1_, cr, kMargin, kMarginBuffer, *manip_, kCastCount);

  EXPECT_NEAR(results.gradients[0].scale, 1.0 - kCcTime, 1e-12);
  EXPECT_NEAR(results.cc_gradients[0].scale, kCcTime, 1e-12);
  EXPECT_EQ(results.gradients[0].cc_type, ContinuousCollisionType::CCType_Between);
  EXPECT_EQ(results.cc_gradients[0].cc_type, ContinuousCollisionType::CCType_Between);
}
