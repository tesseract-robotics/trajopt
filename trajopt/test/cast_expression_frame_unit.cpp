#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <tesseract/common/resource_locator.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/logging.hpp>

using namespace trajopt;
using namespace tesseract::collision;
using namespace tesseract::environment;
using namespace tesseract::kinematics;
using tesseract::common::LinkId;

namespace
{
/** @brief One variable per joint, indexed from offset. No solver is involved; only the index
 * matters, because an expression records variables and the value vector is read by index. */
sco::VarVector makeVars(std::size_t offset, std::size_t n)
{
  sco::VarVector vars;
  vars.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    vars.emplace_back(std::make_shared<sco::VarRep>(offset + i, "v" + std::to_string(offset + i), nullptr));
  return vars;
}

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

/**
 * @brief The cast whose end poses link i of a contact carries, when the segment is split into
 * cast_count equal casts
 *
 * Identifies the cast from the contact's stored poses rather than its time, so a contact placed in
 * the wrong cast by the implementation cannot be placed there identically here.
 */
std::optional<long> matchCast(const ContactResult& cr,
                              std::size_t i,
                              const JointGroup& manip,
                              const Eigen::VectorXd& q0,
                              const Eigen::VectorXd& q1,
                              long cast_count)
{
  const auto count = static_cast<double>(cast_count);
  for (long k = 0; k < cast_count; ++k)
  {
    const Eigen::Isometry3d at_start =
        manip.calcFwdKin(lerp(q0, q1, static_cast<double>(k) / count)).at(cr.link_ids[i]);
    const Eigen::Isometry3d at_end =
        manip.calcFwdKin(lerp(q0, q1, static_cast<double>(k + 1) / count)).at(cr.link_ids[i]);
    if ((at_start.matrix() - cr.transform[i].matrix()).norm() < 1e-9 &&
        (at_end.matrix() - cr.cc_transform[i].matrix()).norm() < 1e-9)
      return k;
  }
  return std::nullopt;
}
}  // namespace

class CastExpressionFrameTest : public testing::Test
{
public:
  void SetUp() override { trajopt_common::gLogLevel = trajopt_common::LevelError; }

  static Environment::Ptr loadEnvironment(const std::string& urdf, const std::string& srdf)
  {
    auto env = std::make_shared<Environment>();
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/" + urdf);
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/" + srdf);
    const tesseract::common::ResourceLocator::Ptr locator =
        std::make_shared<tesseract::common::GeneralResourceLocator>();
    if (!env->init(urdf_file, srdf_file, locator))
      return nullptr;
    return env;
  }

  /**
   * @brief The coefficients a correct expression must carry for one contact of a cast check
   *
   * Each active link's contact point is the fixed blend, at the contact's fraction of its cast, of
   * the witness point the link carries at the cast's two end states. The cast is identified from
   * the poses the contact carries, not from its time.
   */
  static Eigen::VectorXd referenceCoeffs(const ContactResult& cr,
                                         const JointGroup& manip,
                                         const Eigen::VectorXd& q0,
                                         const Eigen::VectorXd& q1,
                                         long cast_count)
  {
    const auto n = static_cast<Eigen::Index>(manip.numJoints());
    Eigen::VectorXd expected = Eigen::VectorXd::Zero(2 * n);
    const auto count = static_cast<double>(cast_count);
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (!manip.isActiveLinkId(cr.link_ids[i]))
        continue;

      // A cast check times every active link
      EXPECT_GE(cr.cc_time[i], 0.0);
      const std::optional<long> cast = matchCast(cr, i, manip, q0, q1, cast_count);
      if (!cast.has_value())
      {
        ADD_FAILURE() << "link " << i << " carries the end poses of no cast";
        continue;
      }
      const Eigen::Vector3d signed_normal = ((i == 0) ? -1.0 : 1.0) * cr.normal;
      const double start = static_cast<double>(*cast) / count;
      const double end = static_cast<double>(*cast + 1) / count;

      // A contact whose time names an end of its cast was found with the link at that end, so the
      // witness it reports in world coordinates lies on the link there. Away from its own time the
      // witness lies inside the volume the link sweeps, on neither end's surface, and the stored local
      // point stands in for it as a fixed point of the link. The end that does not carry the contact's
      // time carries no weight, so one point serves both.
      Eigen::Vector3d local_point = cr.nearest_points_local[i];
      if (const double s = tesseract::common::almostEqualRelativeAndAbs(cr.cc_time[i], start) ? start : end;
          tesseract::common::almostEqualRelativeAndAbs(cr.cc_time[i], s))
        local_point = manip.calcFwdKin(lerp(q0, q1, s)).at(cr.link_ids[i]).inverse() * cr.nearest_points[i];

      const auto [d0, d1] = intervalDerivatives(manip,
                                                cr.link_ids[i],
                                                local_point,
                                                signed_normal,
                                                q0,
                                                q1,
                                                start,
                                                end,
                                                cr.cc_time[i]);
      expected.head(n) += d0;
      expected.tail(n) += d1;
    }
    return expected;
  }

  /** @brief How many active links of a contact lie well inside their cast, where the cast model's
   * two ends differ from any single state */
  static int interiorLinks(const ContactResult& cr,
                           const JointGroup& manip,
                           const Eigen::VectorXd& q0,
                           const Eigen::VectorXd& q1,
                           long cast_count)
  {
    int interior = 0;
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (!manip.isActiveLinkId(cr.link_ids[i]))
        continue;
      const std::optional<long> cast = matchCast(cr, i, manip, q0, q1, cast_count);
      if (!cast.has_value())
        continue;
      const double tau = (cr.cc_time[i] * static_cast<double>(cast_count)) - static_cast<double>(*cast);
      if (tau > 0.05 && tau < 0.95)
        ++interior;
    }
    return interior;
  }

  /**
   * @brief The block of a full two-state coefficient vector that @p eval_type's expression populates:
   * the segment end's for a fixed start, the segment start's for a fixed end, both for neither fixed.
   */
  static Eigen::VectorXd freeBlock(const Eigen::VectorXd& full, CollisionExpressionEvaluatorType eval_type, long n)
  {
    switch (eval_type)
    {
      case CollisionExpressionEvaluatorType::START_FIXED_END_FREE:
        return full.tail(n);
      case CollisionExpressionEvaluatorType::START_FREE_END_FIXED:
        return full.head(n);
      default:
        return full;
    }
  }

  /**
   * @brief Build a cast evaluator's expressions for one segment and compare every contact's
   * coefficients against the cast model, over whichever timestep @p eval_type leaves free
   * @return How many active links of the contacts lie well inside their cast
   */
  static int checkSegment(const Environment::Ptr& env,
                          const std::string& group,
                          const Eigen::VectorXd& q0,
                          const Eigen::VectorXd& q1,
                          CollisionEvaluatorType type,
                          double longest_valid_segment_length,
                          long cast_count,
                          CollisionExpressionEvaluatorType eval_type)
  {
    auto manip = env->getJointGroup(group);
    const auto n = static_cast<Eigen::Index>(manip->numJoints());
    const auto un = static_cast<std::size_t>(n);

    trajopt_common::TrajOptCollisionConfig config(0.02, 1);
    config.collision_check_config.type = type;
    config.collision_check_config.longest_valid_segment_length = longest_valid_segment_length;
    config.collision_margin_buffer = 0.05;

    CastCollisionEvaluator evaluator(manip, env, config, makeVars(0, un), makeVars(un, un), eval_type);
    EXPECT_EQ(evaluator.GetCastCount(q0, q1), cast_count);

    DblVec x(q0.data(), q0.data() + n);
    x.insert(x.end(), q1.data(), q1.data() + n);

    sco::AffExprVector exprs;
    std::vector<double> exprs_margin;
    std::vector<double> exprs_coeff;
    evaluator.CalcDistExpressions(x, exprs, exprs_margin, exprs_coeff);

    const ContactResultVectorConstPtr contacts = evaluator.GetContactResultVectorCached(x);
    EXPECT_TRUE(contacts != nullptr && !contacts->empty());
    if (contacts == nullptr)
      return 0;

    // Every contact yields an expression: setActiveCollisionObjects(manip->getActiveLinkIds()) means
    // every reported pair has at least one active link, so none are ever filtered out here.
    EXPECT_EQ(exprs.size(), contacts->size());
    if (exprs.size() != contacts->size())
      return 0;

    int interior = 0;
    for (std::size_t c = 0; c < contacts->size(); ++c)
    {
      const ContactResult& cr = (*contacts)[c].get();
      interior += interiorLinks(cr, *manip, q0, q1, cast_count);

      const Eigen::VectorXd expected = freeBlock(referenceCoeffs(cr, *manip, q0, q1, cast_count), eval_type, n);
      // isApprox is a relative test and is satisfied by any pair of near-zero vectors, so a contact
      // whose normal lies near the null space of the witness point's Jacobian would pass regardless
      // of implementation without this floor.
      EXPECT_GT(expected.norm(), 1e-2) << "contact " << c
                                       << " produces no usable gradient, so the comparison below would "
                                          "hold for any implementation";
      // Converted the same way the QP builder converts it, so the comparison is against the row the
      // solver would actually see; sized for both timesteps' variables since a fixed timestep's vars
      // still occupy their half of the index space, just unused by this expression.
      Eigen::SparseVector<double> actual_sparse;
      sco::exprToEigen(exprs[c], actual_sparse, 2 * n);
      const Eigen::VectorXd actual = freeBlock(Eigen::VectorXd(actual_sparse), eval_type, n);

      EXPECT_TRUE(actual.isApprox(expected, 1e-4))
          << "contact " << c << "\ngot      " << actual.transpose() << "\nexpected " << expected.transpose();
    }
    return interior;
  }
};

// A right-arm PR2 near a table: a chain with genuine rotational joints, so a link's Jacobian
// varies with configuration. A purely prismatic manipulator's Jacobian is constant everywhere,
// which would make an expression built at the wrong states numerically indistinguishable from one
// built at the right ones, and this test would prove nothing. The segment is 1.72 long in joint
// space, so 35 casts of at most 0.05.
TEST_F(CastExpressionFrameTest, SubdividedPr2ContactsBlendTheirCastEnds)  // NOLINT
{
  const Environment::Ptr env = loadEnvironment("arm_around_table.urdf", "pr2.srdf");
  ASSERT_TRUE(env != nullptr);

  Eigen::VectorXd q0(7);
  q0 << -1.1, 1.2, -1.5, -1.4, -1.1, -1.3, 0.2;
  Eigen::VectorXd q1(7);
  q1 << -0.3, 0.7, -0.9, -0.8, -0.4, -0.7, 0.9;

  const int interior = checkSegment(env,
                                    "right_arm",
                                    q0,
                                    q1,
                                    CollisionEvaluatorType::LVS_CONTINUOUS,
                                    0.05,
                                    35,
                                    CollisionExpressionEvaluatorType::START_FREE_END_FREE);
  EXPECT_GT(interior, 0) << "no contact lies well inside its cast, so a gradient taken at a single state would pass "
                            "too";
}

// The sphere arm's tip sweeps through the obstacle. Checked as four casts, the first cast's
// contact lies well inside it.
TEST_F(CastExpressionFrameTest, SubdividedSphereArmContactsBlendTheirCastEnds)  // NOLINT
{
  const Environment::Ptr env = loadEnvironment("sphere_arm.urdf", "sphere_arm.srdf");
  ASSERT_TRUE(env != nullptr);

  Eigen::VectorXd q0(3);
  q0 << 0.5482, -1.2413, 1.9887;
  Eigen::VectorXd q1(3);
  q1 << 0.0152, -1.8207, 1.8467;

  const int interior = checkSegment(env,
                                    "manipulator",
                                    q0,
                                    q1,
                                    CollisionEvaluatorType::LVS_CONTINUOUS,
                                    0.2,
                                    4,
                                    CollisionExpressionEvaluatorType::START_FREE_END_FREE);
  EXPECT_GT(interior, 0) << "no contact lies well inside its cast, so a gradient taken at a single state would pass "
                            "too";
}

// The sphere arm's contacts are all Between, so removeInvalidContactResults keeps them even with
// the segment start fixed: no contact sits exactly at the start, which is the only case it drops.
TEST_F(CastExpressionFrameTest, SubdividedSphereArmFixedStartContactsBlendTheirCastEnds)  // NOLINT
{
  const Environment::Ptr env = loadEnvironment("sphere_arm.urdf", "sphere_arm.srdf");
  ASSERT_TRUE(env != nullptr);

  Eigen::VectorXd q0(3);
  q0 << 0.5482, -1.2413, 1.9887;
  Eigen::VectorXd q1(3);
  q1 << 0.0152, -1.8207, 1.8467;

  const int interior = checkSegment(env,
                                    "manipulator",
                                    q0,
                                    q1,
                                    CollisionEvaluatorType::LVS_CONTINUOUS,
                                    0.2,
                                    4,
                                    CollisionExpressionEvaluatorType::START_FIXED_END_FREE);
  EXPECT_GT(interior, 0) << "no contact lies well inside its cast, so a gradient taken at a single state would pass "
                            "too";
}

// As SubdividedSphereArmFixedStartContactsBlendTheirCastEnds, with the segment end fixed instead.
TEST_F(CastExpressionFrameTest, SubdividedSphereArmFixedEndContactsBlendTheirCastEnds)  // NOLINT
{
  const Environment::Ptr env = loadEnvironment("sphere_arm.urdf", "sphere_arm.srdf");
  ASSERT_TRUE(env != nullptr);

  Eigen::VectorXd q0(3);
  q0 << 0.5482, -1.2413, 1.9887;
  Eigen::VectorXd q1(3);
  q1 << 0.0152, -1.8207, 1.8467;

  const int interior = checkSegment(env,
                                    "manipulator",
                                    q0,
                                    q1,
                                    CollisionEvaluatorType::LVS_CONTINUOUS,
                                    0.2,
                                    4,
                                    CollisionExpressionEvaluatorType::START_FREE_END_FIXED);
  EXPECT_GT(interior, 0) << "no contact lies well inside its cast, so a gradient taken at a single state would pass "
                            "too";
}

// Cast once, a contact's expression takes the start timestep's jacobian at the segment start and
// the end timestep's at the segment end (Schulman et al. 2014, Eq. (20)). CONTINUOUS casts once
// whatever the longest valid segment length says.
TEST_F(CastExpressionFrameTest, SingleCastSphereArmContactTakesEachTimestepAtItsOwnState)  // NOLINT
{
  const Environment::Ptr env = loadEnvironment("sphere_arm.urdf", "sphere_arm.srdf");
  ASSERT_TRUE(env != nullptr);

  Eigen::VectorXd q0(3);
  q0 << 0.5482, -1.2413, 1.9887;
  Eigen::VectorXd q1(3);
  q1 << 0.0152, -1.8207, 1.8467;

  const int interior = checkSegment(env,
                                    "manipulator",
                                    q0,
                                    q1,
                                    CollisionEvaluatorType::CONTINUOUS,
                                    0.2,
                                    1,
                                    CollisionExpressionEvaluatorType::START_FREE_END_FREE);
  EXPECT_GT(interior, 0) << "no contact lies well inside the cast, so a gradient taken at a single state would pass "
                            "too";
}

// As SingleCastSphereArmContactTakesEachTimestepAtItsOwnState, with the segment start fixed: the
// one cast is one endpoint's expression, at that endpoint's own state.
TEST_F(CastExpressionFrameTest, SingleCastSphereArmFixedStartContactTakesItsOwnState)  // NOLINT
{
  const Environment::Ptr env = loadEnvironment("sphere_arm.urdf", "sphere_arm.srdf");
  ASSERT_TRUE(env != nullptr);

  Eigen::VectorXd q0(3);
  q0 << 0.5482, -1.2413, 1.9887;
  Eigen::VectorXd q1(3);
  q1 << 0.0152, -1.8207, 1.8467;

  const int interior = checkSegment(env,
                                    "manipulator",
                                    q0,
                                    q1,
                                    CollisionEvaluatorType::CONTINUOUS,
                                    0.2,
                                    1,
                                    CollisionExpressionEvaluatorType::START_FIXED_END_FREE);
  EXPECT_GT(interior, 0) << "no contact lies well inside the cast, so a gradient taken at a single state would pass "
                            "too";
}

// As SingleCastSphereArmFixedStartContactTakesItsOwnState, with the segment end fixed instead.
TEST_F(CastExpressionFrameTest, SingleCastSphereArmFixedEndContactTakesItsOwnState)  // NOLINT
{
  const Environment::Ptr env = loadEnvironment("sphere_arm.urdf", "sphere_arm.srdf");
  ASSERT_TRUE(env != nullptr);

  Eigen::VectorXd q0(3);
  q0 << 0.5482, -1.2413, 1.9887;
  Eigen::VectorXd q1(3);
  q1 << 0.0152, -1.8207, 1.8467;

  const int interior = checkSegment(env,
                                    "manipulator",
                                    q0,
                                    q1,
                                    CollisionEvaluatorType::CONTINUOUS,
                                    0.2,
                                    1,
                                    CollisionExpressionEvaluatorType::START_FREE_END_FIXED);
  EXPECT_GT(interior, 0) << "no contact lies well inside the cast, so a gradient taken at a single state would pass "
                            "too";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
