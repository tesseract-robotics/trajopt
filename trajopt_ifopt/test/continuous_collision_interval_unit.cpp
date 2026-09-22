#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>

#include <trajopt_common/collision_types.h>
#include <trajopt_common/logging.hpp>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>

using namespace trajopt_ifopt;
using namespace tesseract::collision;
using namespace tesseract::environment;
using namespace tesseract::kinematics;
using tesseract::common::LinkId;

namespace
{
// A segment of the sphere arm whose tip sweeps through the obstacle, 0.79997 long in joint space.
// Checked as one cast it reports one contact, a fifth of the way into the sweep; checked as four
// casts, the first cast's contact lies well inside it; checked at 17 interpolated states, the first
// 11 are within the contact distance.
Eigen::VectorXd segmentStart() { return (Eigen::VectorXd(3) << 0.5482, -1.2413, 1.9887).finished(); }
Eigen::VectorXd segmentEnd() { return (Eigen::VectorXd(3) << 0.0152, -1.8207, 1.8467).finished(); }
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

/** @brief Compare against an expected value, refusing to compare two vanishing vectors: isApprox is
 * a relative test that any pair of near-zero vectors satisfies */
void expectMatches(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected, std::size_t contact)
{
  EXPECT_GT(expected.norm(), 1e-2) << "contact " << contact
                                   << " produces no usable gradient, so the comparison below would hold "
                                      "for any implementation";
  EXPECT_TRUE(actual.isApprox(expected, 1e-4))
      << "contact " << contact << "\ngot      " << actual.transpose() << "\nexpected " << expected.transpose();
}
}  // namespace

class ContinuousCollisionIntervalTest : public testing::Test
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>();
  JointGroup::ConstPtr manip_;
  const Eigen::VectorXd q0_{ segmentStart() };
  const Eigen::VectorXd q1_{ segmentEnd() };

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/sphere_arm.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/sphere_arm.srdf");

    const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    ASSERT_TRUE(env_->init(urdf_file, srdf_file, locator));

    manip_ = env_->getJointGroup("manipulator");
    ASSERT_TRUE(manip_ != nullptr);

    trajopt_common::gLogLevel = trajopt_common::LevelError;
  }

  static trajopt_common::TrajOptCollisionConfig makeConfig(CollisionEvaluatorType type,
                                                           double longest_valid_segment_length)
  {
    trajopt_common::TrajOptCollisionConfig config(0.02, 1);
    config.collision_check_config.type = type;
    config.collision_check_config.longest_valid_segment_length = longest_valid_segment_length;
    config.collision_margin_buffer = 0.05;
    return config;
  }

  /**
   * @brief The derivatives of a contact's distance under the cast model, summed over its active
   * links, with respect to the segment start and end
   * @param cast_count The casts the segment was checked with, or 0 for a check at interpolated states
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> expectedDerivatives(const ContactResult& cr, long cast_count) const
  {
    Eigen::VectorXd d0 = Eigen::VectorXd::Zero(q0_.size());
    Eigen::VectorXd d1 = Eigen::VectorXd::Zero(q0_.size());
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (!manip_->isActiveLinkId(cr.link_ids[i]))
        continue;

      // Every check here times each active link
      EXPECT_GE(cr.cc_time[i], 0.0);
      double start = cr.cc_time[i];
      double end = cr.cc_time[i];
      if (cast_count > 0)
      {
        const std::optional<long> cast = matchCast(cr, i, *manip_, q0_, q1_, cast_count);
        if (!cast.has_value())
        {
          ADD_FAILURE() << "link " << i << " carries the end poses of no cast";
          continue;
        }
        start = static_cast<double>(*cast) / static_cast<double>(cast_count);
        end = static_cast<double>(*cast + 1) / static_cast<double>(cast_count);
      }
      const Eigen::Vector3d signed_normal = ((i == 0) ? -1.0 : 1.0) * cr.normal;
      const auto [link_d0, link_d1] = intervalDerivatives(
          *manip_, cr.link_ids[i], cr.nearest_points_local[i], signed_normal, q0_, q1_, start, end, cr.cc_time[i]);
      d0 += link_d0;
      d1 += link_d1;
    }
    return { d0, d1 };
  }

  /** @brief How many active links of the contacts lie well inside their cast, where the cast
   * model's two ends differ from any single state */
  int interiorLinks(const ContactResultVector& contacts, long cast_count) const
  {
    int interior = 0;
    for (const ContactResult& cr : contacts)
    {
      for (std::size_t i = 0; i < 2; ++i)
      {
        if (cast_count <= 0 || !manip_->isActiveLinkId(cr.link_ids[i]))
          continue;
        const std::optional<long> cast = matchCast(cr, i, *manip_, q0_, q1_, cast_count);
        if (!cast.has_value())
          continue;
        const double tau = (cr.cc_time[i] * static_cast<double>(cast_count)) - static_cast<double>(*cast);
        if (tau > 0.05 && tau < 0.95)
          ++interior;
      }
    }
    return interior;
  }

  /**
   * @brief Check every contact's gradient from the evaluator against the cast model
   * @return interiorLinks of the contacts
   */
  int checkEvaluatorGradients(ContinuousCollisionEvaluator& evaluator, long cast_count) const
  {
    EXPECT_EQ(evaluator.getCastCount(q0_, q1_), cast_count);

    trajopt_common::CollisionCacheData data;
    evaluator.calcCollisionData(data, q0_, q1_, false, false, 1000);
    // One link pair with one shape each, so one gradient set whose results follow the contacts' order
    EXPECT_EQ(data.contact_results_map.size(), 1U);
    EXPECT_EQ(data.gradient_results_sets.size(), 1U);
    if (data.contact_results_map.size() != 1 || data.gradient_results_sets.size() != 1)
      return 0;

    const ContactResultVector& contacts = data.contact_results_map.begin()->second;
    const auto& gradients = data.gradient_results_sets.front().results;
    EXPECT_EQ(gradients.size(), contacts.size());

    const Eigen::Index n = q0_.size();
    for (std::size_t k = 0; k < contacts.size() && k < gradients.size(); ++k)
    {
      Eigen::VectorXd actual = Eigen::VectorXd::Zero(2 * n);
      for (std::size_t i = 0; i < 2; ++i)
      {
        const trajopt_common::LinkGradientResults& g0 = gradients[k].gradients[i];
        const trajopt_common::LinkGradientResults& g1 = gradients[k].cc_gradients[i];
        if (g0.has_gradient)
          actual.head(n) += g0.scale * g0.gradient;
        if (g1.has_gradient)
          actual.tail(n) += g1.scale * g1.gradient;
      }
      const auto [d0, d1] = expectedDerivatives(contacts[k], cast_count);
      Eigen::VectorXd expected(2 * n);
      expected << d0, d1;
      expectMatches(actual, expected, k);
    }
    return interiorLinks(contacts, cast_count);
  }
};

// Checked as four casts, each contact's gradient is the derivative of its cast's model: the link's
// jacobians at both ends of the cast it was found in, weighted by where in the cast it lies
TEST_F(ContinuousCollisionIntervalTest, SubdividedEvaluatorGradientBlendsEachContactsCastEnds)  // NOLINT
{
  LVSContinuousCollisionEvaluator evaluator(manip_, env_, makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS, 0.2));
  EXPECT_GT(checkEvaluatorGradients(evaluator, 4), 0) << "no contact lies well inside its cast, so a gradient taken at "
                                                         "a single state would pass too";
}

// CONTINUOUS casts the segment once whatever the longest valid segment length says, and each
// timestep's gradient is the jacobian at that timestep's own state (Schulman et al. 2014, Eq. (20))
TEST_F(ContinuousCollisionIntervalTest, SingleCastEvaluatorGradientTakesEachTimestepAtItsOwnState)  // NOLINT
{
  LVSContinuousCollisionEvaluator evaluator(manip_, env_, makeConfig(CollisionEvaluatorType::CONTINUOUS, 0.2));
  EXPECT_GT(checkEvaluatorGradients(evaluator, 1), 0) << "no contact lies well inside the cast, so a gradient taken at "
                                                         "a single state would pass too";
}

// A check at interpolated states finds each contact at a point in time and linearises it there
TEST_F(ContinuousCollisionIntervalTest, InterpolatedStateEvaluatorGradientIsAtTheContactState)  // NOLINT
{
  LVSDiscreteCollisionEvaluator evaluator(manip_, env_, makeConfig(CollisionEvaluatorType::LVS_DISCRETE, 0.05));
  checkEvaluatorGradients(evaluator, 0);
}

// The constraint's jacobian places each contact in its cast from the evaluator's cast count, so its
// rows are the negated derivatives of the cast model
TEST_F(ContinuousCollisionIntervalTest, ConstraintJacobianBlendsEachContactsCastEnds)  // NOLINT
{
  auto evaluator = std::make_shared<LVSContinuousCollisionEvaluator>(
      manip_, env_, makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS, 0.2));

  const std::vector<Bounds> bounds = toBounds(manip_->getLimits().joint_limits);
  const std::vector<std::string> joint_names = tesseract::common::toNames(manip_->getJointIds());
  std::vector<std::unique_ptr<Node>> nodes;
  nodes.push_back(std::make_unique<Node>("Joint_Position_0"));
  const std::shared_ptr<const Var> var0 = nodes.back()->addVar("position", joint_names, q0_, bounds);
  nodes.push_back(std::make_unique<Node>("Joint_Position_1"));
  const std::shared_ptr<const Var> var1 = nodes.back()->addVar("position", joint_names, q1_, bounds);
  auto variables = std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes));

  Problem nlp(variables);
  auto constraint = std::make_shared<ContinuousCollisionConstraintD>(
      evaluator, std::array<std::shared_ptr<const Var>, 2>{ var0, var1 }, false, false);
  nlp.addConstraintSet(constraint);
  nlp.setVariables(variables->getValues().data());
  const Eigen::MatrixXd jac = constraint->getJacobian().toDense();

  // The same check the constraint ran, for the contacts its rows follow
  trajopt_common::CollisionCacheData data;
  evaluator->calcCollisionData(data, q0_, q1_, false, false, 0);
  ASSERT_EQ(data.contact_results_map.size(), 1U);
  const ContactResultVector& contacts = data.contact_results_map.begin()->second;
  ASSERT_EQ(jac.rows(), static_cast<Eigen::Index>(contacts.size()));

  const Eigen::Index n = q0_.size();
  for (std::size_t k = 0; k < contacts.size(); ++k)
  {
    const auto row = static_cast<Eigen::Index>(k);
    const auto [d0, d1] = expectedDerivatives(contacts[k], 4);
    Eigen::VectorXd expected(2 * n);
    expected << -d0, -d1;
    Eigen::VectorXd actual(2 * n);
    actual << jac.row(row).segment(var0->getIndex(), n).transpose(),
        jac.row(row).segment(var1->getIndex(), n).transpose();
    expectMatches(actual, expected, k);
  }
  EXPECT_GT(interiorLinks(contacts, 4), 0) << "no contact lies well inside its cast, so a gradient taken at a single "
                                              "state would pass too";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
