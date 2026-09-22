#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <tesseract/common/resource_locator.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/logging.hpp>

using namespace trajopt;
using namespace tesseract::collision;
using namespace tesseract::environment;
using namespace tesseract::kinematics;

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

}  // namespace

// A right-arm PR2 near a table: a chain with genuine rotational joints, so a link's Jacobian
// varies with configuration. A purely prismatic manipulator's Jacobian is constant everywhere,
// which would make an expression built at the wrong linearisation state numerically
// indistinguishable from one built at the right one, and this test would prove nothing.
class DiscreteExpressionFrameTest : public testing::Test
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>();

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");

    const tesseract::common::ResourceLocator::Ptr locator =
        std::make_shared<tesseract::common::GeneralResourceLocator>();
    ASSERT_TRUE(env_->init(urdf_file, srdf_file, locator));

    trajopt_common::gLogLevel = trajopt_common::LevelError;
  }

  /** @brief The coefficients a correct expression must carry for one contact.
   *
   * The linearised quantity is the contact distance at the state the contact occurs at. Its
   * derivative with respect to the segment endpoints is that gradient weighted by (1 - cc_time)
   * for the start block and cc_time for the end block. A link the checker gave no interval keeps
   * full weight at the state it is linearised about. The Jacobian is taken numerically so the
   * oracle shares no code with the implementation.
   *
   * This mirrors the implementation's branch chain and scale expression line for line, so it does
   * not independently check branch *precedence* - a wrong order would be reproduced identically
   * here and this test would still pass. cast_gradient_frame_unit.cpp closes that gap by pinning
   * specific expected states by construction. */
  static Eigen::VectorXd referenceCoeffs(const ContactResult& cr,
                                         const JointGroup& manip,
                                         const Eigen::VectorXd& q0,
                                         const Eigen::VectorXd& q1)
  {
    const auto n = static_cast<Eigen::Index>(manip.numJoints());
    Eigen::VectorXd expected = Eigen::VectorXd::Zero(2 * n);

    for (std::size_t i = 0; i < 2; ++i)
    {
      if (!manip.isActiveLinkId(cr.link_ids[i]))
        continue;

      const bool has_interval = cr.cc_time[i] >= 0.0;

      for (int block = 0; block < 2; ++block)
      {
        const bool is_timestep1 = (block == 1);
        Eigen::VectorXd q_t;
        if (cr.cc_type[i] == ContinuousCollisionType::CCType_Time0)
          q_t = q0;
        else if (cr.cc_type[i] == ContinuousCollisionType::CCType_Time1)
          q_t = q1;
        else if (has_interval)
          q_t = q0 + (q1 - q0) * cr.cc_time[i];
        else
          q_t = is_timestep1 ? q1 : q0;

        double scale = 1.0;
        if (has_interval)
          scale = is_timestep1 ? cr.cc_time[i] : 1.0 - cr.cc_time[i];

        Eigen::MatrixXd num_jac(6, manip.numJoints());
        numericalJacobian(
            num_jac, Eigen::Isometry3d::Identity(), manip, q_t, cr.link_ids[i], cr.nearest_points_local[i]);
        const Eigen::VectorXd g = ((i == 0) ? -1.0 : 1.0) * cr.normal.transpose() * num_jac.topRows(3);

        if (is_timestep1)
          expected.tail(n) += scale * g;
        else
          expected.head(n) += scale * g;
      }
    }
    return expected;
  }
};

// A subdivided segment reports contacts from sub-intervals between its endpoints. The expression
// built for such a contact must linearise at the state the contact occurs at; linearising at an
// endpoint gives a search direction for a configuration the contact does not happen in.
TEST_F(DiscreteExpressionFrameTest, SubdividedContactExpressionUsesInterpolatedState)  // NOLINT
{
  auto manip = env_->getJointGroup("right_arm");
  const auto n = static_cast<Eigen::Index>(manip->numJoints());
  const auto un = static_cast<std::size_t>(n);

  trajopt_common::TrajOptCollisionConfig config(0.02, 1);
  config.collision_check_config.type = CollisionEvaluatorType::LVS_DISCRETE;
  config.collision_check_config.longest_valid_segment_length = 0.05;
  config.collision_margin_buffer = 0.05;

  DiscreteCollisionEvaluator evaluator(
      manip, env_, config, makeVars(0, un), makeVars(un, un), CollisionExpressionEvaluatorType::START_FREE_END_FREE);

  Eigen::VectorXd q0(7);
  q0 << -1.1, 1.2, -1.5, -1.4, -1.1, -1.3, 0.2;
  Eigen::VectorXd q1(7);
  q1 << -0.3, 0.7, -0.9, -0.8, -0.4, -0.7, 0.9;
  DblVec x;
  for (Eigen::Index i = 0; i < n; ++i)
    x.push_back(q0(i));
  for (Eigen::Index i = 0; i < n; ++i)
    x.push_back(q1(i));

  sco::AffExprVector exprs;
  std::vector<double> exprs_margin;
  std::vector<double> exprs_coeff;
  evaluator.CalcDistExpressions(x, exprs, exprs_margin, exprs_coeff);

  const ContactResultVectorConstPtr contacts = evaluator.GetContactResultVectorCached(x);
  ASSERT_TRUE(contacts != nullptr);
  ASSERT_FALSE(contacts->empty());

  // Every contact yields an expression: setActiveCollisionObjects(manip->getActiveLinkIds()) means
  // every reported pair has at least one active link, so none are ever filtered out here.
  ASSERT_EQ(exprs.size(), contacts->size());

  int between = 0;
  for (std::size_t c = 0; c < contacts->size(); ++c)
  {
    const ContactResult& cr = (*contacts)[c].get();
    for (std::size_t i = 0; i < 2; ++i)
      if (manip->isActiveLinkId(cr.link_ids[i]) && cr.cc_type[i] == ContinuousCollisionType::CCType_Between)
        ++between;

    const Eigen::VectorXd expected = referenceCoeffs(cr, *manip, q0, q1);
    // isApprox is a relative test and is satisfied by any pair of near-zero vectors, so a contact
    // whose normal lies near the null space of the witness point's Jacobian would pass regardless
    // of implementation without this floor.
    EXPECT_GT(expected.norm(), 1e-2) << "contact " << c
                                     << " produces no usable gradient, so the comparison below would "
                                        "hold for any implementation";
    // Converted the same way the QP builder converts it, so the comparison is against the row the
    // solver would actually see.
    Eigen::SparseVector<double> actual_sparse;
    sco::exprToEigen(exprs[c], actual_sparse, 2 * n);
    const Eigen::VectorXd actual = actual_sparse;

    EXPECT_TRUE(actual.isApprox(expected, 1e-4))
        << "contact " << c << "\ngot      " << actual.transpose() << "\nexpected " << expected.transpose();
  }

  ASSERT_GT(between, 0) << "no contact was typed Between, so every comparison above would hold for a "
                           "gradient taken at either endpoint and this test proves nothing";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
