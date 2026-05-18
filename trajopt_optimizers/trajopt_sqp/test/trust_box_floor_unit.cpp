/**
 * @file trust_box_floor_unit.cpp
 * @brief Unit tests for TrajOptQPProblem trust-box construction near and past variable bounds.
 *
 * Policy: clamp the iterate into its variable bounds [lb, ub], then strict-shrink the trust box
 * [x-Δ, x+Δ] against [lb, ub]. For x inside [lb, ub] this is a no-op and yields the standard
 * trust-region box with |p|_∞ ≤ Δ. For x drifted outside [lb, ub], the clamp keeps the box
 * non-empty (width Δ, flush against the violated bound) so the next QP step pulls the iterate
 * back through the bound.
 *
 * Replaces the earlier PR #472 "slide always" behavior, which kept full 2·Δ width even when the
 * iterate sat on a bound — at the cost of letting the QP step exceed Δ on the open side.
 */
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>

namespace
{
constexpr double kLb = -2.5;
constexpr double kUb = 2.5;
constexpr double kBi = 0.0015;  // trust half-width Δ

std::shared_ptr<trajopt_sqp::TrajOptQPProblem> makeProblem(double x_init, double lb, double ub)
{
  auto node = std::make_unique<trajopt_ifopt::Node>("Joints");
  const std::vector<std::string> names{ "j0" };
  const Eigen::VectorXd values = (Eigen::VectorXd(1) << x_init).finished();
  const std::vector<trajopt_ifopt::Bounds> bounds{ trajopt_ifopt::Bounds(lb, ub) };
  node->addVar("position", names, values, bounds);

  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  nodes.push_back(std::move(node));
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes));

  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);
  qp->setup();
  return qp;
}

// Read the variable trust-box segment. With no constraints, it occupies bounds_lower.head(n_nlp_vars).
struct TrustBox
{
  double lower;
  double upper;
  double width() const { return upper - lower; }
};
TrustBox readBox(const trajopt_sqp::TrajOptQPProblem& qp)
{
  const Eigen::Index n = qp.getNumNLPVars();
  return { qp.getBoundsLower().head(n)[0], qp.getBoundsUpper().head(n)[0] };
}
}  // namespace

// Iterate well inside [lb, ub]: standard centered box [x-Δ, x+Δ], width 2·Δ.
TEST(TrajOptQPTrustBox, InteriorIterateGetsCenteredBox)  // NOLINT
{
  auto qp = makeProblem(0.0, kLb, kUb);
  qp->setBoxSize(Eigen::VectorXd::Constant(1, kBi));
  const auto box = readBox(*qp);

  EXPECT_NEAR(box.lower, 0.0 - kBi, 1e-12);
  EXPECT_NEAR(box.upper, 0.0 + kBi, 1e-12);
  EXPECT_NEAR(box.width(), 2.0 * kBi, 1e-12);
}

// Iterate within Δ of the upper bound: box clamped at ub, lower side asymmetric. Width transitions
// from 2·Δ down to Δ as x rises from (ub-Δ) to ub. Verifying the midpoint case x = ub - Δ/2.
TEST(TrajOptQPTrustBox, IterateNearUpperBoundShrinksAsymmetrically)  // NOLINT
{
  const double x_val = kUb - (kBi / 2.0);
  auto qp = makeProblem(x_val, kLb, kUb);
  qp->setBoxSize(Eigen::VectorXd::Constant(1, kBi));
  const auto box = readBox(*qp);

  EXPECT_NEAR(box.lower, x_val - kBi, 1e-12);
  EXPECT_NEAR(box.upper, kUb, 1e-12);
  EXPECT_NEAR(box.width(), 1.5 * kBi, 1e-12);
  // TR invariant: max(|lower - x|, |upper - x|) = max(Δ, Δ/2) = Δ. Held.
  EXPECT_LE(std::max(std::abs(box.lower - x_val), std::abs(box.upper - x_val)), kBi + 1e-12);
}

// Iterate at the upper bound exactly: box [ub-Δ, ub], width Δ, fully TR-compliant.
TEST(TrajOptQPTrustBox, IterateAtUpperBoundGetsBoxOfWidthBi)  // NOLINT
{
  auto qp = makeProblem(kUb, kLb, kUb);
  qp->setBoxSize(Eigen::VectorXd::Constant(1, kBi));
  const auto box = readBox(*qp);

  EXPECT_NEAR(box.lower, kUb - kBi, 1e-12);
  EXPECT_NEAR(box.upper, kUb, 1e-12);
  EXPECT_NEAR(box.width(), kBi, 1e-12);
}

// PR #472 bug case: iterate past the upper bound by >> Δ. The clamp keeps the box flush against
// the violated bound with width Δ — no inversion, no OSQP error. The QP step will pull x back
// through the bound (step magnitude ≈ |x - ub|, unavoidable for recovery).
TEST(TrajOptQPTrustBox, IterateFarPastUpperBoundStaysAtWidthBi)  // NOLINT
{
  const double x_val = 2.6;  // ub = 2.5, violation = 0.1, Δ = 0.0015
  auto qp = makeProblem(x_val, kLb, kUb);
  qp->setBoxSize(Eigen::VectorXd::Constant(1, kBi));
  const auto box = readBox(*qp);

  EXPECT_NEAR(box.lower, kUb - kBi, 1e-12);
  EXPECT_NEAR(box.upper, kUb, 1e-12);
  EXPECT_NEAR(box.width(), kBi, 1e-12);
  EXPECT_GT(box.width(), 0.0);  // explicit non-inversion guard
}

// Mirror: iterate past the lower bound. Box flush at lb with width Δ.
TEST(TrajOptQPTrustBox, IterateFarPastLowerBoundStaysAtWidthBi)  // NOLINT
{
  auto qp = makeProblem(-2.6, kLb, kUb);
  qp->setBoxSize(Eigen::VectorXd::Constant(1, kBi));
  const auto box = readBox(*qp);

  EXPECT_NEAR(box.lower, kLb, 1e-12);
  EXPECT_NEAR(box.upper, kLb + kBi, 1e-12);
  EXPECT_NEAR(box.width(), kBi, 1e-12);
}

// Variable range narrower than the trust radius: box is clamped to the full natural range
// [lb, ub], which is the best we can do.
TEST(TrajOptQPTrustBox, BoundsNarrowerThanBiClampToRange)  // NOLINT
{
  const double narrow_lb = 0.0;
  const double narrow_ub = 0.1;
  const double bi = 0.5;  // Δ wider than the variable's allowed range
  auto qp = makeProblem(0.05, narrow_lb, narrow_ub);
  qp->setBoxSize(Eigen::VectorXd::Constant(1, bi));
  const auto box = readBox(*qp);

  EXPECT_NEAR(box.lower, narrow_lb, 1e-12);
  EXPECT_NEAR(box.upper, narrow_ub, 1e-12);
}
