/**
 * @file qp_problem_merit_unit.cpp
 * @brief Pins how the QP problems weight constraint and cost rows in the QP, in the convex model, and in
 *        the exact merit the trust-region solver compares them against.
 */
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

namespace
{
using CoeffFn = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

/** @brief Select the dynamic-size constructor of LinearTestSet. */
struct Dynamic
{
};

/**
 * @brief Constraint set whose value is its variable block, with an identity Jacobian over that block.
 * @details Every row shares one bound. update() recomputes the per-row coefficients from the current
 * variable values through @p coeff_fn, as the collision constraints recompute theirs.
 */
class LinearTestSet : public trajopt_ifopt::ConstraintSet
{
public:
  LinearTestSet(std::shared_ptr<const trajopt_ifopt::Var> var,
                std::string name,
                trajopt_ifopt::Bounds bound,
                CoeffFn coeff_fn)
    : ConstraintSet(std::move(name), static_cast<int>(var->size()))
    , var_(std::move(var))
    , bound_(bound)
    , coeff_fn_(std::move(coeff_fn))
    , coeffs_(coeff_fn_(var_->value()))
  {
    non_zeros_ = var_->size();
  }

  LinearTestSet(Dynamic,
                std::shared_ptr<const trajopt_ifopt::Var> var,
                std::string name,
                trajopt_ifopt::Bounds bound,
                CoeffFn coeff_fn)
    : ConstraintSet(std::move(name), true)
    , var_(std::move(var))
    , bound_(bound)
    , coeff_fn_(std::move(coeff_fn))
    , coeffs_(coeff_fn_(var_->value()))
  {
  }

  int update() override
  {
    if (isDynamic())
    {
      rows_ = static_cast<int>(var_->size());
      non_zeros_ = var_->size();
    }
    coeffs_ = coeff_fn_(var_->value());
    return rows_;
  }

  Eigen::VectorXd getValues() const override { return var_->value(); }
  Eigen::VectorXd getCoefficients() const override { return coeffs_; }
  std::vector<trajopt_ifopt::Bounds> getBounds() const override
  {
    return std::vector<trajopt_ifopt::Bounds>(static_cast<std::size_t>(var_->size()), bound_);
  }

  trajopt_ifopt::Jacobian getJacobian() const override
  {
    trajopt_ifopt::Jacobian jac(static_cast<int>(var_->size()), variables_->getRows());
    jac.reserve(var_->size());
    for (Eigen::Index j = 0; j < var_->size(); ++j)
    {
      jac.startVec(static_cast<int>(j));
      jac.insertBack(static_cast<int>(j), var_->getIndex() + j) = 1.0;
    }
    jac.finalize();
    return jac;
  }

private:
  std::shared_ptr<const trajopt_ifopt::Var> var_;
  trajopt_ifopt::Bounds bound_;
  CoeffFn coeff_fn_;
  Eigen::VectorXd coeffs_;
};

/** @brief Dynamic constraint set that currently has no rows, as a collision set out of contact does. */
class EmptyTestSet : public trajopt_ifopt::ConstraintSet
{
public:
  explicit EmptyTestSet(std::string name) : ConstraintSet(std::move(name), true) {}

  int update() override
  {
    rows_ = 0;
    non_zeros_ = 0;
    return rows_;
  }

  Eigen::VectorXd getValues() const override { return {}; }
  Eigen::VectorXd getCoefficients() const override { return {}; }
  std::vector<trajopt_ifopt::Bounds> getBounds() const override { return {}; }
  trajopt_ifopt::Jacobian getJacobian() const override { return { 0, variables_->getRows() }; }
};

struct TestVariables
{
  std::shared_ptr<trajopt_ifopt::NodesVariables> variables;
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
};

/** @brief One node, holding one unbounded variable block, per entry of @p starts. */
TestVariables makeVariables(const std::vector<Eigen::VectorXd>& starts)
{
  TestVariables t;
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  for (std::size_t k = 0; k < starts.size(); ++k)
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("node" + std::to_string(k));
    const auto n = static_cast<std::size_t>(starts[k].size());
    t.vars.push_back(node->addVar("position",
                                  std::vector<std::string>(n, "j"),
                                  starts[k],
                                  std::vector<trajopt_ifopt::Bounds>(n, trajopt_ifopt::NoBound)));
    nodes.push_back(std::move(node));
  }
  t.variables = std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes));
  return t;
}

using trajopt_common::toVectorXd;

CoeffFn constantWeights(Eigen::VectorXd weights)
{
  return [weights = std::move(weights)](const Eigen::VectorXd& /*x*/) { return weights; };
}

/** @brief Weights 1 + |x_i|, so they change whenever the iterate moves. */
Eigen::VectorXd growingWeights(const Eigen::VectorXd& x) { return (1.0 + x.array().abs()).matrix(); }

void expectVectorNear(const Eigen::Ref<const Eigen::VectorXd>& actual,
                      const Eigen::VectorXd& expected,
                      double tol = 1e-12)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (Eigen::Index i = 0; i < actual.size(); ++i)
    EXPECT_NEAR(actual(i), expected(i), tol) << "at index " << i;
}
}  // namespace

// A merit set with no rows still owns its merit-coefficient slot, so each later set is penalized by its own.
TEST(QPProblemMerit, EmptySetKeepsItsMeritCoefficientSlot)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.2 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addConstraintSet(std::make_shared<EmptyTestSet>("empty"));
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      Dynamic{}, t.vars[0], "linear", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 2.0, 3.0 }))));
  qp->setup();
  qp->setConstraintMeritCoeff(toVectorXd({ 10.0, 100.0 }));
  qp->convexify();

  // Each equality row owns a (+, -) slack pair, penalized by its set's merit coefficient times the row weight.
  const Eigen::VectorXd& gradient = qp->getGradient();
  ASSERT_EQ(gradient.size(), 6);
  expectVectorNear(gradient.tail(4), toVectorXd({ 200.0, 200.0, 300.0, 300.0 }));
}

// Weights that a fixed-size set rewrites in update() reach the QP at the next convexify(), for merit
// constraints and for penalty costs alike.
TEST(QPProblemMerit, ConvexifyRefreshesFixedSizeSetWeights)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.2 }), toVectorXd({ 0.3, 0.6 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(std::make_shared<LinearTestSet>(t.vars[1], "hinge", trajopt_ifopt::BoundSmallerZero, growingWeights),
                 trajopt_sqp::CostPenaltyType::kHinge);
  qp->addConstraintSet(
      std::make_shared<LinearTestSet>(t.vars[0], "linear", trajopt_ifopt::Bounds(0.0, 0.0), growingWeights));
  qp->setup();
  qp->convexify();
  // Slacks: one per hinge row, charged its weight, then a (+, -) pair per equality row, charged the default
  // merit coefficient 10 times its weight. Weights 1 + |x|: hinge (1.3, 1.6), constraint (1.5, 1.2).
  expectVectorNear(qp->getGradient().tail(6), toVectorXd({ 1.3, 1.6, 15.0, 15.0, 12.0, 12.0 }));

  const Eigen::VectorXd x_new = toVectorXd({ 1.0, 0.4, -0.5, 0.2 });
  qp->setVariables(x_new.data());
  qp->convexify();
  // Weights at x_new: hinge (1.5, 1.2), constraint (2.0, 1.4).
  expectVectorNear(qp->getGradient().tail(6), toVectorXd({ 1.5, 1.2, 20.0, 20.0, 14.0, 14.0 }));
}

// Hinge rows are violated above 0; each row's violation is scaled by its weight.
TEST(QPProblemMerit, ExactHingeCostIsWeighted)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(
      std::make_shared<LinearTestSet>(
          t.vars[0], "hinge", trajopt_ifopt::BoundSmallerZero, constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))),
      trajopt_sqp::CostPenaltyType::kHinge);
  qp->setup();
  const Eigen::VectorXd costs = qp->getExactCosts();
  ASSERT_EQ(costs.size(), 1);
  EXPECT_NEAR(costs(0), 4.2, 1e-12);  // 2 * 0.5 + 3 * 0 + 4 * 0.8
}

// A cost row of weight 0 is disabled, so an infinite violation on it adds nothing instead of NaN.
TEST(QPProblemMerit, ZeroWeightCostRowWithInfiniteViolationAddsNothing)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ std::numeric_limits<double>::infinity(), -0.3, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(
      std::make_shared<LinearTestSet>(
          t.vars[0], "hinge", trajopt_ifopt::BoundSmallerZero, constantWeights(toVectorXd({ 0.0, 3.0, 4.0 }))),
      trajopt_sqp::CostPenaltyType::kHinge);
  qp->addCostSet(
      std::make_shared<LinearTestSet>(
          t.vars[0], "squared", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 0.0, 3.0, 4.0 }))),
      trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  const Eigen::VectorXd costs = qp->getExactCosts();
  ASSERT_EQ(costs.size(), 2);
  EXPECT_NEAR(costs(0), 2.83, 1e-12);  // 3 * 0.09 + 4 * 0.64
  EXPECT_NEAR(costs(1), 3.2, 1e-12);   // 4 * 0.8
}

// The convex hinge cost is the weighted violation of the linearized rows, whatever the slack values.
TEST(QPProblemMerit, ConvexHingeCostIgnoresSlackValues)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(
      std::make_shared<LinearTestSet>(
          t.vars[0], "hinge", trajopt_ifopt::BoundSmallerZero, constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))),
      trajopt_sqp::CostPenaltyType::kHinge);
  qp->setup();
  qp->convexify();
  ASSERT_EQ(qp->getNumQPVars(), 6);  // three NLP variables, one slack per hinge row

  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(6);
  qp_vals.head(3) = toVectorXd({ 0.5, -0.3, 0.8 });
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 4.2, 1e-12);

  // Slacks at the values a QP solution gives them, absorbing every violation.
  qp_vals.tail(3) = toVectorXd({ 0.5, 0.0, 0.8 });
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 4.2, 1e-12);

  // Away from the linearization point.
  qp_vals.head(3) = toVectorXd({ 0.2, 0.1, 1.0 });
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 4.7, 1e-12);  // 2 * 0.2 + 3 * 0.1 + 4 * 1.0
}

// Absolute-cost rows are penalized in both directions, weighted, and slack-free in the model.
TEST(QPProblemMerit, AbsoluteCostIsWeightedAndIgnoresSlackValues)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "absolute", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 2.0, 3.0 }))),
                 trajopt_sqp::CostPenaltyType::kAbsolute);
  qp->setup();
  qp->convexify();
  EXPECT_NEAR(qp->getExactCosts()(0), 1.9, 1e-12);  // 2 * 0.5 + 3 * 0.3

  ASSERT_EQ(qp->getNumQPVars(), 6);  // two NLP variables, a (+, -) slack pair per row
  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(6);
  qp_vals.head(2) = toVectorXd({ 0.5, -0.3 });
  qp_vals.tail(4) = toVectorXd({ 0.0, 0.5, 0.3, 0.0 });  // the slacks that zero each row at a QP solution
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 1.9, 1e-12);
}

// Every residual here is linear in x, so the convex model equals the exact merit everywhere and the
// trust-region ratio of any step is 1.
TEST(QPProblemMerit, LinearProblemStepHasUnitImproveRatio)  // NOLINT
{
  const TestVariables t =
      makeVariables({ toVectorXd({ 0.5, 0.8 }), toVectorXd({ 0.4, -0.6 }), toVectorXd({ 0.3, -0.1 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "hinge", trajopt_ifopt::BoundSmallerZero, constantWeights(toVectorXd({ 2.0, 3.0 }))),
                 trajopt_sqp::CostPenaltyType::kHinge);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[1], "equality", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 4.0, 5.0 }))));
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[2], "squared", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 1.0, 1.0 }))),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  const Eigen::VectorXd x0 = qp->getVariableValues();

  trajopt_sqp::TrustRegionSQPSolver solver(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
  solver.init(qp);
  solver.stepSQPSolver();

  const trajopt_sqp::SQPResults& results = solver.getResults();
  EXPECT_GT(results.approx_merit_improve, 0.0);
  EXPECT_NEAR(results.merit_improve_ratio, 1.0, 1e-9);
  // The step is accepted, and the best iterate carries the weighted violations of its own point.
  ASSERT_FALSE(results.best_var_vals.isApprox(x0));
  expectVectorNear(results.best_constraint_violations.weighted, qp->getExactConstraintViolations().weighted);
}

// One entry per merit set: raw sums the row violations, weighted sums each times its row weight.
TEST(QPProblemMerit, ExactViolationsAreSummedPerSetAndWeighted)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3, 0.8 }), toVectorXd({ 0.1, -0.4 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[0], "a", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))));
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[1], "b", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 5.0, 6.0 }))));
  qp->setup();

  const trajopt_sqp::ConstraintViolations cv = qp->getExactConstraintViolations();
  expectVectorNear(cv.raw, toVectorXd({ 1.6, 0.5 }));
  expectVectorNear(cv.weighted, toVectorXd({ 5.1, 2.9 }));  // 0.5*2 + 0.3*3 + 0.8*4, 0.1*5 + 0.4*6
}

// After convexify() at a point, the convex model reproduces the exact violations there, including weights
// that follow the iterate.
TEST(QPProblemMerit, ConvexViolationsMatchExactAtConvexifyPoint)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3, 0.8 }), toVectorXd({ 0.1, -0.4 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addConstraintSet(
      std::make_shared<LinearTestSet>(t.vars[0], "a", trajopt_ifopt::Bounds(0.0, 0.0), growingWeights));
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[1], "b", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 5.0, 6.0 }))));
  qp->setup();
  qp->convexify();

  const Eigen::VectorXd x_new = toVectorXd({ 1.0, 0.2, -0.5, 0.1, -0.4 });
  qp->setVariables(x_new.data());
  qp->convexify();

  const trajopt_sqp::ConstraintViolations exact = qp->getExactConstraintViolations();
  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(qp->getNumQPVars());
  qp_vals.head(5) = x_new;
  const trajopt_sqp::ConstraintViolations convex = qp->evaluateConvexConstraintViolations(qp_vals);

  // Set a at x_new: weights (2.0, 1.2, 1.5), so 1.0*2.0 + 0.2*1.2 + 0.5*1.5.
  expectVectorNear(exact.raw, toVectorXd({ 1.7, 0.5 }));
  expectVectorNear(exact.weighted, toVectorXd({ 2.99, 2.9 }));
  expectVectorNear(convex.raw, exact.raw);
  expectVectorNear(convex.weighted, exact.weighted);
}

// A row of weight 0 is disabled: it leaves the feasibility metric and contributes nothing to the merit.
TEST(QPProblemMerit, ZeroWeightRowLeavesTheFeasibilityMetric)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[0], "a", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 0.0, 3.0, 4.0 }))));
  qp->setup();
  qp->convexify();

  const trajopt_sqp::ConstraintViolations exact = qp->getExactConstraintViolations();
  expectVectorNear(exact.raw, toVectorXd({ 1.1 }));
  expectVectorNear(exact.weighted, toVectorXd({ 4.1 }));

  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(qp->getNumQPVars());
  qp_vals.head(3) = toVectorXd({ 0.5, -0.3, 0.8 });
  const trajopt_sqp::ConstraintViolations convex = qp->evaluateConvexConstraintViolations(qp_vals);
  expectVectorNear(convex.raw, toVectorXd({ 1.1 }));
  expectVectorNear(convex.weighted, toVectorXd({ 4.1 }));
}

// A disabled row's violation is excluded before summing, not multiplied by its zero weight, so an infinite
// violation on that row cannot turn the feasibility metric or the merit into NaN.
TEST(QPProblemMerit, ZeroWeightRowWithInfiniteViolationStaysFinite)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ std::numeric_limits<double>::infinity(), -0.3, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[0], "a", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 0.0, 3.0, 4.0 }))));
  qp->setup();

  const trajopt_sqp::ConstraintViolations exact = qp->getExactConstraintViolations();
  expectVectorNear(exact.raw, toVectorXd({ 1.1 }));
  expectVectorNear(exact.weighted, toVectorXd({ 4.1 }));
}

// IfoptQPProblem applies no per-row constraint weights, in the QP or the merit, so both views are equal.
TEST(QPProblemMerit, IfoptViolationsAreUnweighted)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::IfoptQPProblem>(t.variables);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[0], "a", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))));
  qp->setup();
  qp->convexify();

  const trajopt_sqp::ConstraintViolations exact = qp->getExactConstraintViolations();
  expectVectorNear(exact.raw, toVectorXd({ 0.5, 0.3, 0.8 }));
  expectVectorNear(exact.weighted, exact.raw);

  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(qp->getNumQPVars());
  qp_vals.head(3) = toVectorXd({ 0.5, -0.3, 0.8 });
  const trajopt_sqp::ConstraintViolations convex = qp->evaluateConvexConstraintViolations(qp_vals);
  expectVectorNear(convex.raw, exact.raw);
  expectVectorNear(convex.weighted, exact.raw);
}

// The solver's merit charges each set's merit coefficient against its weighted violation, and keeps the raw
// violation for the feasibility test.
TEST(QPProblemMerit, SeedMeritWeightsConstraintViolations)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.3, 0.8 }), toVectorXd({ 0.5, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[0], "a", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))));
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[1], "hinge", trajopt_ifopt::BoundSmallerZero, constantWeights(toVectorXd({ 2.0, 3.0 }))),
                 trajopt_sqp::CostPenaltyType::kHinge);
  qp->setup();

  trajopt_sqp::TrustRegionSQPSolver solver(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
  solver.init(qp);

  const trajopt_sqp::SQPResults& results = solver.getResults();
  expectVectorNear(results.best_constraint_violations.raw, toVectorXd({ 1.6 }));
  expectVectorNear(results.best_constraint_violations.weighted, toVectorXd({ 5.1 }));
  // Hinge cost 2*0.5 + 3*0.8 = 3.4, plus the initial merit coefficient times the weighted violation.
  EXPECT_NEAR(results.best_exact_merit, 3.4 + (solver.params.initial_merit_error_coeff * 5.1), 1e-12);
}
