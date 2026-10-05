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
 * @details Rows are bounded one by one, or all by one shared bound. update() recomputes the per-row
 * coefficients from the current variable values through @p coeff_fn, as the collision constraints recompute
 * theirs.
 */
class LinearTestSet : public trajopt_ifopt::ConstraintSet
{
public:
  LinearTestSet(std::shared_ptr<const trajopt_ifopt::Var> var,
                std::string name,
                std::vector<trajopt_ifopt::Bounds> bounds,
                CoeffFn coeff_fn)
    : ConstraintSet(std::move(name), static_cast<int>(var->size()))
    , var_(std::move(var))
    , bounds_(std::move(bounds))
    , coeff_fn_(std::move(coeff_fn))
    , coeffs_(coeff_fn_(var_->value()))
  {
    non_zeros_ = var_->size();
  }

  LinearTestSet(const std::shared_ptr<const trajopt_ifopt::Var>& var,
                std::string name,
                trajopt_ifopt::Bounds bound,
                CoeffFn coeff_fn)
    : LinearTestSet(var,
                    std::move(name),
                    std::vector<trajopt_ifopt::Bounds>(static_cast<std::size_t>(var->size()), bound),
                    std::move(coeff_fn))
  {
  }

  LinearTestSet(Dynamic,
                std::shared_ptr<const trajopt_ifopt::Var> var,
                std::string name,
                trajopt_ifopt::Bounds bound,
                CoeffFn coeff_fn)
    : ConstraintSet(std::move(name), true)
    , var_(std::move(var))
    , bounds_(static_cast<std::size_t>(var_->size()), bound)
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
  std::vector<trajopt_ifopt::Bounds> getBounds() const override { return bounds_; }

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
  std::vector<trajopt_ifopt::Bounds> bounds_;
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

/**
 * @brief Constraint set with value M * x over its variable block and Jacobian M.
 * @details Every entry of M is stored in the Jacobian, whatever its magnitude. Every row shares one bound.
 */
class MatrixTestSet : public trajopt_ifopt::ConstraintSet
{
public:
  MatrixTestSet(std::shared_ptr<const trajopt_ifopt::Var> var,
                std::string name,
                Eigen::MatrixXd matrix,
                trajopt_ifopt::Bounds bound,
                Eigen::VectorXd weights)
    : ConstraintSet(std::move(name), static_cast<int>(matrix.rows()))
    , var_(std::move(var))
    , matrix_(std::move(matrix))
    , bound_(bound)
    , weights_(std::move(weights))
  {
    non_zeros_ = matrix_.size();
  }

  int update() override { return rows_; }

  Eigen::VectorXd getValues() const override { return matrix_ * var_->value(); }
  Eigen::VectorXd getCoefficients() const override { return weights_; }
  std::vector<trajopt_ifopt::Bounds> getBounds() const override
  {
    std::vector<trajopt_ifopt::Bounds> bounds(static_cast<std::size_t>(matrix_.rows()), bound_);
    return bounds;
  }

  trajopt_ifopt::Jacobian getJacobian() const override
  {
    trajopt_ifopt::Jacobian jac(static_cast<int>(matrix_.rows()), variables_->getRows());
    jac.reserve(matrix_.size());
    for (Eigen::Index r = 0; r < matrix_.rows(); ++r)
    {
      jac.startVec(static_cast<int>(r));
      for (Eigen::Index c = 0; c < matrix_.cols(); ++c)
        jac.insertBack(static_cast<int>(r), var_->getIndex() + c) = matrix_(r, c);
    }
    jac.finalize();
    return jac;
  }

private:
  std::shared_ptr<const trajopt_ifopt::Var> var_;
  Eigen::MatrixXd matrix_;
  trajopt_ifopt::Bounds bound_;
  Eigen::VectorXd weights_;
};

/** @brief The rows of an AffineTestSet: jac * x + offset over its variable block, one bound and one weight per row. */
struct AffineRows
{
  Eigen::MatrixXd jac;
  Eigen::VectorXd offset;
  std::vector<trajopt_ifopt::Bounds> bounds;
  Eigen::VectorXd weights;
};

/**
 * @brief Dynamic constraint set that reports the rows @p rows holds at each update(), as a collision set's rows
 * follow its contacts.
 * @details A zero of AffineRows::jac has no entry in the Jacobian.
 */
class AffineTestSet : public trajopt_ifopt::ConstraintSet
{
public:
  AffineTestSet(std::shared_ptr<const trajopt_ifopt::Var> var, std::string name, std::shared_ptr<const AffineRows> rows)
    : ConstraintSet(std::move(name), true), var_(std::move(var)), source_(std::move(rows))
  {
    AffineTestSet::update();
  }

  int update() override
  {
    current_ = *source_;
    rows_ = static_cast<int>(current_.offset.size());
    non_zeros_ = (current_.jac.array() != 0.0).count();
    return rows_;
  }

  Eigen::VectorXd getValues() const override { return current_.jac * var_->value() + current_.offset; }
  Eigen::VectorXd getCoefficients() const override { return current_.weights; }
  std::vector<trajopt_ifopt::Bounds> getBounds() const override { return current_.bounds; }

  trajopt_ifopt::Jacobian getJacobian() const override
  {
    trajopt_ifopt::Jacobian jac(rows_, variables_->getRows());
    for (Eigen::Index r = 0; r < current_.jac.rows(); ++r)
    {
      for (Eigen::Index c = 0; c < current_.jac.cols(); ++c)
      {
        if (current_.jac(r, c) != 0.0)
          jac.insert(r, var_->getIndex() + c) = current_.jac(r, c);
      }
    }
    jac.makeCompressed();
    return jac;
  }

private:
  std::shared_ptr<const trajopt_ifopt::Var> var_;
  std::shared_ptr<const AffineRows> source_;
  AffineRows current_;
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

const double kInf = std::numeric_limits<double>::infinity();

/** @brief An equality row at 0.1, a row bounded above by 0.2 and a row bounded below by 0.1. */
std::vector<trajopt_ifopt::Bounds> mixedBounds()
{
  return { trajopt_ifopt::Bounds(0.1, 0.1), trajopt_ifopt::Bounds(-kInf, 0.2), trajopt_ifopt::Bounds(0.1, kInf) };
}

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

/** @brief Expect the Hessian of @p qp to hold @p diagonal on its diagonal and nothing else. */
void expectDiagonalHessian(const trajopt_sqp::TrajOptQPProblem& qp, const Eigen::VectorXd& diagonal)
{
  const Eigen::MatrixXd hessian = qp.getHessian().toDense();
  const Eigen::MatrixXd expected = diagonal.asDiagonal();
  EXPECT_TRUE(hessian.isApprox(expected)) << hessian;
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

// A linear-penalty cost charges each row by its own bound: both ways for an equality row, one way for a
// one-sided row. Absolute and hinge name the same penalty.
TEST(QPProblemMerit, LinearCostMixesEqualityAndOneSidedRows)  // NOLINT
{
  for (const auto penalty : { trajopt_sqp::CostPenaltyType::kAbsolute, trajopt_sqp::CostPenaltyType::kHinge })
  {
    const TestVariables t = makeVariables({ toVectorXd({ 0.5, 0.8, -0.3 }) });
    auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
    const std::vector<trajopt_ifopt::Bounds> bounds{ trajopt_ifopt::Bounds(0.0, 0.0),
                                                     trajopt_ifopt::Bounds(-kInf, 0.2),
                                                     trajopt_ifopt::Bounds(0.1, kInf) };
    qp->addCostSet(
        std::make_shared<LinearTestSet>(t.vars[0], "mixed", bounds, constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))),
        penalty);
    qp->setup();
    qp->convexify();
    EXPECT_NEAR(qp->getExactCosts()(0), 4.4, 1e-12);  // 2 * 0.5 + 3 * (0.8 - 0.2) + 4 * (0.1 + 0.3)

    // A (+, -) slack pair for the equality row, then one slack per one-sided row, each charged its row weight.
    ASSERT_EQ(qp->getNumQPVars(), 7);
    expectVectorNear(qp->getGradient().tail(4), toVectorXd({ 2.0, 2.0, 3.0, 4.0 }));
    Eigen::MatrixXd expected_slack_block(3, 4);
    expected_slack_block << 1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
    const Eigen::MatrixXd slack_block = qp->getConstraintMatrix().toDense().block(0, 3, 3, 4);
    EXPECT_TRUE(slack_block.isApprox(expected_slack_block)) << slack_block;

    Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(7);
    qp_vals.head(3) = toVectorXd({ 0.5, 0.8, -0.3 });
    EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 4.4, 1e-12);

    // Inside both one-sided bounds only the equality row is charged.
    qp_vals.head(3) = toVectorXd({ -0.1, 0.1, 0.3 });
    EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 0.2, 1e-12);
  }
}

// Minimizing |x0| + 3 max(0, x1 - 0.2) + (x0 - 1)^2 + (x1 - 1)^2: x0 stops where the absolute row's slope
// balances the pull, x1 stops on its bound.
TEST(QPProblemMerit, LinearMixedRowCostSolvesToItsOptimum)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.0, 0.0 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  const std::vector<trajopt_ifopt::Bounds> bounds{ trajopt_ifopt::Bounds(0.0, 0.0), trajopt_ifopt::Bounds(-kInf, 0.2) };
  qp->addCostSet(std::make_shared<LinearTestSet>(t.vars[0], "mixed", bounds, constantWeights(toVectorXd({ 1.0, 3.0 }))),
                 trajopt_sqp::CostPenaltyType::kAbsolute);
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "pull", trajopt_ifopt::Bounds(1.0, 1.0), constantWeights(toVectorXd({ 1.0, 1.0 }))),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();

  trajopt_sqp::TrustRegionSQPSolver solver(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
  solver.solve(qp);
  expectVectorNear(qp->getVariableValues(), toVectorXd({ 0.5, 0.2 }), 1e-3);
}

// A row bounded on both sides, or on neither, has no slack model, so every penalty type refuses it.
TEST(QPProblemMerit, CostRowsBoundedOnBothOrNoSidesAreRejected)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, 0.8 }) });
  for (const auto penalty : { trajopt_sqp::CostPenaltyType::kSquared,
                              trajopt_sqp::CostPenaltyType::kAbsolute,
                              trajopt_sqp::CostPenaltyType::kHinge })
  {
    for (const auto& unsupported : { trajopt_ifopt::Bounds(-1.0, 1.0), trajopt_ifopt::NoBound })
    {
      auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
      const std::vector<trajopt_ifopt::Bounds> bounds{ trajopt_ifopt::Bounds(0.0, 0.0), unsupported };
      EXPECT_THROW(qp->addCostSet(std::make_shared<LinearTestSet>(
                                      t.vars[0], "unsupported", bounds, constantWeights(toVectorXd({ 1.0, 1.0 }))),
                                  penalty),
                   std::runtime_error);
    }
  }
}

// A dynamic set that has no rows when it is added is refused once it reports a row bounded on both sides.
TEST(QPProblemMerit, DynamicCostRowBoundedOnBothSidesIsRejectedWhenReported)  // NOLINT
{
  for (const auto penalty : { trajopt_sqp::CostPenaltyType::kSquared, trajopt_sqp::CostPenaltyType::kHinge })
  {
    const TestVariables t = makeVariables({ toVectorXd({ 0.5 }) });
    auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
    auto rows = std::make_shared<AffineRows>();
    rows->jac = Eigen::MatrixXd::Zero(0, 1);
    qp->addCostSet(std::make_shared<AffineTestSet>(t.vars[0], "late", rows), penalty);
    qp->setup();
    qp->convexify();

    *rows = AffineRows{
      Eigen::MatrixXd::Ones(1, 1), toVectorXd({ 0.0 }), { trajopt_ifopt::Bounds(-1.0, 1.0) }, toVectorXd({ 1.0 })
    };
    const Eigen::VectorXd x_new = toVectorXd({ 0.6 });
    qp->setVariables(x_new.data());
    EXPECT_THROW(qp->convexify(), std::runtime_error);  // NOLINT
  }
}

// A constraint set is held to the same row shapes as a cost set, and is refused when it is added.
TEST(QPProblemMerit, ConstraintRowsBoundedOnBothOrNoSidesAreRejected)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, 0.8 }) });
  for (const auto& unsupported : { trajopt_ifopt::Bounds(-1.0, 1.0), trajopt_ifopt::NoBound })
  {
    auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
    const std::vector<trajopt_ifopt::Bounds> bounds{ trajopt_ifopt::Bounds(0.0, 0.0), unsupported };
    EXPECT_THROW(qp->addConstraintSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "unsupported", bounds, constantWeights(toVectorXd({ 1.0, 1.0 })))),
                 std::runtime_error);
  }
}

// A dynamic constraint set that has no rows when it is added is refused by name once it reports an unbounded row.
TEST(QPProblemMerit, DynamicConstraintRowWithoutBoundsIsRejectedByNameWhenReported)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  auto rows = std::make_shared<AffineRows>();
  rows->jac = Eigen::MatrixXd::Zero(0, 1);
  qp->addConstraintSet(std::make_shared<AffineTestSet>(t.vars[0], "late", rows));
  qp->setup();
  qp->convexify();

  *rows =
      AffineRows{ Eigen::MatrixXd::Ones(1, 1), toVectorXd({ 0.0 }), { trajopt_ifopt::NoBound }, toVectorXd({ 1.0 }) };
  const Eigen::VectorXd x_new = toVectorXd({ 0.6 });
  qp->setVariables(x_new.data());
  try
  {
    qp->convexify();
    FAIL() << "convexify() accepted an unbounded row";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("'late'"), std::string::npos) << e.what();
  }
}

// A squared cost charges an equality row in the objective and a one-sided row through a slack that carries the
// row weight on its Hessian diagonal.
TEST(QPProblemMerit, SquaredCostModelsOneSidedRowsWithSlacks)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, 0.8, -0.3 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "mixed", mixedBounds(), constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();
  EXPECT_NEAR(qp->getExactCosts()(0), 2.04, 1e-12);  // 2 * 0.4^2 + 3 * 0.6^2 + 4 * 0.4^2

  // One slack per one-sided row, none for the equality row; two cost rows above the five variable rows.
  ASSERT_EQ(qp->getNumQPVars(), 5);
  ASSERT_EQ(qp->getNumQPConstraints(), 7);

  // 2 (x0 - 0.1)^2 in the objective; the slacks cost 3 s0^2 + 4 s1^2.
  expectVectorNear(qp->getGradient(), toVectorXd({ -0.4, 0.0, 0.0, 0.0, 0.0 }));
  expectDiagonalHessian(*qp, toVectorXd({ 2.0, 0.0, 0.0, 3.0, 4.0 }));
  EXPECT_EQ(qp->getHessian().nonZeros(), 3);  // the one-sided rows have no entry over the NLP variables

  // x1 - s0 <= 0.2 and x2 + s1 >= 0.1, with both slacks non-negative.
  Eigen::MatrixXd expected_rows(2, 5);
  expected_rows << 0, 1, 0, -1, 0, 0, 0, 1, 0, 1;
  const Eigen::MatrixXd rows = qp->getConstraintMatrix().toDense().topRows(2);
  EXPECT_TRUE(rows.isApprox(expected_rows)) << rows;
  EXPECT_EQ(qp->getBoundsLower()(0), -kInf);
  EXPECT_NEAR(qp->getBoundsUpper()(0), 0.2, 1e-12);
  EXPECT_NEAR(qp->getBoundsLower()(1), 0.1, 1e-12);
  EXPECT_EQ(qp->getBoundsUpper()(1), kInf);
  expectVectorNear(qp->getBoundsLower().tail(2), toVectorXd({ 0.0, 0.0 }));

  // The convex cost is read off the linearized rows, whatever the slack values.
  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(5);
  qp_vals.head(3) = toVectorXd({ 0.5, 0.8, -0.3 });
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 2.04, 1e-12);
  qp_vals.tail(2) = toVectorXd({ 0.6, 0.4 });
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 2.04, 1e-12);

  // x2 is inside its bound here, so only the first two rows are charged: 2 * 0.2^2 + 3 * 0.3^2.
  qp_vals.head(3) = toVectorXd({ 0.3, 0.5, 0.4 });
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 0.35, 1e-12);
}

// With no equality row the Hessian has no entries over the NLP variables and holds the row weights on the slack
// diagonal.
// A row of weight 0 keeps its slack and costs nothing.
TEST(QPProblemMerit, SquaredCostOfOnlyOneSidedRows)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.8, -0.3, 0.9 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  const std::vector<trajopt_ifopt::Bounds> bounds{ trajopt_ifopt::Bounds(-kInf, 0.2),
                                                   trajopt_ifopt::Bounds(0.1, kInf),
                                                   trajopt_ifopt::Bounds(-kInf, 0.2) };
  qp->addCostSet(
      std::make_shared<LinearTestSet>(t.vars[0], "one-sided", bounds, constantWeights(toVectorXd({ 3.0, 4.0, 0.0 }))),
      trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();
  EXPECT_NEAR(qp->getExactCosts()(0), 1.72, 1e-12);  // 3 * 0.6^2 + 4 * 0.4^2

  ASSERT_EQ(qp->getNumQPVars(), 6);
  expectVectorNear(qp->getGradient(), Eigen::VectorXd::Zero(6));
  expectDiagonalHessian(*qp, toVectorXd({ 0.0, 0.0, 0.0, 3.0, 4.0, 0.0 }));
  EXPECT_EQ(qp->getHessian().nonZeros(), 3);  // the slack diagonal only

  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(6);
  qp_vals.head(3) = toVectorXd({ 0.8, -0.3, 0.9 });
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 1.72, 1e-12);
}

// A one-sided row whose Jacobian row has no entries still gets its slack and its Hessian diagonal, with no other
// entry in the objective.
TEST(QPProblemMerit, SquaredOneSidedRowWithoutJacobianEntries)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  auto rows = std::make_shared<AffineRows>();
  rows->jac = Eigen::MatrixXd::Zero(1, 2);
  rows->offset = toVectorXd({ 0.5 });
  rows->bounds = { trajopt_ifopt::Bounds(-kInf, 0.2) };
  rows->weights = toVectorXd({ 3.0 });
  qp->addCostSet(std::make_shared<AffineTestSet>(t.vars[0], "constant", rows), trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();
  EXPECT_NEAR(qp->getExactCosts()(0), 0.27, 1e-12);  // 3 * 0.3^2

  ASSERT_EQ(qp->getNumQPVars(), 3);
  expectDiagonalHessian(*qp, toVectorXd({ 0.0, 0.0, 3.0 }));
  EXPECT_NEAR(qp->evaluateConvexCosts(toVectorXd({ 0.5, 0.8, 0.0 }))(0), 0.27, 1e-12);
}

// The QP row of a one-sided row is the linearized residual: its Jacobian row over the NLP variables, and its
// bound shifted by the residual's constant part.
TEST(QPProblemMerit, SquaredOneSidedRowsOfAnAffineResidual)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, 0.8 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  auto rows = std::make_shared<AffineRows>();
  rows->jac = (Eigen::MatrixXd(2, 2) << 1, 2, 1, -1).finished();
  rows->offset = toVectorXd({ 0.3, -0.5 });
  rows->bounds = { trajopt_ifopt::Bounds(-kInf, 1.0), trajopt_ifopt::Bounds(0.2, kInf) };
  rows->weights = toVectorXd({ 2.0, 3.0 });
  qp->addCostSet(std::make_shared<AffineTestSet>(t.vars[0], "affine", rows), trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();

  // The rows evaluate to 2.4 and -0.8, which is 1.4 above and 1.0 below their bounds: 2 * 1.4^2 + 3 * 1.0^2.
  EXPECT_NEAR(qp->getExactCosts()(0), 6.92, 1e-12);

  // x0 + 2 x1 - s0 <= 1.0 - 0.3 and x0 - x1 + s1 >= 0.2 + 0.5.
  ASSERT_EQ(qp->getNumQPVars(), 4);
  Eigen::MatrixXd expected_rows(2, 4);
  expected_rows << 1, 2, -1, 0, 1, -1, 0, 1;
  const Eigen::MatrixXd qp_rows = qp->getConstraintMatrix().toDense().topRows(2);
  EXPECT_TRUE(qp_rows.isApprox(expected_rows)) << qp_rows;
  EXPECT_EQ(qp->getBoundsLower()(0), -kInf);
  EXPECT_NEAR(qp->getBoundsUpper()(0), 0.7, 1e-12);
  EXPECT_NEAR(qp->getBoundsLower()(1), 0.7, 1e-12);
  EXPECT_EQ(qp->getBoundsUpper()(1), kInf);

  EXPECT_NEAR(qp->evaluateConvexCosts(toVectorXd({ 0.5, 0.8, 0.0, 0.0 }))(0), 6.92, 1e-12);
  // At (0.1, 0.2) the rows evaluate to 0.8, inside its bound, and -0.6, which is 0.8 below: 3 * 0.8^2.
  EXPECT_NEAR(qp->evaluateConvexCosts(toVectorXd({ 0.1, 0.2, 0.0, 0.0 }))(0), 1.92, 1e-12);
}

// A dynamic squared cost sizes its QP rows and slacks from the rows it reports at each convexification.
TEST(QPProblemMerit, DynamicSquaredCostFollowsTheRowsItReports)  // NOLINT
{
  const Eigen::VectorXd x = toVectorXd({ 0.5, 0.8, -0.3 });
  const TestVariables t = makeVariables({ x });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  auto rows = std::make_shared<AffineRows>();
  rows->jac = Eigen::MatrixXd::Identity(3, 3);
  rows->offset = Eigen::VectorXd::Zero(3);
  rows->bounds = mixedBounds();
  rows->weights = toVectorXd({ 2.0, 3.0, 4.0 });
  qp->addCostSet(std::make_shared<AffineTestSet>(t.vars[0], "changing", rows), trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();

  // An equality row, then an upper-bound and a lower-bound row with one slack each.
  ASSERT_EQ(qp->getNumQPVars(), 5);
  ASSERT_EQ(qp->getNumQPConstraints(), 7);
  expectDiagonalHessian(*qp, toVectorXd({ 2.0, 0.0, 0.0, 3.0, 4.0 }));
  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(5);
  qp_vals.head(3) = x;
  EXPECT_NEAR(qp->getExactCosts()(0), 2.04, 1e-12);  // 2 * 0.4^2 + 3 * 0.6^2 + 4 * 0.4^2
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 2.04, 1e-12);

  // At the next iterate the set reports a lower-bound row on x2, then an equality row on x0.
  rows->jac = (Eigen::MatrixXd(2, 3) << 0, 0, 1, 1, 0, 0).finished();
  rows->offset = Eigen::VectorXd::Zero(2);
  rows->bounds = { trajopt_ifopt::Bounds(0.1, kInf), trajopt_ifopt::Bounds(0.1, 0.1) };
  rows->weights = toVectorXd({ 5.0, 6.0 });
  const Eigen::VectorXd x_new = toVectorXd({ 0.5, 0.8, -0.2 });
  qp->setVariables(x_new.data());
  qp->convexify();

  ASSERT_EQ(qp->getNumQPVars(), 4);
  ASSERT_EQ(qp->getNumQPConstraints(), 5);
  expectDiagonalHessian(*qp, toVectorXd({ 6.0, 0.0, 0.0, 5.0 }));
  const Eigen::MatrixXd qp_row = qp->getConstraintMatrix().toDense().topRows(1);
  EXPECT_TRUE(qp_row.isApprox(toVectorXd({ 0.0, 0.0, 1.0, 1.0 }).transpose())) << qp_row;  // x2 + s >= 0.1
  EXPECT_NEAR(qp->getBoundsLower()(0), 0.1, 1e-12);
  EXPECT_EQ(qp->getBoundsUpper()(0), kInf);
  qp_vals = Eigen::VectorXd::Zero(4);
  qp_vals.head(3) = x_new;
  EXPECT_NEAR(qp->getExactCosts()(0), 1.41, 1e-12);  // 5 * 0.3^2 + 6 * 0.4^2
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 1.41, 1e-12);

  // Then two upper-bound rows and a lower-bound row: nothing of the earlier equality row stays in the objective.
  *rows = AffineRows{
    Eigen::MatrixXd::Identity(3, 3),
    Eigen::VectorXd::Zero(3),
    { trajopt_ifopt::Bounds(-kInf, 0.2), trajopt_ifopt::Bounds(-kInf, 0.2), trajopt_ifopt::Bounds(0.1, kInf) },
    toVectorXd({ 1.0, 2.0, 3.0 })
  };
  const Eigen::VectorXd x_last = toVectorXd({ 0.5, 0.8, -0.1 });
  qp->setVariables(x_last.data());
  qp->convexify();

  ASSERT_EQ(qp->getNumQPVars(), 6);
  ASSERT_EQ(qp->getNumQPConstraints(), 9);
  expectDiagonalHessian(*qp, toVectorXd({ 0.0, 0.0, 0.0, 1.0, 2.0, 3.0 }));
  EXPECT_EQ(qp->getHessian().nonZeros(), 3);
  qp_vals = Eigen::VectorXd::Zero(6);
  qp_vals.head(3) = x_last;
  EXPECT_NEAR(qp->getExactCosts()(0), 0.93, 1e-12);  // 1 * 0.3^2 + 2 * 0.6^2 + 3 * 0.2^2
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 0.93, 1e-12);
}

// A dynamic squared cost that reports no rows, after having had some, leaves the QP without objective and costs
// nothing; its rows come back with their cost.
TEST(QPProblemMerit, DynamicSquaredCostThatLosesItsRowsCostsNothing)  // NOLINT
{
  const Eigen::VectorXd x = toVectorXd({ 0.5, 0.8, -0.3 });
  const TestVariables t = makeVariables({ x });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  AffineRows some;
  some.jac = Eigen::MatrixXd::Identity(3, 3);
  some.offset = Eigen::VectorXd::Zero(3);
  some.bounds = mixedBounds();
  some.weights = toVectorXd({ 2.0, 3.0, 4.0 });
  AffineRows none;
  none.jac = Eigen::MatrixXd::Zero(0, 3);
  auto rows = std::make_shared<AffineRows>(some);
  qp->addCostSet(std::make_shared<AffineTestSet>(t.vars[0], "vanishing", rows), trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();
  ASSERT_EQ(qp->getNumQPVars(), 5);

  // The set's rows follow the iterate: none at the next one.
  *rows = none;
  const Eigen::VectorXd x_new = toVectorXd({ 0.4, 0.8, -0.3 });
  qp->setVariables(x_new.data());
  qp->convexify();
  ASSERT_EQ(qp->getNumQPVars(), 3);
  ASSERT_EQ(qp->getNumQPConstraints(), 3);
  EXPECT_TRUE(qp->getHessian().toDense().isZero());
  EXPECT_TRUE(qp->getGradient().isZero());
  expectVectorNear(qp->getExactCosts(), toVectorXd({ 0.0 }));
  expectVectorNear(qp->evaluateConvexCosts(x_new), toVectorXd({ 0.0 }));

  *rows = some;
  qp->setVariables(x.data());
  qp->convexify();
  ASSERT_EQ(qp->getNumQPVars(), 5);
  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(5);
  qp_vals.head(3) = x;
  EXPECT_NEAR(qp->getExactCosts()(0), 2.04, 1e-12);  // 2 * 0.4^2 + 3 * 0.6^2 + 4 * 0.4^2
  EXPECT_NEAR(qp->evaluateConvexCosts(qp_vals)(0), 2.04, 1e-12);
}

// The slack rows of a squared cost sit below the penalty and merit rows without disturbing them: with every
// row kind present, the convex model reproduces the exact costs and violations at the convexify point, and
// since every residual is linear the trust-region ratio of the step is 1.
TEST(QPProblemMerit, SquaredSlackRowsCoexistWithPenaltyAndMeritRows)  // NOLINT
{
  const TestVariables t =
      makeVariables({ toVectorXd({ 0.5, 0.8, -0.3 }), toVectorXd({ 0.4, -0.6 }), toVectorXd({ 0.3, -0.1 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "mixed", mixedBounds(), constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[1], "equality", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 4.0, 5.0 }))));
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[2], "hinge", trajopt_ifopt::BoundSmallerZero, constantWeights(toVectorXd({ 2.0, 3.0 }))),
                 trajopt_sqp::CostPenaltyType::kHinge);
  qp->setup();
  qp->convexify();

  // QP variables: the 7 NLP variables, the hinge slacks (7, 8), the equality slack pairs (9 to 12), then the
  // squared slacks (13, 14). QP rows: hinge (0, 1), equality (2, 3), squared one-sided (4, 5), then the variables.
  ASSERT_EQ(qp->getNumQPVars(), 15);
  ASSERT_EQ(qp->getNumQPConstraints(), 21);
  Eigen::VectorXd expected_diagonal = Eigen::VectorXd::Zero(15);
  expected_diagonal(0) = 2.0;
  expected_diagonal(13) = 3.0;
  expected_diagonal(14) = 4.0;
  expectDiagonalHessian(*qp, expected_diagonal);
  Eigen::MatrixXd expected_rows = Eigen::MatrixXd::Zero(2, 15);
  expected_rows(0, 1) = 1.0;  // x1 - s13 <= 0.2
  expected_rows(0, 13) = -1.0;
  expected_rows(1, 2) = 1.0;  // x2 + s14 >= 0.1
  expected_rows(1, 14) = 1.0;
  const Eigen::MatrixXd squared_rows = qp->getConstraintMatrix().toDense().middleRows(4, 2);
  EXPECT_TRUE(squared_rows.isApprox(expected_rows)) << squared_rows;
  expectVectorNear(qp->getGradient().segment(7, 2), toVectorXd({ 2.0, 3.0 }));
  expectVectorNear(qp->getGradient().tail(2), toVectorXd({ 0.0, 0.0 }));

  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(qp->getNumQPVars());
  qp_vals.head(7) = qp->getVariableValues();
  expectVectorNear(qp->getExactCosts(), toVectorXd({ 2.04, 0.6 }));  // hinge: 2 * 0.3
  expectVectorNear(qp->evaluateConvexCosts(qp_vals), qp->getExactCosts());
  const trajopt_sqp::ConstraintViolations exact = qp->getExactConstraintViolations();
  expectVectorNear(exact.weighted, toVectorXd({ 4.6 }));  // 4 * 0.4 + 5 * 0.6
  expectVectorNear(qp->evaluateConvexConstraintViolations(qp_vals).weighted, exact.weighted);

  trajopt_sqp::TrustRegionSQPSolver solver(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
  solver.init(qp);
  solver.stepSQPSolver();
  const trajopt_sqp::SQPResults& results = solver.getResults();
  EXPECT_GT(results.approx_merit_improve, 0.0);
  EXPECT_NEAR(results.merit_improve_ratio, 1.0, 1e-9);
}

// Minimizing x0^2 + 3 max(0, x1 - 0.2)^2 + (x0 - 1)^2 + (x1 - 1)^2: x0 settles halfway, and x1 stops past its
// bound where the squared violation balances the pull, 6 (x1 - 0.2) = 2 (1 - x1).
TEST(QPProblemMerit, SquaredMixedRowCostSolvesToItsOptimum)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.0, 0.0 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  const std::vector<trajopt_ifopt::Bounds> bounds{ trajopt_ifopt::Bounds(0.0, 0.0), trajopt_ifopt::Bounds(-kInf, 0.2) };
  qp->addCostSet(std::make_shared<LinearTestSet>(t.vars[0], "mixed", bounds, constantWeights(toVectorXd({ 1.0, 3.0 }))),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "pull", trajopt_ifopt::Bounds(1.0, 1.0), constantWeights(toVectorXd({ 1.0, 1.0 }))),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();

  trajopt_sqp::TrustRegionSQPSolver solver(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
  solver.solve(qp);
  expectVectorNear(qp->getVariableValues(), toVectorXd({ 0.5, 0.4 }), 1e-3);
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
  ASSERT_EQ(results.best_var_vals.size(), x0.size());
  ASSERT_FALSE(results.best_var_vals.isApprox(x0));
  expectVectorNear(results.best_constraint_violations.weighted, qp->getExactConstraintViolations().weighted);
}

// Calling setup() again on an unchanged problem gives the same QP.
TEST(QPProblemMerit, SecondSetupGivesTheSameQP)  // NOLINT
{
  const TestVariables t =
      makeVariables({ toVectorXd({ 0.5, 0.8, -0.3 }), toVectorXd({ 0.4, -0.6 }), toVectorXd({ 0.3, -0.1 }) });
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[0], "mixed", mixedBounds(), constantWeights(toVectorXd({ 2.0, 3.0, 4.0 }))),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->addConstraintSet(std::make_shared<LinearTestSet>(
      t.vars[1], "equality", trajopt_ifopt::Bounds(0.0, 0.0), constantWeights(toVectorXd({ 4.0, 5.0 }))));
  qp->addCostSet(std::make_shared<LinearTestSet>(
                     t.vars[2], "hinge", trajopt_ifopt::BoundSmallerZero, constantWeights(toVectorXd({ 2.0, 3.0 }))),
                 trajopt_sqp::CostPenaltyType::kHinge);
  qp->setup();
  qp->convexify();
  const Eigen::MatrixXd hessian = qp->getHessian().toDense();
  const Eigen::MatrixXd constraint_matrix = qp->getConstraintMatrix().toDense();
  const Eigen::VectorXd gradient = qp->getGradient();
  const Eigen::VectorXd bounds_lower = qp->getBoundsLower();
  const Eigen::VectorXd bounds_upper = qp->getBoundsUpper();

  qp->setup();
  qp->convexify();
  EXPECT_EQ(qp->getHessian().toDense(), hessian);
  EXPECT_EQ(qp->getConstraintMatrix().toDense(), constraint_matrix);
  EXPECT_EQ(qp->getGradient(), gradient);
  EXPECT_EQ(qp->getBoundsLower(), bounds_lower);
  EXPECT_EQ(qp->getBoundsUpper(), bounds_upper);
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

// A squared cost keeps its curvature when the products of its Jacobian entries are small: only the linearized
// rows are filtered, never their square. The objective handed to the solver is the model the cost is scored with.
TEST(QPProblemMerit, SquaredCostKeepsCurvatureOfSmallProducts)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.2 }) });
  Eigen::MatrixXd m(1, 2);
  m << 2e-4, 3e-4;
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(
      std::make_shared<MatrixTestSet>(t.vars[0], "squared", m, trajopt_ifopt::Bounds(0.0, 0.0), toVectorXd({ 1.0 })),
      trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();

  const Eigen::MatrixXd hessian = qp->getHessian().toDense();
  const Eigen::MatrixXd expected = m.transpose() * m;
  ASSERT_EQ(hessian.rows(), 2);
  ASSERT_EQ(hessian.cols(), 2);
  for (Eigen::Index r = 0; r < 2; ++r)
    for (Eigen::Index c = 0; c < 2; ++c)
      EXPECT_NEAR(hessian(r, c), expected(r, c), 1e-20) << "at (" << r << ", " << c << ")";

  // The QP minimizes x'Hx + g'x, which differs from the scored model by a constant only.
  const Eigen::VectorXd gradient = qp->getGradient();
  const auto objective = [&](const Eigen::VectorXd& x) { return x.dot(hessian * x) + gradient.dot(x); };
  const Eigen::VectorXd xa = toVectorXd({ 0.5, -0.2 });
  const Eigen::VectorXd xb = toVectorXd({ 10.0, 10.0 });
  EXPECT_NEAR(
      qp->evaluateConvexCosts(xb).sum() - qp->evaluateConvexCosts(xa).sum(), objective(xb) - objective(xa), 1e-15);
}

// Entries at or below 1e-7 in a squared cost's linearized rows are treated as zero before the rows are squared, as
// trajopt_sco does, so they reach neither the Hessian nor the gradient. They stay in the sparsity pattern.
TEST(QPProblemMerit, SquaredCostDropsSmallJacobianEntriesBeforeSquaring)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, -0.2, 0.3 }) });
  Eigen::MatrixXd m(2, 3);
  m.row(0) << 1.0, 1e-7, -5e-8;   // one retained entry beside one at the threshold and a smaller one
  m.row(1) << 3e-8, -2e-8, 4e-8;  // every entry small
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(std::make_shared<MatrixTestSet>(
                     t.vars[0], "squared", m, trajopt_ifopt::Bounds(0.1, 0.1), toVectorXd({ 100.0, 100.0 })),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();

  // What remains is 100 * (b - x_0)^2 with b within 1e-7 of the bound 0.1: Hessian 100 at (0, 0), gradient about
  // -20 at 0.
  const Eigen::MatrixXd hessian = qp->getHessian().toDense();
  ASSERT_EQ(hessian.rows(), 3);
  ASSERT_EQ(hessian.cols(), 3);
  EXPECT_NEAR(hessian(0, 0), 100.0, 1e-9);
  EXPECT_EQ((hessian.array() != 0.0).count(), 1) << hessian;

  const Eigen::VectorXd& gradient = qp->getGradient();
  ASSERT_EQ(gradient.size(), 3);
  EXPECT_NEAR(gradient(0), -20.0, 1e-4);
  EXPECT_EQ(gradient(1), 0.0);
  EXPECT_EQ(gradient(2), 0.0);

  EXPECT_EQ(qp->getHessian().nonZeros(), 9);
}

// The convex model of a squared cost reproduces the exact cost at the point it was linearized at, also when that
// point is far from the origin and the Jacobian has an entry that is filtered out.
TEST(QPProblemMerit, SquaredCostModelIsExactAtConvexifyPoint)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, 1000.0 }) });
  Eigen::MatrixXd m(1, 2);
  m << 1.0, 5e-8;
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(
      std::make_shared<MatrixTestSet>(t.vars[0], "squared", m, trajopt_ifopt::Bounds(0.0, 0.0), toVectorXd({ 1.0 })),
      trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  qp->convexify();

  // The row's value is 0.5 + 1000 * 5e-8 = 0.50005.
  expectVectorNear(qp->getExactCosts(), toVectorXd({ 0.2500500025 }));
  expectVectorNear(qp->evaluateConvexCosts(qp->getVariableValues()), qp->getExactCosts());
}

// The convex model of the rows that enter the QP as constraints (hinge costs and merit constraints) reproduces
// their exact values at the point it was linearized at. Their Jacobian entries below 1e-7 are zero in the
// constraint matrix and stay in its sparsity pattern.
TEST(QPProblemMerit, ConstraintRowModelIsExactAtConvexifyPoint)  // NOLINT
{
  const TestVariables t = makeVariables({ toVectorXd({ 0.5, 1000.0 }), toVectorXd({ 0.5, 1000.0 }) });
  Eigen::MatrixXd m(1, 2);
  m << 1.0, 5e-8;
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(t.variables);
  qp->addCostSet(
      std::make_shared<MatrixTestSet>(t.vars[0], "hinge", m, trajopt_ifopt::BoundSmallerZero, toVectorXd({ 2.0 })),
      trajopt_sqp::CostPenaltyType::kHinge);
  qp->addConstraintSet(
      std::make_shared<MatrixTestSet>(t.vars[1], "equality", m, trajopt_ifopt::Bounds(0.0, 0.0), toVectorXd({ 3.0 })));
  qp->setup();
  qp->convexify();

  // The hinge row has one slack and the equality row two, followed by one identity row per QP variable.
  const trajopt_ifopt::Jacobian& constraint_matrix = qp->getConstraintMatrix();
  EXPECT_EQ(constraint_matrix.coeff(0, 0), 1.0);
  EXPECT_EQ(constraint_matrix.coeff(0, 1), 0.0);
  EXPECT_EQ(constraint_matrix.coeff(1, 2), 1.0);
  EXPECT_EQ(constraint_matrix.coeff(1, 3), 0.0);
  EXPECT_EQ(constraint_matrix.nonZeros(), 14);

  Eigen::VectorXd qp_vals = Eigen::VectorXd::Zero(qp->getNumQPVars());
  qp_vals.head(4) = qp->getVariableValues();

  // Each row's value is 0.50005: the hinge cost is twice that, the equality is violated by that.
  expectVectorNear(qp->getExactCosts(), toVectorXd({ 1.0001 }));
  expectVectorNear(qp->evaluateConvexCosts(qp_vals), qp->getExactCosts());

  const trajopt_sqp::ConstraintViolations exact = qp->getExactConstraintViolations();
  const trajopt_sqp::ConstraintViolations convex = qp->evaluateConvexConstraintViolations(qp_vals);
  expectVectorNear(exact.raw, toVectorXd({ 0.50005 }));
  expectVectorNear(convex.raw, exact.raw);
  expectVectorNear(convex.weighted, exact.weighted);
}
