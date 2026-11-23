/**
 * @file qp_row_reduction_unit.cpp
 * @brief Tests the reduction of the QP of TrajOptQPProblem to the one-sided slack rows that can change sign in the box
 *
 * @author Roelof Oomen
 * @date November 6, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <tesseract/common/logging.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>

#include "scripted_qp_solver.h"

namespace
{
const double kInf = std::numeric_limits<double>::infinity();
/** @brief The default merit coefficient, which weighs every constraint row's slack */
const double kMerit = 10.0;
/** @brief The default trust box half width */
const double kBox = 0.1;

/** @brief One fixed row of a LinearRows set: value a·x, its bound and its coefficient. */
struct Row
{
  Eigen::Vector2d a;
  trajopt_ifopt::Bounds bound;
  double w{ 1.0 };
};

/** @brief A row a·x <= ub of coefficient w. */
Row upper(const Eigen::Vector2d& a, double ub, double w = 1) { return { a, trajopt_ifopt::Bounds(-kInf, ub), w }; }

/** @brief A row a·x >= lb of coefficient w. */
Row lower(const Eigen::Vector2d& a, double lb, double w = 1) { return { a, trajopt_ifopt::Bounds(lb, kInf), w }; }

/** @brief Constraint set of fixed rows a_i·x over a two-joint variable block. */
class LinearRows : public trajopt_ifopt::ConstraintSet
{
public:
  LinearRows(std::shared_ptr<const trajopt_ifopt::Var> var, std::string name, std::vector<Row> rows)
    : ConstraintSet(std::move(name), static_cast<int>(rows.size())), var_(std::move(var)), rows_spec_(std::move(rows))
  {
    non_zeros_ = 2 * static_cast<Eigen::Index>(rows_);
  }

  int update() override { return rows_; }

  Eigen::VectorXd getValues() const override
  {
    Eigen::VectorXd v(rows_);
    for (Eigen::Index i = 0; i < rows_; ++i)
      v(i) = rows_spec_[static_cast<std::size_t>(i)].a.dot(var_->value());
    return v;
  }

  Eigen::VectorXd getCoefficients() const override
  {
    Eigen::VectorXd w(rows_);
    for (Eigen::Index i = 0; i < rows_; ++i)
      w(i) = rows_spec_[static_cast<std::size_t>(i)].w;
    return w;
  }

  std::vector<trajopt_ifopt::Bounds> getBounds() const override
  {
    std::vector<trajopt_ifopt::Bounds> b;
    b.reserve(rows_spec_.size());
    for (const auto& r : rows_spec_)
      b.push_back(r.bound);
    return b;
  }

  trajopt_ifopt::Jacobian getJacobian() const override
  {
    trajopt_ifopt::Jacobian jac(rows_, variables_->getRows());
    jac.reserve(non_zeros_);
    for (Eigen::Index i = 0; i < rows_; ++i)
    {
      jac.startVec(i);
      for (Eigen::Index c = 0; c < 2; ++c)
        jac.insertBack(i, var_->getIndex() + c) = rows_spec_[static_cast<std::size_t>(i)].a(c);
    }
    jac.finalize();
    return jac;
  }

private:
  std::shared_ptr<const trajopt_ifopt::Var> var_;
  std::vector<Row> rows_spec_;
};

/** @brief A problem of two-joint waypoints, by default one, each at x = (0, 0) and unbounded. */
struct TestProblem
{
  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  std::shared_ptr<trajopt_sqp::TrajOptQPProblem> qp;

  explicit TestProblem(const Eigen::Vector2d& x0 = Eigen::Vector2d::Zero(),
                       const std::vector<trajopt_ifopt::Bounds>& bounds = { trajopt_ifopt::NoBound,
                                                                            trajopt_ifopt::NoBound },
                       int waypoints = 1)
  {
    std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
    for (int i = 0; i < waypoints; ++i)
    {
      auto node = std::make_unique<trajopt_ifopt::Node>("Joints" + std::to_string(i));
      vars.push_back(node->addVar("position", { "j0", "j1" }, x0, bounds));
      nodes.push_back(std::move(node));
    }
    qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(
        std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes)));
  }

  /** @brief Add @p rows over the variables of @p waypoint as a constraint set. */
  void addConstraint(std::vector<Row> rows, std::size_t waypoint = 0)
  {
    qp->addConstraintSet(std::make_shared<LinearRows>(vars[waypoint], "cnt", std::move(rows)));
  }

  /** @brief Add @p rows over the variables of @p waypoint as a cost set. */
  void addCost(std::vector<Row> rows, trajopt_sqp::CostPenaltyType type, std::size_t waypoint = 0)
  {
    qp->addCostSet(std::make_shared<LinearRows>(vars[waypoint], "cost", std::move(rows)), type);
  }
};

/** @brief Forwards to OSQP and records whether a QP arrived in other dimensions than the solver was initialized for. */
class DimensionCheckingSolver : public trajopt_sqp::test::ScriptedQPSolver
{
public:
  DimensionCheckingSolver() : ScriptedQPSolver(std::make_shared<trajopt_sqp::OSQPEigenSolver>()) {}

  bool mismatched{ false };

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts) override
  {
    num_vars_ = num_vars;
    num_cnts_ = num_cnts;
    return ScriptedQPSolver::init(num_vars, num_cnts);
  }

  bool updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian) override
  {
    mismatched = mismatched || hessian.rows() != num_vars_;
    return ScriptedQPSolver::updateHessianMatrix(hessian);
  }

  bool updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& matrix) override
  {
    mismatched = mismatched || matrix.rows() != num_cnts_ || matrix.cols() != num_vars_;
    return ScriptedQPSolver::updateLinearConstraintsMatrix(matrix);
  }

private:
  Eigen::Index num_vars_{ 0 };
  Eigen::Index num_cnts_{ 0 };
};

/** @brief A copy of everything the QP solver reads from the problem. */
struct QPSnapshot
{
  Eigen::Index num_vars{ 0 };
  Eigen::Index num_cnts{ 0 };
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  Eigen::MatrixXd constraint_matrix;
  Eigen::VectorXd bounds_lower;
  Eigen::VectorXd bounds_upper;

  /** @brief The rows above the variable rows: the linearized constraint and cost rows */
  Eigen::Index rows() const { return num_cnts - num_vars; }

  /**
   * @brief The QP objective z'Hz + g'z at NLP values @p x, each slack at its least feasible value
   * @details Each slack must sit in one one-sided row; that is the least value, and the objective's minimum over it.
   */
  double value(const Eigen::Vector2d& x) const
  {
    Eigen::VectorXd z = Eigen::VectorXd::Zero(num_vars);
    z.head(2) = x;
    for (Eigen::Index c = 2; c < num_vars; ++c)
    {
      Eigen::Index row = -1;
      for (Eigen::Index r = 0; r < rows(); ++r)
      {
        if (constraint_matrix(r, c) != 0.0)
        {
          EXPECT_EQ(row, -1) << "slack " << c << " sits in more than one row";
          row = r;
        }
      }
      EXPECT_GE(row, 0) << "slack " << c << " sits in no row";
      const double ax = constraint_matrix.row(row).head(2).dot(x);
      const double coeff = constraint_matrix(row, c);
      z(c) = (coeff < 0) ? std::max(0.0, ax - bounds_upper(row)) : std::max(0.0, bounds_lower(row) - ax);
    }
    return z.dot(hessian * z) + gradient.dot(z);
  }

  /** @brief The trust box the QP solves over: the bounds of the two NLP variable rows */
  Eigen::Vector2d boxLower() const { return bounds_lower.segment(rows(), 2); }
  Eigen::Vector2d boxUpper() const { return bounds_upper.segment(rows(), 2); }
};

QPSnapshot snapshot(const trajopt_sqp::QPProblem& qp)
{
  return { qp.getNumQPVars(),  qp.getNumQPConstraints(),           qp.getHessian().toDense(),
           qp.getGradient(),   qp.getConstraintMatrix().toDense(), qp.getBoundsLower(),
           qp.getBoundsUpper() };
}

/** @brief Convexify @p qp with the reduction set to @p reduce and return its QP. */
QPSnapshot convexified(trajopt_sqp::QPProblem& qp, bool reduce)
{
  qp.setQPRowReduction(reduce);
  qp.convexify();
  return snapshot(qp);
}

void expectBitEqual(const QPSnapshot& a, const QPSnapshot& b)
{
  ASSERT_EQ(a.num_vars, b.num_vars);
  ASSERT_EQ(a.num_cnts, b.num_cnts);
  EXPECT_TRUE((a.hessian.array() == b.hessian.array()).all());
  EXPECT_TRUE((a.gradient.array() == b.gradient.array()).all());
  EXPECT_TRUE((a.constraint_matrix.array() == b.constraint_matrix.array()).all());
  EXPECT_TRUE((a.bounds_lower.array() == b.bounds_lower.array()).all());
  EXPECT_TRUE((a.bounds_upper.array() == b.bounds_upper.array()).all());
}

/**
 * @brief Expect that @p reduced solves over the box of @p full and that its objective differs from the full QP's,
 * slacks minimized, by one constant over a grid of that box: the two QPs then have the same minimizers
 */
void expectSameModel(const QPSnapshot& full, const QPSnapshot& reduced)
{
  ASSERT_TRUE(full.boxLower() == reduced.boxLower());
  ASSERT_TRUE(full.boxUpper() == reduced.boxUpper());
  const Eigen::Vector2d lo = full.boxLower();
  const Eigen::Vector2d hi = full.boxUpper();
  const int n = 6;
  const double offset = full.value(lo) - reduced.value(lo);
  for (int i = 0; i <= n; ++i)
  {
    for (int j = 0; j <= n; ++j)
    {
      const Eigen::Vector2d x(lo(0) + ((hi(0) - lo(0)) * i / n), lo(1) + ((hi(1) - lo(1)) * j / n));
      EXPECT_NEAR(full.value(x) - reduced.value(x), offset, 1e-12 * (1 + std::abs(offset))) << "at " << x.transpose();
    }
  }
}

/** @brief Convexify @p qp without and with the reduction, check the model and return the reduced QP. */
QPSnapshot reducedAndChecked(trajopt_sqp::QPProblem& qp)
{
  const QPSnapshot full = convexified(qp, false);
  const QPSnapshot reduced = convexified(qp, true);
  expectSameModel(full, reduced);
  return reduced;
}

/** @brief Expect that the reduction leaves the QP exactly as it is without it. */
void expectNothingReduced(trajopt_sqp::QPProblem& qp)
{
  const QPSnapshot full = convexified(qp, false);
  expectBitEqual(convexified(qp, true), full);
}
}  // namespace

class QPRowReduction : public testing::Test
{
protected:
  void SetUp() override
  {
    level_ = tesseract::common::getLogger()->level();
    tesseract::common::getLogger()->set_level(spdlog::level::off);
  }

  void TearDown() override { tesseract::common::getLogger()->set_level(level_); }

private:
  spdlog::level::level_enum level_{ spdlog::level::info };
};

/** @brief Off must leave the QP exactly as it is without the option, also after it was on */
TEST_F(QPRowReduction, OffKeepsTheFullQP)  // NOLINT
{
  TestProblem never_set;
  never_set.addConstraint({ upper({ 1, 0 }, 1) });
  never_set.qp->setup();
  never_set.qp->convexify();
  const QPSnapshot full = snapshot(*never_set.qp);
  EXPECT_EQ(full.rows(), 1);

  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, 1) });
  p.qp->setup();
  EXPECT_EQ(convexified(*p.qp, true).rows(), 0);
  expectBitEqual(convexified(*p.qp, false), full);
}

/** @brief x <= 1 holds everywhere in the box [-0.1, 0.1]: the row and its slack go */
TEST_F(QPRowReduction, DropsRowThatCannotBecomeViolated)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, 1), upper({ 0, 1 }, 0.05) });
  p.qp->setup();
  const QPSnapshot reduced = reducedAndChecked(*p.qp);
  ASSERT_EQ(reduced.rows(), 1);
  EXPECT_EQ(reduced.num_vars, 3);
  EXPECT_EQ(reduced.constraint_matrix(0, 1), 1.0);
  EXPECT_EQ(reduced.bounds_upper(0), 0.05);
}

/** @brief x <= -1 is violated everywhere in the box: its slack is x + 1 there, a linear cost of weight w */
TEST_F(QPRowReduction, FoldsRowThatCannotBecomeSatisfied)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, -1, 2) });
  p.qp->setup();
  const QPSnapshot full = convexified(*p.qp, false);
  const QPSnapshot reduced = reducedAndChecked(*p.qp);
  ASSERT_EQ(reduced.rows(), 0);
  EXPECT_EQ(reduced.num_vars, 2);
  EXPECT_DOUBLE_EQ(reduced.gradient(0), full.gradient(0) + (kMerit * 2));
  EXPECT_EQ(reduced.gradient(1), full.gradient(1));
}

/** @brief A lower-bounded row is classified as its negation: -x >= -1 is dropped, x >= 1 folds to -w·x */
TEST_F(QPRowReduction, MirrorsLowerBoundedRows)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ lower({ 1, 0 }, -1), lower({ 0, 1 }, 1, 2), lower({ 1, 1 }, 0.05) });
  p.qp->setup();
  const QPSnapshot full = convexified(*p.qp, false);
  const QPSnapshot reduced = reducedAndChecked(*p.qp);
  ASSERT_EQ(reduced.rows(), 1);
  EXPECT_EQ(reduced.bounds_lower(0), 0.05);
  EXPECT_DOUBLE_EQ(reduced.gradient(1), full.gradient(1) - (kMerit * 2));
}

/** @brief A row that can change sign in the box is kept, so the QP is the full one */
TEST_F(QPRowReduction, KeepsRowThatCanChangeSign)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, 0.05), lower({ 1, 1 }, -0.15) });
  p.qp->setup();
  expectNothingReduced(*p.qp);
}

/** @brief x <= 0.1 is tight at the box edge, within rounding of changing sign: kept, until the box shrinks */
TEST_F(QPRowReduction, KeepsRowAtTheBoxEdge)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, kBox) });
  p.qp->setup();
  expectNothingReduced(*p.qp);

  p.qp->setBoxSize(Eigen::Vector2d::Constant(kBox / 2));
  EXPECT_EQ(reducedAndChecked(*p.qp).rows(), 0);
}

/** @brief A hinge cost row and a constraint row that are positive multiples become one row with both weights */
TEST_F(QPRowReduction, MergesPositiveMultiples)  // NOLINT
{
  TestProblem p;
  const Eigen::Vector2d a(1, 0.5);
  p.addConstraint({ upper(a, 0.02, 1) });
  p.addCost({ upper(2 * a, 0.04, 3) }, trajopt_sqp::CostPenaltyType::kHinge);
  p.qp->setup();
  const QPSnapshot reduced = reducedAndChecked(*p.qp);
  ASSERT_EQ(reduced.rows(), 1);
  ASSERT_EQ(reduced.num_vars, 3);
  // The hinge cost row comes first in the QP and leads; the constraint row, half as long, adds half its weight
  EXPECT_EQ(reduced.constraint_matrix(0, 0), 2 * a(0));
  EXPECT_EQ(reduced.constraint_matrix(0, 1), 2 * a(1));
  EXPECT_EQ(reduced.bounds_upper(0), 0.04);
  EXPECT_DOUBLE_EQ(reduced.gradient(2), 3 + (kMerit * 1 * 0.5));
}

/** @brief Parallel rows with different normalized bounds have their own kinks: both stay */
TEST_F(QPRowReduction, KeepsParallelRowsWithDifferentBounds)  // NOLINT
{
  TestProblem p;
  const Eigen::Vector2d a(1, 0.5);
  p.addConstraint({ upper(a, 0.02), upper(2 * a, 0.05) });
  p.qp->setup();
  expectNothingReduced(*p.qp);
}

/** @brief Rows that differ only in a coefficient far smaller than their largest are not multiples: both stay */
TEST_F(QPRowReduction, KeepsRowsThatDifferInASmallCoefficient)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper(Eigen::Vector2d(1e12, 1), 0), upper(Eigen::Vector2d(1e12, 1.5), 0) });
  p.qp->setup();
  expectNothingReduced(*p.qp);
}

/** @brief Equal rows of different waypoints sit in different columns: each waypoint's pair merges on its own */
TEST_F(QPRowReduction, MergesWithinAWaypointOnly)  // NOLINT
{
  TestProblem p(Eigen::Vector2d::Zero(), { trajopt_ifopt::NoBound, trajopt_ifopt::NoBound }, 2);
  const Eigen::Vector2d a(1, 0.5);
  for (std::size_t waypoint = 0; waypoint < 2; ++waypoint)
  {
    p.addConstraint({ upper(a, 0.02, 1) }, waypoint);
    p.addCost({ upper(a, 0.02, 3) }, trajopt_sqp::CostPenaltyType::kHinge, waypoint);
  }
  p.qp->setup();
  ASSERT_EQ(convexified(*p.qp, false).rows(), 4);
  const QPSnapshot reduced = convexified(*p.qp, true);
  ASSERT_EQ(reduced.rows(), 2);
  ASSERT_EQ(reduced.num_vars, 6);
  for (Eigen::Index row = 0; row < 2; ++row)
  {
    Eigen::RowVectorXd expected = Eigen::RowVectorXd::Zero(6);
    expected.segment(2 * row, 2) = a;
    expected(4 + row) = -1;
    EXPECT_TRUE((reduced.constraint_matrix.row(row).array() == expected.array()).all()) << "row " << row;
    EXPECT_DOUBLE_EQ(reduced.gradient(4 + row), 3 + kMerit);
  }
}

/** @brief The one-sided rows of a squared cost are left alone, even one that cannot become satisfied */
TEST_F(QPRowReduction, LeavesSquaredCostRowsAlone)  // NOLINT
{
  TestProblem p;
  p.addCost({ upper({ 1, 0 }, -1), upper({ 0, 1 }, 1) }, trajopt_sqp::CostPenaltyType::kSquared);
  p.qp->setup();
  expectNothingReduced(*p.qp);
}

/**
 * @brief Rows copied into a QP that is reduced keep their slacks, renumbered: an equality row its two, the one-sided
 * row of a squared cost its one with the weight on the Hessian diagonal
 */
TEST_F(QPRowReduction, CopiesOtherRowsIntoAReducedQP)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, 1),
                    { { 0, 1 }, trajopt_ifopt::Bounds(0.02, 0.02), 1 },
                    upper({ 1, 0 }, -1, 2),
                    upper({ 1, 1 }, 0.05) });
  p.addCost({ upper({ 1, 0 }, -1, 3), { { 0, 1 }, trajopt_ifopt::Bounds(0.3, 0.3), 1 } },
            trajopt_sqp::CostPenaltyType::kSquared);
  p.qp->setup();
  const QPSnapshot full = convexified(*p.qp, false);
  ASSERT_EQ(full.rows(), 5);
  ASSERT_EQ(full.num_vars, 8);
  const QPSnapshot reduced = reducedAndChecked(*p.qp);
  ASSERT_EQ(reduced.rows(), 3);
  ASSERT_EQ(reduced.num_vars, 6);

  // The equality row, with slacks 2 and 3
  EXPECT_EQ(reduced.constraint_matrix(0, 1), 1.0);
  EXPECT_EQ(reduced.constraint_matrix(0, 2), 1.0);
  EXPECT_EQ(reduced.constraint_matrix(0, 3), -1.0);
  EXPECT_EQ(reduced.bounds_lower(0), full.bounds_lower(1));
  EXPECT_EQ(reduced.bounds_upper(0), full.bounds_upper(1));
  EXPECT_EQ(reduced.gradient(2), kMerit);
  EXPECT_EQ(reduced.gradient(3), kMerit);

  // The kept row, with slack 4
  EXPECT_EQ(reduced.constraint_matrix(1, 4), -1.0);
  EXPECT_EQ(reduced.bounds_upper(1), 0.05);
  EXPECT_EQ(reduced.gradient(4), kMerit);

  // The squared cost's row, with slack 5 in place of the full QP's 7
  EXPECT_EQ(reduced.constraint_matrix(2, 0), 1.0);
  EXPECT_EQ(reduced.constraint_matrix(2, 5), -1.0);
  EXPECT_EQ(reduced.bounds_upper(2), full.bounds_upper(4));
  EXPECT_EQ(reduced.gradient(5), 0.0);
  ASSERT_NE(full.hessian(7, 7), 0.0);
  EXPECT_EQ(reduced.hessian(5, 5), full.hessian(7, 7));
  EXPECT_TRUE((reduced.hessian.topLeftCorner(2, 2).array() == full.hessian.topLeftCorner(2, 2).array()).all());
  EXPECT_EQ(reduced.hessian.diagonal().segment(2, 3).cwiseAbs().maxCoeff(), 0.0);

  // Every slack keeps its bounds
  EXPECT_TRUE((reduced.bounds_lower.tail(4).array() == 0.0).all());
  EXPECT_TRUE((reduced.bounds_upper.tail(4).array() == kInf).all());
}

/** @brief A bound of 1e20 is infinite to Bounds, so a row with one is one-sided and reduced like any other */
TEST_F(QPRowReduction, ReducesRowsBoundedByALargeFiniteValue)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ { { 1, 0 }, trajopt_ifopt::Bounds(-1e20, 1), 1 },
                    { { 0, 1 }, trajopt_ifopt::Bounds(-1, 1e20), 1 },
                    { { 1, 1 }, trajopt_ifopt::Bounds(-1e20, -1), 1 } });
  p.qp->setup();
  const QPSnapshot full = convexified(*p.qp, false);
  const QPSnapshot reduced = reducedAndChecked(*p.qp);
  EXPECT_EQ(reduced.rows(), 0);
  EXPECT_DOUBLE_EQ(reduced.gradient(0), full.gradient(0) + kMerit);
  EXPECT_DOUBLE_EQ(reduced.gradient(1), full.gradient(1) + kMerit);
}

/** @brief setup() discards the reduced QP, which describes the problem as it was convexified before */
TEST_F(QPRowReduction, SetupDiscardsTheReducedQP)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, 1) });
  p.qp->setup();
  const QPSnapshot full = convexified(*p.qp, false);
  ASSERT_EQ(convexified(*p.qp, true).rows(), 0);
  p.qp->setup();
  EXPECT_EQ(p.qp->getNumQPVars(), full.num_vars);
  EXPECT_EQ(p.qp->getNumQPConstraints(), full.num_cnts);
}

/**
 * @brief With j0 set to 2 beyond its limit 1, the box is [0.9, 1] for j0, not 2 +- 0.1: x <= 1.5 holds everywhere in
 * it, although it is violated at the current values
 */
TEST_F(QPRowReduction, ClassifiesOverTheClampedBox)  // NOLINT
{
  TestProblem p(Eigen::Vector2d::Zero(), { trajopt_ifopt::Bounds(-1, 1), trajopt_ifopt::NoBound });
  p.addConstraint({ upper({ 1, 0 }, 1.5), upper({ 0, 1 }, 0.05) });
  p.qp->setup();
  // Construction clamps the values into their limits, setting them does not
  const Eigen::Vector2d outside(2, 0);
  p.qp->setVariables(outside.data());
  ASSERT_EQ(p.qp->getVariableValues()(0), 2.0);
  const QPSnapshot reduced = reducedAndChecked(*p.qp);
  EXPECT_EQ(reduced.boxLower()(0), 0.9);
  EXPECT_EQ(reduced.boxUpper()(0), 1.0);
  ASSERT_EQ(reduced.rows(), 1);
  EXPECT_EQ(reduced.constraint_matrix(0, 1), 1.0);
}

/**
 * @brief The reduced QP never solves over more than the box it was built for: a box grown without a convexify() is
 * clipped to it, a shrunk one is served, and the next convexify() builds for the grown box
 */
TEST_F(QPRowReduction, ClipsTheBoxToTheOneItWasBuiltFor)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, 0.3), upper({ 0, 1 }, 0.05) });
  p.qp->setup();
  const QPSnapshot reduced = convexified(*p.qp, true);
  ASSERT_EQ(reduced.rows(), 1);

  p.qp->setBoxSize(Eigen::Vector2d::Constant(0.5));
  const QPSnapshot grown = snapshot(*p.qp);
  EXPECT_EQ(grown.num_vars, reduced.num_vars);
  EXPECT_EQ(grown.num_cnts, reduced.num_cnts);
  EXPECT_TRUE(grown.boxLower() == Eigen::Vector2d::Constant(-kBox));
  EXPECT_TRUE(grown.boxUpper() == Eigen::Vector2d::Constant(kBox));

  p.qp->setBoxSize(Eigen::Vector2d::Constant(kBox / 2));
  EXPECT_TRUE(snapshot(*p.qp).boxUpper() == Eigen::Vector2d::Constant(kBox / 2));

  p.qp->setBoxSize(Eigen::Vector2d::Constant(0.5));
  const QPSnapshot rebuilt = reducedAndChecked(*p.qp);
  EXPECT_EQ(rebuilt.rows(), 2);
  EXPECT_TRUE(rebuilt.boxUpper() == Eigen::Vector2d::Constant(0.5));
}

/** @brief A strictly convex problem has one minimizer, which the solver must find with and without the reduction */
TEST_F(QPRowReduction, SolutionMatchesTheFullQP)  // NOLINT
{
  auto solve = [](bool reduce) {
    TestProblem p;
    const Eigen::Vector2d a(1, 1);
    p.addConstraint({ upper({ 1, 0 }, -1), upper({ 0, 1 }, 5), upper(a, 0) });
    p.addCost({ upper(2 * a, 0, 2) }, trajopt_sqp::CostPenaltyType::kHinge);
    p.addCost({ { { 1, 0 }, trajopt_ifopt::Bounds(0.5, 0.5), 1 }, { { 0, 1 }, trajopt_ifopt::Bounds(0.3, 0.3), 1 } },
              trajopt_sqp::CostPenaltyType::kSquared);
    p.qp->setup();

    trajopt_sqp::TrustRegionSQPSolver solver(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
    solver.params.qp_row_reduction = reduce;
    solver.solve(p.qp);
    EXPECT_EQ(solver.getStatus(), trajopt_sqp::SQPStatus::kConverged);
    return std::make_pair(Eigen::Vector2d(solver.getResults().best_var_vals), p.qp->getNumQPVars());
  };

  const auto [x_full, n_full] = solve(false);
  const auto [x_reduced, n_reduced] = solve(true);
  EXPECT_EQ(n_full, 6);
  EXPECT_LT(n_reduced, n_full);
  EXPECT_NEAR(x_full(0), -1, 1e-4);
  EXPECT_NEAR(x_full(1), 0.3, 1e-4);
  EXPECT_NEAR(x_reduced(0), x_full(0), 1e-6);
  EXPECT_NEAR(x_reduced(1), x_full(1), 1e-6);
}

/**
 * @brief A solver that solved a reduced QP holds its dimensions; after setup() the problem reports the full ones again,
 * and the next solve must not hand the full QP to a solver still sized for the reduced one
 */
TEST_F(QPRowReduction, SolverReuseAfterSetupStartsFromTheNewDimensions)  // NOLINT
{
  TestProblem p;
  p.addConstraint({ upper({ 1, 0 }, -1), upper({ 0, 1 }, 5) });
  p.addCost({ { { 1, 0 }, trajopt_ifopt::Bounds(0.5, 0.5), 1 }, { { 0, 1 }, trajopt_ifopt::Bounds(0.3, 0.3), 1 } },
            trajopt_sqp::CostPenaltyType::kSquared);
  p.qp->setup();

  auto qp_solver = std::make_shared<DimensionCheckingSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  for (const bool reduce : { true, false })
  {
    solver.params.qp_row_reduction = reduce;
    solver.solve(p.qp);
    EXPECT_EQ(solver.getStatus(), trajopt_sqp::SQPStatus::kConverged);
    EXPECT_NEAR(solver.getResults().best_var_vals(0), -1, 1e-4);
    EXPECT_NEAR(solver.getResults().best_var_vals(1), 0.3, 1e-4);
    EXPECT_FALSE(qp_solver->mismatched) << "reduction " << reduce;

    const Eigen::Vector2d start = Eigen::Vector2d::Zero();
    p.qp->setVariables(start.data());
    p.qp->setup();
  }
}

/**
 * @brief More merit sets than reduced QP constraint rows, one of them kept with its slack: the OSQP warm start, which
 * walks one row per merit set to seed the slacks, has fewer rows to walk than sets
 */
TEST_F(QPRowReduction, WarmStartStaysInsideTheReducedQP)  // NOLINT
{
  TestProblem p;
  for (int i = 0; i < 5; ++i)
    p.addConstraint({ upper({ 1, 0 }, -1) });
  p.addConstraint({ upper({ 0, 1 }, 0.05) });
  p.addCost({ { { 1, 0 }, trajopt_ifopt::Bounds(0, 0), 1 }, { { 0, 1 }, trajopt_ifopt::Bounds(0, 0), 1 } },
            trajopt_sqp::CostPenaltyType::kSquared);
  p.qp->setup();

  // 6 merit sets, 5 with their row folded at x = 0: one constraint row above the three variable rows
  const QPSnapshot reduced = convexified(*p.qp, true);
  ASSERT_EQ(p.qp->getNumNLPConstraints(), 6);
  ASSERT_EQ(reduced.rows(), 1);
  ASSERT_EQ(reduced.num_vars, 3);

  trajopt_sqp::TrustRegionSQPSolver solver(std::make_shared<trajopt_sqp::OSQPEigenSolver>());
  solver.params.qp_row_reduction = true;
  solver.solve(p.qp);
  EXPECT_EQ(solver.getStatus(), trajopt_sqp::SQPStatus::kConverged);
  EXPECT_NEAR(solver.getResults().best_var_vals(0), -1, 1e-4);
  EXPECT_NEAR(solver.getResults().best_var_vals(1), 0, 1e-4);
}
