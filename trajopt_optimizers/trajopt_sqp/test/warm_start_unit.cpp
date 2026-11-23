/**
 * @file warm_start_unit.cpp
 * @brief Tests the QP start point computed at a linearization point, and when the SQP solver seeds the QP solver
 *
 * @author Roelof Oomen
 * @date October 1, 2026
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
#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/core/eigen_types.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/qp_solver.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/warm_start.h>

namespace
{
constexpr double inf = std::numeric_limits<double>::infinity();

/**
 * @brief Two NLP variables x0, x1 and five slacks s0..s4 (columns 2..6), in TrajOptQPProblem's column layout
 * @details TrajOptQPProblem's slack coefficients are +1 and -1; row 1's -2 checks that a slack is divided by its
 * coefficient. Rows:
 *   0: x0 + s0 >= 1             (lower-bound row)
 *   1: x1 - 2 s1 <= 0.5         (upper-bound row, slack coefficient -2)
 *   2: x0 + x1 + s2 - s3 = 2    (equality row)
 *   3: x0 + s4 >= -1            (satisfied lower-bound row)
 *   4: x0 in [-1, 0.1]          (bound row)
 *   5: x1 in [-1, 1]            (bound row)
 *   6..10: s_i >= 0             (slack bound rows)
 */
struct StartFixture
{
  trajopt_ifopt::Jacobian A{ 11, 7 };
  Eigen::VectorXd lower{ 11 };
  Eigen::VectorXd upper{ 11 };

  StartFixture()
  {
    const std::vector<Eigen::Triplet<double>> t{ { 0, 0, 1.0 }, { 0, 2, 1.0 }, { 1, 1, 1.0 }, { 1, 3, -2.0 },
                                                 { 2, 0, 1.0 }, { 2, 1, 1.0 }, { 2, 4, 1.0 }, { 2, 5, -1.0 },
                                                 { 3, 0, 1.0 }, { 3, 6, 1.0 }, { 4, 0, 1.0 }, { 5, 1, 1.0 },
                                                 { 6, 2, 1.0 }, { 7, 3, 1.0 }, { 8, 4, 1.0 }, { 9, 5, 1.0 },
                                                 { 10, 6, 1.0 } };
    A.setFromTriplets(t.begin(), t.end());
    lower << 1.0, -inf, 2.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    upper << inf, 0.5, 2.0, inf, 0.1, 1.0, inf, inf, inf, inf, inf;
  }

  Eigen::VectorXd at(const Eigen::Vector2d& x) const { return trajopt_sqp::withMinimalSlacks(A, lower, upper, x); }
};

/** @brief Forwards to an OSQPEigenSolver and records the calls made on it */
class RecordingQPSolver : public trajopt_sqp::QPSolver
{
public:
  /** @brief The QPSolver methods called on this solver, in order */
  std::vector<std::string> calls;
  /** @brief What each solve returned, in order */
  std::vector<bool> solve_results;
  /** @brief The number of first solves reported as failed, whatever the inner solver returned */
  int fail_first_solves{ 0 };
  /** @brief When set, setWarmStart rejects every seed without passing it on */
  bool reject_warm_start{ false };

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts) override
  {
    calls.emplace_back("init");
    return inner_.init(num_vars, num_cnts);
  }
  bool clear() override
  {
    calls.emplace_back("clear");
    return inner_.clear();
  }
  bool solve() override
  {
    calls.emplace_back("solve");
    const bool solved = inner_.solve() && static_cast<int>(solve_results.size()) >= fail_first_solves;
    solve_results.push_back(solved);
    return solved;
  }
  Eigen::VectorXd getSolution() override { return inner_.getSolution(); }
  bool updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian) override
  {
    calls.emplace_back("updateHessianMatrix");
    return inner_.updateHessianMatrix(hessian);
  }
  bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient) override
  {
    calls.emplace_back("updateGradient");
    return inner_.updateGradient(gradient);
  }
  bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lower) override
  {
    calls.emplace_back("updateLowerBound");
    return inner_.updateLowerBound(lower);
  }
  bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upper) override
  {
    calls.emplace_back("updateUpperBound");
    return inner_.updateUpperBound(upper);
  }
  bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lower,
                    const Eigen::Ref<const Eigen::VectorXd>& upper) override
  {
    calls.emplace_back("updateBounds");
    return inner_.updateBounds(lower, upper);
  }
  bool updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& matrix) override
  {
    calls.emplace_back("updateLinearConstraintsMatrix");
    return inner_.updateLinearConstraintsMatrix(matrix);
  }
  bool setWarmStart(const trajopt_sqp::QPProblem& qp_problem) override
  {
    calls.emplace_back("setWarmStart");
    if (reject_warm_start)
      return false;
    return inner_.setWarmStart(qp_problem);
  }
  trajopt_sqp::QPSolverStatus getSolverStatus() const override { return inner_.getSolverStatus(); }

private:
  trajopt_sqp::OSQPEigenSolver inner_;
};

/** @brief Forwards to a real problem; can replace its exact costs by call index */
class CostHookQPProblem : public trajopt_sqp::QPProblem
{
public:
  explicit CostHookQPProblem(std::shared_ptr<trajopt_sqp::QPProblem> inner) : inner_(std::move(inner)) {}

  /** @brief Called with the 1-based getExactCosts call index and the inner costs; returns the costs to report */
  std::function<Eigen::VectorXd(int, const Eigen::VectorXd&)> exact_costs_hook;

  void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> c) override { inner_->addConstraintSet(c); }
  void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> c, trajopt_sqp::CostPenaltyType t) override
  {
    inner_->addCostSet(c, t);
  }
  void setup() override { inner_->setup(); }
  void setVariables(const double* x) override { inner_->setVariables(x); }
  Eigen::VectorXd getVariableValues() const override { return inner_->getVariableValues(); }
  void convexify() override { inner_->convexify(); }
  double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& v) const override
  {
    return inner_->evaluateTotalConvexCost(v);
  }
  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& v) const override
  {
    return inner_->evaluateConvexCosts(v);
  }
  double getTotalExactCost() const override { return getExactCosts().sum(); }
  Eigen::VectorXd getExactCosts() const override
  {
    Eigen::VectorXd costs = inner_->getExactCosts();
    ++exact_cost_calls_;
    return exact_costs_hook ? exact_costs_hook(exact_cost_calls_, costs) : costs;
  }
  trajopt_sqp::ConstraintViolations
  evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& v) const override
  {
    return inner_->evaluateConvexConstraintViolations(v);
  }
  trajopt_sqp::ConstraintViolations getExactConstraintViolations() const override
  {
    return inner_->getExactConstraintViolations();
  }
  void scaleBoxSize(double& scale) override { inner_->scaleBoxSize(scale); }
  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& b) override { inner_->setBoxSize(b); }
  void setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& c) override
  {
    inner_->setConstraintMeritCoeff(c);
  }
  void print() const override { inner_->print(); }
  Eigen::Index getNumNLPVars() const override { return inner_->getNumNLPVars(); }
  Eigen::Index getNumNLPConstraints() const override { return inner_->getNumNLPConstraints(); }
  Eigen::Index getNumNLPCosts() const override { return inner_->getNumNLPCosts(); }
  Eigen::Index getNumQPVars() const override { return inner_->getNumQPVars(); }
  Eigen::Index getNumQPConstraints() const override { return inner_->getNumQPConstraints(); }
  const std::vector<std::string>& getNLPConstraintNames() const override { return inner_->getNLPConstraintNames(); }
  const std::vector<std::string>& getNLPCostNames() const override { return inner_->getNLPCostNames(); }
  const Eigen::VectorXd& getBoxSize() const override { return inner_->getBoxSize(); }
  const Eigen::VectorXd& getConstraintMeritCoeff() const override { return inner_->getConstraintMeritCoeff(); }
  const trajopt_ifopt::Jacobian& getHessian() const override { return inner_->getHessian(); }
  const Eigen::VectorXd& getGradient() const override { return inner_->getGradient(); }
  const trajopt_ifopt::Jacobian& getConstraintMatrix() const override { return inner_->getConstraintMatrix(); }
  const Eigen::VectorXd& getBoundsLower() const override { return inner_->getBoundsLower(); }
  const Eigen::VectorXd& getBoundsUpper() const override { return inner_->getBoundsUpper(); }
  Eigen::VectorXd getNLPVariableBoundsLower() const override { return inner_->getNLPVariableBoundsLower(); }
  Eigen::VectorXd getNLPVariableBoundsUpper() const override { return inner_->getNLPVariableBoundsUpper(); }

private:
  std::shared_ptr<trajopt_sqp::QPProblem> inner_;
  mutable int exact_cost_calls_{ 0 };
};

/**
 * @brief One variable x in [-1, 1], starting at 0 and pulled toward 0.8 by a squared cost
 * @param constraint_target When set, adds the hard constraint x = constraint_target
 */
std::shared_ptr<trajopt_sqp::TrajOptQPProblem> makeProblem(std::optional<double> constraint_target = std::nullopt)
{
  auto node = std::make_unique<trajopt_ifopt::Node>("Joints");
  const std::shared_ptr<const trajopt_ifopt::Var> var =
      node->addVar("position", { "j0" }, Eigen::VectorXd::Zero(1), { trajopt_ifopt::Bounds(-1.0, 1.0) });
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  nodes.push_back(std::move(node));
  auto variables = std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes));

  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(variables);
  qp->addCostSet(std::make_shared<trajopt_ifopt::JointPosConstraint>(
                     Eigen::VectorXd::Constant(1, 0.8), var, Eigen::VectorXd::Ones(1), "Target"),
                 trajopt_sqp::CostPenaltyType::kSquared);
  if (constraint_target)
  {
    const std::vector<trajopt_ifopt::Bounds> cnt_bounds{ trajopt_ifopt::Bounds(*constraint_target,
                                                                               *constraint_target) };
    qp->addConstraintSet(
        std::make_shared<trajopt_ifopt::JointPosConstraint>(cnt_bounds, var, Eigen::VectorXd::Ones(1), "Pin"));
  }
  qp->setup();
  return qp;
}
}  // namespace

class TrustRegionSeeding : public testing::Test
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

// Each slack covers exactly the shortfall of its row at the given NLP point.
TEST(WithMinimalSlacks, SizesEachSlackToItsRowShortfall)  // NOLINT
{
  const StartFixture f;
  const Eigen::VectorXd x = f.at(Eigen::Vector2d(0.1, 0.9));
  ASSERT_EQ(x.size(), 7);
  EXPECT_DOUBLE_EQ(x(0), 0.1);
  EXPECT_DOUBLE_EQ(x(1), 0.9);
  EXPECT_DOUBLE_EQ(x(2), 0.9);  // 1 - 0.1
  EXPECT_DOUBLE_EQ(x(3), 0.2);  // (0.9 - 0.5) / 2
  EXPECT_DOUBLE_EQ(x(4), 1.0);  // 2 - (0.1 + 0.9)
  EXPECT_DOUBLE_EQ(x(5), 0.0);
  EXPECT_DOUBLE_EQ(x(6), 0.0);  // row 3 holds at the start
}

// The NLP values are taken as given: a bound row they violate has no slack and stays violated.
TEST(WithMinimalSlacks, LeavesTheNLPValuesUnchanged)  // NOLINT
{
  const StartFixture f;
  const Eigen::VectorXd x = f.at(Eigen::Vector2d(0.2, 0.9));
  EXPECT_DOUBLE_EQ(x(0), 0.2);  // outside row 4's [-1, 0.1]
  EXPECT_DOUBLE_EQ(x(1), 0.9);
  EXPECT_DOUBLE_EQ(x(2), 0.8);  // 1 - 0.2
}

// From an NLP point within the variable bounds, every row of the QP holds.
TEST(WithMinimalSlacks, SlacksMakeEveryRowFeasible)  // NOLINT
{
  const StartFixture f;
  for (const Eigen::Vector2d& start : { Eigen::Vector2d(0.1, 0.9), Eigen::Vector2d(-1.0, 1.0), Eigen::Vector2d(0, 0) })
  {
    const Eigen::VectorXd x = f.at(start);
    ASSERT_TRUE(x.allFinite());
    const Eigen::VectorXd ax = f.A * x;
    for (Eigen::Index r = 0; r < ax.size(); ++r)
    {
      EXPECT_GE(ax(r), f.lower(r) - 1e-12) << "row " << r << " start " << start.transpose();
      EXPECT_LE(ax(r), f.upper(r) + 1e-12) << "row " << r << " start " << start.transpose();
    }
  }

  // Above the lowered equality, x0 + x1 = 1.1: the negative slack takes the excess
  StartFixture g;
  g.lower(2) = 0.0;
  g.upper(2) = 0.0;
  const Eigen::VectorXd x = g.at(Eigen::Vector2d(0.1, 1.0));
  EXPECT_DOUBLE_EQ(x(4), 0.0);
  EXPECT_DOUBLE_EQ(x(5), 1.1);
}

// A constraint row holding a single slack entry sizes that slack like any other row.
TEST(WithMinimalSlacks, SlackOnlyRowsSizeTheirSlack)  // NOLINT
{
  // Columns: x0, s0. Rows: s0 >= bound, x0 in [-1, 1], s0 >= 0
  trajopt_ifopt::Jacobian A(3, 2);
  const std::vector<Eigen::Triplet<double>> t{ { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 2, 1, 1.0 } };
  A.setFromTriplets(t.begin(), t.end());
  Eigen::VectorXd upper(3);
  upper << inf, 1.0, inf;

  for (const double bound : { 0.3, 0.0 })
  {
    Eigen::VectorXd lower(3);
    lower << bound, -1.0, 0.0;
    const Eigen::VectorXd x = trajopt_sqp::withMinimalSlacks(A, lower, upper, Eigen::Vector<double, 1>(0.5));
    ASSERT_TRUE(x.allFinite());
    EXPECT_DOUBLE_EQ(x(0), 0.5) << "bound " << bound;
    EXPECT_DOUBLE_EQ(x(1), bound) << "bound " << bound;
  }
}

// The first convexification sets the QP solver up from the whole QP and later ones update it in place; each seeds its
// first solve after its last data update. A trust-region re-solve after a solve that did not fail keeps the solver's
// own iterate.
TEST_F(TrustRegionSeeding, EachConvexificationSeedsItsFirstSolveAfterItsLastUpdate)  // NOLINT
{
  auto recorder = std::make_shared<RecordingQPSolver>();
  auto problem = std::make_shared<CostHookQPProblem>(makeProblem(0.5));
  // Call 1 is init()'s evaluation of the start point; charging the first trial point rejects it and forces a
  // trust-region re-solve
  problem->exact_costs_hook = [](int call, const Eigen::VectorXd& costs) {
    return call == 2 ? Eigen::VectorXd(costs.array() + 1.0) : costs;
  };
  trajopt_sqp::TrustRegionSQPSolver solver(recorder);
  solver.solve(problem);

  int convexifications = 0;
  int re_solves = 0;
  int inits = 0;
  std::size_t solve_index = 0;
  bool after_failure = false;
  bool new_data = false;
  bool seeded = false;
  for (const std::string& call : recorder->calls)
  {
    if (call == "updateHessianMatrix" || call == "updateGradient" || call == "updateLinearConstraintsMatrix")
    {
      new_data = true;
      seeded = false;
    }
    else if (call == "updateBounds" || call == "updateLowerBound" || call == "updateUpperBound")
    {
      // A seed must follow the last update of any kind
      seeded = false;
    }
    else if (call == "init")
    {
      ++inits;
    }
    else if (call == "setWarmStart")
    {
      EXPECT_TRUE(new_data || after_failure) << "a re-solve after a usable solve was re-seeded";
      seeded = true;
    }
    else if (call == "solve")
    {
      if (new_data)
      {
        ++convexifications;
        EXPECT_TRUE(seeded) << "convexification " << convexifications << " solved without a seed after its last update";
      }
      else
      {
        ++re_solves;
      }
      after_failure = !recorder->solve_results.at(solve_index++);
      new_data = false;
    }
  }
  EXPECT_GT(convexifications, 1);
  EXPECT_GT(re_solves, 0) << "no trust-region re-solve occurred";
  EXPECT_EQ(inits, 1) << "convexifications after the first take the QP in place";
}

// A failed solve leaves no iterate worth continuing from: each re-solve after one starts from a fresh seed, both while
// the trust region shrinks and once it is set to its minimum.
TEST_F(TrustRegionSeeding, FailedSolveReseedsTheReSolve)  // NOLINT
{
  auto recorder = std::make_shared<RecordingQPSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(recorder);
  recorder->fail_first_solves = solver.params.max_qp_solver_failures;
  solver.solve(makeProblem());
  const auto failures = static_cast<std::size_t>(solver.params.max_qp_solver_failures);
  ASSERT_GT(recorder->solve_results.size(), failures) << "no solve followed the last failure";

  int re_solves_after_failure = 0;
  std::size_t solve_index = 0;
  bool after_failure = false;
  bool seeded = false;
  for (const std::string& call : recorder->calls)
  {
    if (call == "setWarmStart")
    {
      seeded = true;
    }
    else if (call == "updateBounds")
    {
      // The seed must follow the trust box it is clamped into
      seeded = false;
    }
    else if (call == "solve")
    {
      if (after_failure)
      {
        ++re_solves_after_failure;
        EXPECT_TRUE(seeded) << "solve " << solve_index + 1 << " continued from a failed solve's iterate";
      }
      after_failure = !recorder->solve_results.at(solve_index++);
      seeded = false;
    }
  }
  EXPECT_EQ(re_solves_after_failure, solver.params.max_qp_solver_failures);
}

// A solver that rejects every seed is reported once per run; later rejections in the same run log at debug level.
TEST_F(TrustRegionSeeding, RejectedSeedWarnsOncePerRun)  // NOLINT
{
  auto recorder = std::make_shared<RecordingQPSolver>();
  recorder->reject_warm_start = true;
  trajopt_sqp::TrustRegionSQPSolver solver(recorder);

  std::vector<spdlog::level::level_enum> levels;
  tesseract::common::getLogger()->set_level(spdlog::level::debug);
  const auto handler = tesseract::common::addLogRecordHandler([&levels](const tesseract::common::LogRecord& record) {
    if (record.message.find("rejected the warm start") != std::string::npos)
      levels.push_back(record.level);
  });
  solver.solve(makeProblem(0.5));
  solver.solve(makeProblem(0.5));
  tesseract::common::removeLogRecordHandler(handler);

  const auto count = [&levels](spdlog::level::level_enum level) {
    return std::count(levels.begin(), levels.end(), level);
  };
  ASSERT_GT(std::count(recorder->calls.begin(), recorder->calls.end(), "setWarmStart"), 2) << "too few seeds to tell";
  EXPECT_EQ(count(spdlog::level::warn), 2);
  EXPECT_EQ(count(spdlog::level::debug), static_cast<std::ptrdiff_t>(levels.size()) - 2);
  EXPECT_GT(count(spdlog::level::debug), 0);
}
