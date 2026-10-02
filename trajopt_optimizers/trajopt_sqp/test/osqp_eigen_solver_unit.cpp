/**
 * @file osqp_eigen_solver_unit.cpp
 * @brief Tests how OSQPEigenSolver takes a warm start and new QP data
 *
 * @author Roelof Oomen
 * @date September 24, 2026
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
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/eigen_types.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/trajopt_qp_problem.h>

using trajopt_sqp::OSQPEigenSolver;
using trajopt_sqp::QPSolverStatus;

namespace
{
/** @brief Two variables in [-1, 1] at @p start, pulled toward (0.8, -0.3) by a squared cost weighted by @p coeffs */
std::shared_ptr<trajopt_sqp::TrajOptQPProblem>
makeTargetProblem(const Eigen::Vector2d& start, const Eigen::VectorXd& coeffs = Eigen::VectorXd::Ones(1))
{
  auto node = std::make_unique<trajopt_ifopt::Node>("Joints");
  const std::vector<std::string> names{ "j0", "j1" };
  const std::vector<trajopt_ifopt::Bounds> bounds(2, trajopt_ifopt::Bounds(-1.0, 1.0));
  const std::shared_ptr<const trajopt_ifopt::Var> var = node->addVar("position", names, start, bounds);
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  nodes.push_back(std::move(node));
  auto qp = std::make_shared<trajopt_sqp::TrajOptQPProblem>(
      std::make_shared<trajopt_ifopt::NodesVariables>("trajectory", std::move(nodes)));
  qp->addCostSet(std::make_shared<trajopt_ifopt::JointPosConstraint>(Eigen::Vector2d(0.8, -0.3), var, coeffs, "Target"),
                 trajopt_sqp::CostPenaltyType::kSquared);
  qp->setup();
  return qp;
}

/** @brief Hand the convexified @p qp to @p solver from scratch, the way stepSQPSolver sets a solver up */
void loadQP(OSQPEigenSolver& solver, const trajopt_sqp::QPProblem& qp)
{
  ASSERT_TRUE(solver.clear());
  ASSERT_TRUE(solver.init(qp.getNumQPVars(), qp.getNumQPConstraints()));
  ASSERT_TRUE(solver.updateHessianMatrix(qp.getHessian()));
  ASSERT_TRUE(solver.updateGradient(qp.getGradient()));
  ASSERT_TRUE(solver.updateLinearConstraintsMatrix(qp.getConstraintMatrix()));
  ASSERT_TRUE(solver.updateBounds(qp.getBoundsLower(), qp.getBoundsUpper()));
}

/** @brief Hand a set-up @p solver the vectors of @p qp, whose matrices must equal those it was set up with */
void loadQPVectors(OSQPEigenSolver& solver, const trajopt_sqp::QPProblem& qp)
{
  ASSERT_TRUE(solver.updateGradient(qp.getGradient()));
  ASSERT_TRUE(solver.updateBounds(qp.getBoundsLower(), qp.getBoundsUpper()));
}

/** @brief A solver that checks termination after every ADMM iteration, so an exact start ends after one */
void checkEveryIteration(OSQPEigenSolver& solver)
{
  solver.solver_->settings()->setCheckTermination(1);
  solver.solver_->settings()->setPolish(false);
}

long lastIterations(const OSQPEigenSolver& solver) { return solver.solver_->solver()->info->iter; }

/** @brief A @p rows x @p cols matrix holding @p entries, zero values included */
trajopt_ifopt::Jacobian makeMatrix(Eigen::Index rows,
                                   Eigen::Index cols,
                                   const std::vector<Eigen::Triplet<double>>& entries)
{
  trajopt_ifopt::Jacobian matrix(rows, cols);
  matrix.setFromTriplets(entries.begin(), entries.end());
  return matrix;
}

/** @brief Two variables whose first constraint row, x0 + x1 <= 0.2, is active at the optimum */
struct SmallQP
{
  trajopt_ifopt::Jacobian hessian{ makeMatrix(2, 2, { { 0, 0, 4.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 1, 1, 2.0 } }) };
  Eigen::VectorXd gradient{ Eigen::Vector2d(-1.0, -1.0) };
  trajopt_ifopt::Jacobian constraints{
    makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 2, 1, 1.0 } })
  };
  Eigen::VectorXd lower{ Eigen::Vector3d(-1.0, -1.0, -1.0) };
  Eigen::VectorXd upper{ Eigen::Vector3d(0.2, 1.0, 1.0) };
};

/** @brief Hand @p qp to @p solver from scratch */
void loadQP(OSQPEigenSolver& solver, const SmallQP& qp)
{
  ASSERT_TRUE(solver.clear());
  ASSERT_TRUE(solver.init(qp.gradient.size(), qp.lower.size()));
  ASSERT_TRUE(solver.updateHessianMatrix(qp.hessian));
  ASSERT_TRUE(solver.updateGradient(qp.gradient));
  ASSERT_TRUE(solver.updateLinearConstraintsMatrix(qp.constraints));
  ASSERT_TRUE(solver.updateBounds(qp.lower, qp.upper));
}

/** @brief A solver that stops only close to the exact optimum, so solvers set up from different data agree tightly */
void solveTightly(OSQPEigenSolver& solver)
{
  solver.solver_->settings()->setAbsoluteTolerance(1e-10);
  solver.solver_->settings()->setRelativeTolerance(1e-10);
}

/** @brief The solution of a solver set up from @p qp alone */
Eigen::VectorXd freshSolution(const SmallQP& qp)
{
  OSQPEigenSolver solver;
  solveTightly(solver);
  loadQP(solver, qp);
  EXPECT_TRUE(solver.solve());
  return solver.getSolution();
}

/** @brief Mark the workspace of a set-up @p solver; false when OSQP does not record setup times */
bool markSetUp(OSQPEigenSolver& solver)
{
  // OSQP writes setup_time only while it sets a workspace up
  OSQPFloat& setup_time = solver.solver_->solver()->info->setup_time;
  if (setup_time <= 0.0)
    return false;
  setup_time = -1.0;
  return true;
}

/** @brief Whether OSQP set up a new workspace since markSetUp() */
bool setUpAgain(const OSQPEigenSolver& solver) { return solver.solver_->solver()->info->setup_time != -1.0; }
}  // namespace

// A solver being set up starts from the seed: at an optimum without active rows it terminates after one iteration.
TEST(OSQPEigenSolverUnit, SeedStartsASolverBeingSetUp)  // NOLINT
{
  auto qp = makeTargetProblem(Eigen::Vector2d(0.8, -0.3));  // the optimum is the linearization point
  qp->convexify();
  OSQPEigenSolver solver;
  checkEveryIteration(solver);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, *qp));
  ASSERT_TRUE(solver.setWarmStart(*qp));
  ASSERT_TRUE(solver.solve());
  EXPECT_EQ(lastIterations(solver), 1);
}

// A set-up solver given new vectors starts from the new seed with zero duals, not from the primal and dual iterate it
// kept.
TEST(OSQPEigenSolverUnit, SeedStartsASetUpSolver)  // NOLINT
{
  auto qp = makeTargetProblem(Eigen::Vector2d(0.0, 0.0));  // the box keeps the first solution short of the target
  qp->convexify();
  const Eigen::MatrixXd hessian = qp->getHessian();
  const Eigen::MatrixXd constraints = qp->getConstraintMatrix();
  OSQPEigenSolver solver;
  checkEveryIteration(solver);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, *qp));
  ASSERT_TRUE(solver.setWarmStart(*qp));
  ASSERT_TRUE(solver.solve());

  const Eigen::Vector2d target(0.8, -0.3);
  qp->setVariables(target.data());
  qp->convexify();
  ASSERT_EQ(qp->getNumQPVars(), 2);
  ASSERT_EQ(Eigen::MatrixXd(qp->getHessian()), hessian);
  ASSERT_EQ(Eigen::MatrixXd(qp->getConstraintMatrix()), constraints);
  ASSERT_NO_FATAL_FAILURE(loadQPVectors(solver, *qp));
  ASSERT_TRUE(solver.setWarmStart(*qp));
  ASSERT_TRUE(solver.solve());
  EXPECT_EQ(lastIterations(solver), 1);
  EXPECT_NEAR((solver.getSolution() - target).norm(), 0.0, 1e-6);
}

// With warm starting off, every solve starts cold, even after a seed was given.
TEST(OSQPEigenSolverUnit, WarmStartingOffKeepsTheSolveCold)  // NOLINT
{
  auto qp = makeTargetProblem(Eigen::Vector2d(0.8, -0.3));
  qp->convexify();
  OSQPEigenSolver solver;
  checkEveryIteration(solver);
  solver.solver_->settings()->setWarmStart(false);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, *qp));
  ASSERT_TRUE(solver.setWarmStart(*qp));
  ASSERT_TRUE(solver.solve());
  EXPECT_GT(lastIterations(solver), 1);

  qp->convexify();
  ASSERT_NO_FATAL_FAILURE(loadQPVectors(solver, *qp));
  ASSERT_TRUE(solver.setWarmStart(*qp));
  ASSERT_TRUE(solver.solve());
  EXPECT_GT(lastIterations(solver), 1);
}

// A set-up solver refuses a matrix with another pattern instead of setting OSQP up again behind its caller's back, and
// still takes new vectors.
TEST(OSQPEigenSolverUnit, PatternChangeDoesNotSetUpAgain)  // NOLINT
{
  auto qp = makeTargetProblem(Eigen::Vector2d(0.0, 0.0));
  qp->convexify();
  OSQPEigenSolver solver;
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, *qp));
  ASSERT_TRUE(solver.solve());
  const long iterations = lastIterations(solver);
  ASSERT_GT(iterations, 0);

  // Couple the two variables in the cost and in the first constraint row
  trajopt_ifopt::Jacobian hessian = qp->getHessian();
  hessian.coeffRef(0, 1) = 1.0;
  hessian.coeffRef(1, 0) = 1.0;
  trajopt_ifopt::Jacobian constraints = qp->getConstraintMatrix();
  constraints.coeffRef(0, 1) = 1.0;
  ASSERT_GT(hessian.nonZeros(), qp->getHessian().nonZeros());
  ASSERT_GT(constraints.nonZeros(), qp->getConstraintMatrix().nonZeros());

  EXPECT_FALSE(solver.updateHessianMatrix(hessian));
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(constraints));
  // Setting OSQP up again would reset the statistics of the last solve
  EXPECT_EQ(lastIterations(solver), iterations);
  EXPECT_TRUE(solver.updateGradient(qp->getGradient()));
  EXPECT_TRUE(solver.updateBounds(qp->getBoundsLower(), qp->getBoundsUpper()));
}

// A refused matrix update leaves warm starting off, and so does loading the new matrix from scratch.
TEST(OSQPEigenSolverUnit, RefusedMatrixUpdateKeepsWarmStartingOff)  // NOLINT
{
  trajopt_ifopt::Jacobian hessian(2, 2);
  hessian.insert(0, 0) = 1.0;
  hessian.insert(1, 1) = 1.0;
  trajopt_ifopt::Jacobian diagonal(2, 2);  // x0 and x1 each in a row of their own
  diagonal.insert(0, 0) = 1.0;
  diagonal.insert(1, 1) = 1.0;
  trajopt_ifopt::Jacobian coupled(2, 2);  // same dimensions, one more stored entry
  coupled.insert(0, 0) = 1.0;
  coupled.insert(0, 1) = 1.0;
  coupled.insert(1, 1) = 1.0;
  const Eigen::Vector2d gradient(-0.8, 0.3);
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(2, -1.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(2, 1.0);
  const auto load = [&](OSQPEigenSolver& solver, const trajopt_ifopt::Jacobian& constraints) {
    ASSERT_TRUE(solver.clear());
    ASSERT_TRUE(solver.init(2, 2));
    ASSERT_TRUE(solver.updateHessianMatrix(hessian));
    ASSERT_TRUE(solver.updateGradient(gradient));
    ASSERT_TRUE(solver.updateLinearConstraintsMatrix(constraints));
    ASSERT_TRUE(solver.updateBounds(lower, upper));
  };
  const auto warmStarting = [](const OSQPEigenSolver& solver) {
    return solver.solver_->solver()->settings->warm_starting;
  };

  OSQPEigenSolver solver;
  checkEveryIteration(solver);
  solver.solver_->settings()->setWarmStart(false);
  ASSERT_NO_FATAL_FAILURE(load(solver, diagonal));
  ASSERT_TRUE(solver.solve());
  EXPECT_EQ(warmStarting(solver), 0);
  EXPECT_GT(lastIterations(solver), 1);

  // The way the trust-region solver takes a new matrix: try in place, then load from scratch when refused
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(coupled));
  EXPECT_EQ(warmStarting(solver), 0);
  ASSERT_NO_FATAL_FAILURE(load(solver, coupled));
  ASSERT_TRUE(solver.solve());
  EXPECT_EQ(warmStarting(solver), 0);
  EXPECT_GT(lastIterations(solver), 1);

  // A bounds-only update still starts cold
  ASSERT_TRUE(solver.updateBounds(lower, upper));
  ASSERT_TRUE(solver.solve());
  EXPECT_GT(lastIterations(solver), 1);
}

// The one-sided bound updates load a solver being set up, as updateBounds() does.
TEST(OSQPEigenSolverUnit, OneSidedBoundUpdatesLoadASolverBeingSetUp)  // NOLINT
{
  // Minimize x^2 - 3x subject to -1 <= x <= 1: the upper bound holds the optimum at 1
  trajopt_ifopt::Jacobian A(1, 1);
  A.insert(0, 0) = 1.0;
  trajopt_ifopt::Jacobian hessian(1, 1);
  hessian.insert(0, 0) = 1.0;
  OSQPEigenSolver solver;
  ASSERT_TRUE(solver.init(1, 1));
  ASSERT_TRUE(solver.updateHessianMatrix(hessian));
  ASSERT_TRUE(solver.updateGradient(Eigen::VectorXd::Constant(1, -3.0)));
  ASSERT_TRUE(solver.updateLinearConstraintsMatrix(A));
  EXPECT_TRUE(solver.updateLowerBound(Eigen::VectorXd::Constant(1, -1.0)));
  EXPECT_TRUE(solver.updateUpperBound(Eigen::VectorXd::Constant(1, 1.0)));
  ASSERT_TRUE(solver.solve());
  EXPECT_NEAR(solver.getSolution()[0], 1.0, 1e-3);
}

// A set-up solver takes new matrix values with an unchanged pattern in place and solves as a solver set up from them.
TEST(OSQPEigenSolverUnit, InPlaceUpdateMatchesAFreshSetup)  // NOLINT
{
  const SmallQP qp;
  OSQPEigenSolver solver;
  solveTightly(solver);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
  ASSERT_TRUE(solver.solve());
  const Eigen::VectorXd first = solver.getSolution();
  if (!markSetUp(solver))
    GTEST_SKIP() << "OSQP does not record setup times";

  SmallQP updated;
  updated.hessian = makeMatrix(2, 2, { { 0, 0, 2.0 }, { 0, 1, -0.5 }, { 1, 0, -0.5 }, { 1, 1, 3.0 } });
  updated.gradient = Eigen::Vector2d(-2.0, 1.0);
  updated.constraints = makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 2.0 }, { 1, 0, 0.5 }, { 2, 1, 1.0 } });
  ASSERT_TRUE(solver.updateHessianMatrix(updated.hessian));
  ASSERT_TRUE(solver.updateGradient(updated.gradient));
  ASSERT_TRUE(solver.updateLinearConstraintsMatrix(updated.constraints));
  ASSERT_TRUE(solver.updateBounds(updated.lower, updated.upper));
  ASSERT_TRUE(solver.solve());
  EXPECT_FALSE(setUpAgain(solver));

  const Eigen::VectorXd expected = freshSolution(updated);
  EXPECT_GT((expected - first).norm(), 0.1) << "the new values must move the optimum";
  EXPECT_NEAR((solver.getSolution() - expected).norm(), 0.0, 1e-7);
}

// A RowMajor matrix stores its entries in another order than OSQP's column-ordered copy; its values still land on the
// entries they belong to.
TEST(OSQPEigenSolverUnit, RowMajorPatternIsRecognised)  // NOLINT
{
  const SmallQP qp;
  SmallQP updated = qp;
  updated.constraints = makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 2.0 }, { 1, 0, 0.5 }, { 2, 1, 1.0 } });
  const Eigen::SparseMatrix<double, Eigen::ColMajor> column_major = updated.constraints;
  const auto values = [](const auto& matrix) {
    return std::vector<double>(matrix.valuePtr(), matrix.valuePtr() + matrix.nonZeros());
  };
  ASSERT_NE(values(updated.constraints), values(column_major));

  OSQPEigenSolver solver;
  solveTightly(solver);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
  ASSERT_TRUE(solver.solve());
  if (!markSetUp(solver))
    GTEST_SKIP() << "OSQP does not record setup times";

  ASSERT_TRUE(solver.updateLinearConstraintsMatrix(updated.constraints));
  ASSERT_TRUE(solver.solve());
  EXPECT_FALSE(setUpAgain(solver));
  EXPECT_NEAR((solver.getSolution() - freshSolution(updated)).norm(), 0.0, 1e-7);
}

// A set-up solver refuses a matrix whose pattern differs from the one it was set up with, even at an equal entry
// count, and still takes the pattern it was set up with.
TEST(OSQPEigenSolverUnit, PatternChangeIsRefused)  // NOLINT
{
  const SmallQP qp;
  OSQPEigenSolver solver;
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
  ASSERT_TRUE(solver.solve());

  const trajopt_ifopt::Jacobian diagonal = makeMatrix(2, 2, { { 0, 0, 4.0 }, { 1, 1, 2.0 } });
  // Entry (1, 0) moved to (2, 0): the same entries per column
  const trajopt_ifopt::Jacobian moved_row =
      makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 2, 0, 1.0 }, { 2, 1, 1.0 } });
  // Entry (1, 0) moved to (1, 1): another column
  const trajopt_ifopt::Jacobian moved_column =
      makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 1, 1.0 }, { 2, 1, 1.0 } });
  const trajopt_ifopt::Jacobian extra =
      makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 1, 1, 1.0 }, { 2, 1, 1.0 } });
  ASSERT_EQ(moved_row.nonZeros(), qp.constraints.nonZeros());
  ASSERT_EQ(moved_column.nonZeros(), qp.constraints.nonZeros());

  EXPECT_FALSE(solver.updateHessianMatrix(diagonal));
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(moved_row));
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(moved_column));
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(extra));
  EXPECT_TRUE(solver.updateHessianMatrix(qp.hessian));
  EXPECT_TRUE(solver.updateLinearConstraintsMatrix(qp.constraints));
}

// A set-up solver refuses a constraint matrix with another number of rows.
TEST(OSQPEigenSolverUnit, RowCountChangeIsRefused)  // NOLINT
{
  const SmallQP qp;
  OSQPEigenSolver solver;
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
  ASSERT_TRUE(solver.solve());

  const trajopt_ifopt::Jacobian fewer = makeMatrix(2, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 } });
  const trajopt_ifopt::Jacobian more =
      makeMatrix(4, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 2, 1, 1.0 }, { 3, 1, 1.0 } });
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(fewer));
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(more));
}

// A stored zero is part of the pattern: a matrix that keeps it is taken in place, one that drops it is refused.
TEST(OSQPEigenSolverUnit, ExplicitZerosArePartOfThePattern)  // NOLINT
{
  const SmallQP qp;
  SmallQP zeroed = qp;
  zeroed.hessian = makeMatrix(2, 2, { { 0, 0, 4.0 }, { 0, 1, 0.0 }, { 1, 0, 0.0 }, { 1, 1, 2.0 } });
  zeroed.constraints = makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 0.0 }, { 2, 1, 1.0 } });
  ASSERT_EQ(zeroed.hessian.nonZeros(), qp.hessian.nonZeros());
  ASSERT_EQ(zeroed.constraints.nonZeros(), qp.constraints.nonZeros());
  const trajopt_ifopt::Jacobian dropped_hessian = zeroed.hessian.pruned();
  const trajopt_ifopt::Jacobian dropped_constraints = zeroed.constraints.pruned();
  ASSERT_LT(dropped_hessian.nonZeros(), qp.hessian.nonZeros());
  ASSERT_LT(dropped_constraints.nonZeros(), qp.constraints.nonZeros());

  OSQPEigenSolver solver;
  solveTightly(solver);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
  ASSERT_TRUE(solver.solve());
  if (!markSetUp(solver))
    GTEST_SKIP() << "OSQP does not record setup times";

  EXPECT_TRUE(solver.updateHessianMatrix(zeroed.hessian));
  EXPECT_TRUE(solver.updateLinearConstraintsMatrix(zeroed.constraints));
  ASSERT_TRUE(solver.solve());
  EXPECT_FALSE(setUpAgain(solver));
  EXPECT_NEAR((solver.getSolution() - freshSolution(zeroed)).norm(), 0.0, 1e-7);

  EXPECT_FALSE(solver.updateHessianMatrix(dropped_hessian));
  EXPECT_FALSE(solver.updateLinearConstraintsMatrix(dropped_constraints));
}

namespace
{
/** @brief SmallQP's patterns with values that move its optimum */
SmallQP makeReweightedQP()
{
  SmallQP qp;
  qp.hessian = makeMatrix(2, 2, { { 0, 0, 2.0 }, { 0, 1, -0.5 }, { 1, 0, -0.5 }, { 1, 1, 3.0 } });
  qp.constraints = makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 2.0 }, { 1, 0, 0.5 }, { 2, 1, 1.0 } });
  return qp;
}
}  // namespace

// A refused matrix update drops the values taken in place before it: they reach OSQP neither on the next solve nor
// after the caller sets the solver up again.
TEST(OSQPEigenSolverUnit, RefusedMatrixUpdateDropsThePendingValues)  // NOLINT
{
  const SmallQP qp;
  const SmallQP reweighted = makeReweightedQP();
  const Eigen::VectorXd expected = freshSolution(qp);
  ASSERT_GT((freshSolution(reweighted) - expected).norm(), 0.01) << "the pending values must move the optimum";
  const trajopt_ifopt::Jacobian moved =
      makeMatrix(3, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 2, 0, 1.0 }, { 2, 1, 1.0 } });

  for (const bool set_up_again : { false, true })
  {
    OSQPEigenSolver solver;
    solveTightly(solver);
    ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
    ASSERT_TRUE(solver.solve());
    ASSERT_TRUE(solver.updateHessianMatrix(reweighted.hessian));
    ASSERT_FALSE(solver.updateLinearConstraintsMatrix(moved));
    if (set_up_again)
    {
      ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
      ASSERT_TRUE(solver.solve());
    }
    // A set-up solver hands OSQP whatever is pending at its next solve
    ASSERT_TRUE(solver.solve());
    EXPECT_NEAR((solver.getSolution() - expected).norm(), 0.0, 1e-7) << "set up again: " << set_up_again;
  }
}

// clear() drops the values taken in place: they do not reach the solver set up after it.
TEST(OSQPEigenSolverUnit, ClearDropsThePendingValues)  // NOLINT
{
  const SmallQP qp;
  const SmallQP reweighted = makeReweightedQP();
  const Eigen::VectorXd expected = freshSolution(qp);
  ASSERT_GT((freshSolution(reweighted) - expected).norm(), 0.01) << "the pending values must move the optimum";

  OSQPEigenSolver solver;
  solveTightly(solver);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
  ASSERT_TRUE(solver.solve());
  ASSERT_TRUE(solver.updateHessianMatrix(reweighted.hessian));
  ASSERT_TRUE(solver.updateLinearConstraintsMatrix(reweighted.constraints));
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
  ASSERT_TRUE(solver.solve());
  // A set-up solver hands OSQP whatever is pending at its next solve
  ASSERT_TRUE(solver.solve());
  EXPECT_NEAR((solver.getSolution() - expected).norm(), 0.0, 1e-7);
}

// A seed given after an in-place matrix update is scaled as the new data is: a seed at the new optimum terminates at
// the first check.
TEST(OSQPEigenSolverUnit, SeedAfterAnInPlaceUpdateUsesTheNewScaling)  // NOLINT
{
  auto qp = makeTargetProblem(Eigen::Vector2d(0.0, 0.0));
  qp->convexify();
  OSQPEigenSolver solver;
  checkEveryIteration(solver);
  ASSERT_NO_FATAL_FAILURE(loadQP(solver, *qp));
  ASSERT_TRUE(solver.setWarmStart(*qp));
  ASSERT_TRUE(solver.solve());

  // Weights this unequal rescale the two variables differently
  const Eigen::Vector2d target(0.8, -0.3);
  auto reweighted = makeTargetProblem(target, Eigen::Vector2d(50.0, 0.02));
  reweighted->convexify();
  ASSERT_TRUE(solver.updateHessianMatrix(reweighted->getHessian()));
  ASSERT_TRUE(solver.updateGradient(reweighted->getGradient()));
  ASSERT_TRUE(solver.updateLinearConstraintsMatrix(reweighted->getConstraintMatrix()));
  ASSERT_TRUE(solver.updateBounds(reweighted->getBoundsLower(), reweighted->getBoundsUpper()));
  ASSERT_TRUE(solver.setWarmStart(*reweighted));
  ASSERT_TRUE(solver.solve());
  EXPECT_EQ(lastIterations(solver), 1);
  EXPECT_NEAR((solver.getSolution() - target).norm(), 0.0, 1e-6);
}

TEST(OSQPEigenSolverUnit, SuccessfulSolveClearsFailure)  // NOLINT
{
  // Minimize x'x subject to x0 + x1 >= 3 and x0 + x1 <= 1: infeasible until the lower bound is dropped
  constexpr double inf = std::numeric_limits<double>::infinity();
  const std::vector<Eigen::Triplet<double>> triplets{ { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 1, 1, 1.0 } };
  trajopt_ifopt::Jacobian A(2, 2);
  A.setFromTriplets(triplets.begin(), triplets.end());
  trajopt_ifopt::Jacobian hessian(2, 2);
  hessian.setIdentity();

  OSQPEigenSolver solver;
  solver.init(2, 2);
  solver.updateHessianMatrix(hessian);
  solver.updateGradient(Eigen::Vector2d::Zero());
  solver.updateLinearConstraintsMatrix(A);
  solver.updateBounds(Eigen::Vector2d(3.0, -inf), Eigen::Vector2d(inf, 1.0));
  EXPECT_FALSE(solver.solve());
  EXPECT_EQ(solver.getSolverStatus(), QPSolverStatus::kFailed);

  solver.updateBounds(Eigen::Vector2d(-inf, -inf), Eigen::Vector2d(inf, 1.0));
  ASSERT_TRUE(solver.solve());
  EXPECT_EQ(solver.getSolverStatus(), QPSolverStatus::kInitialized);
}

TEST(OSQPEigenSolverUnit, SmallGradientEntriesReachTheSolver)  // NOLINT
{
  // The solver takes the QP as given: a gradient entry of 5e-8 is not dropped
  OSQPEigenSolver solver;
  trajopt_ifopt::Jacobian A(1, 1);
  A.insert(0, 0) = 1.0;
  trajopt_ifopt::Jacobian hessian(1, 1);
  hessian.insert(0, 0) = 1.0;
  solver.init(1, 1);
  solver.updateHessianMatrix(hessian);
  solver.updateGradient(Eigen::VectorXd::Constant(1, 5e-8));
  solver.updateLinearConstraintsMatrix(A);
  solver.updateBounds(Eigen::VectorXd::Constant(1, -1.0), Eigen::VectorXd::Constant(1, 1.0));
  EXPECT_DOUBLE_EQ(solver.solver_->data()->getData()->q[0], 5e-8);
}

namespace
{
/** @brief A solver that checks termination after its last iteration only, adapts no rho and does not polish */
void checkOnlyAfter(OSQPEigenSolver& solver, int iterations)
{
  solver.solver_->settings()->setCheckTermination(0);
  solver.solver_->settings()->setAdaptiveRho(false);
  solver.solver_->settings()->setPolish(false);
  solver.solver_->settings()->setMaxIteration(iterations);
}

/** @brief Two uncoupled variables pulled toward 0.5, with 0 <= scale x0 <= 1e-3 and -1 <= x1 <= 1 */
SmallQP makeNarrowRowQP(double scale)
{
  SmallQP qp;
  qp.hessian = makeMatrix(2, 2, { { 0, 0, 1.0 }, { 1, 1, 1.0 } });
  qp.constraints = makeMatrix(2, 2, { { 0, 0, scale }, { 1, 1, 1.0 } });
  qp.lower = Eigen::Vector2d(0.0, -1.0);
  qp.upper = Eigen::Vector2d(1e-3, 1.0);
  return qp;
}

/** @brief A solver that starts every solve cold and ends it, Solved, after exactly @p iterations */
void iterateExactly(OSQPEigenSolver& solver, int iterations)
{
  checkOnlyAfter(solver, iterations);
  solver.solver_->settings()->setWarmStart(false);
  solver.solver_->settings()->setAbsoluteTolerance(1e6);
  solver.solver_->settings()->setRelativeTolerance(1e6);
}
}  // namespace

// OSQP gives a row whose scaled bounds nearly meet the rho of an equality. A bounds update after an in-place matrix
// update classifies the rows by the new matrices' scaling, so the solver iterates as one set up from the new data.
TEST(OSQPEigenSolverUnit, BoundsUpdateClassifiesRowsByTheNewScaling)  // NOLINT
{
  // Scaling row 0 up ten thousandfold scales its bounds' gap down about a hundredfold, across OSQP's equality tolerance
  const SmallQP old_qp = makeNarrowRowQP(1.0);
  const SmallQP new_qp = makeNarrowRowQP(1e4);
  constexpr int iterations = 10;

  OSQPEigenSolver fresh;
  iterateExactly(fresh, iterations);
  ASSERT_NO_FATAL_FAILURE(loadQP(fresh, new_qp));
  ASSERT_TRUE(fresh.solve());
  const Eigen::VectorXd expected = fresh.getSolution();

  const std::vector<std::pair<std::string, std::function<bool(OSQPEigenSolver&)>>> updates{
    { "updateBounds", [&](OSQPEigenSolver& s) { return s.updateBounds(new_qp.lower, new_qp.upper); } },
    { "updateLowerBound", [&](OSQPEigenSolver& s) { return s.updateLowerBound(new_qp.lower); } },
    { "updateUpperBound", [&](OSQPEigenSolver& s) { return s.updateUpperBound(new_qp.upper); } },
  };
  for (const auto& [name, update] : updates)
  {
    OSQPEigenSolver solver;
    iterateExactly(solver, iterations);
    ASSERT_NO_FATAL_FAILURE(loadQP(solver, old_qp));
    ASSERT_TRUE(solver.solve());
    ASSERT_TRUE(solver.updateHessianMatrix(new_qp.hessian));
    ASSERT_TRUE(solver.updateGradient(new_qp.gradient));
    ASSERT_TRUE(solver.updateLinearConstraintsMatrix(new_qp.constraints));
    ASSERT_TRUE(update(solver)) << name;
    ASSERT_TRUE(solver.solve()) << name;
    EXPECT_NEAR((solver.getSolution() - expected).norm(), 0.0, 1e-9) << name;
  }
}

// A bounds update fails when OSQP rejects the matrix values pending before it: a Hessian that leaves the KKT matrix not
// quasidefinite fails the refactorization.
TEST(OSQPEigenSolverUnit, BoundsUpdateFailsOnRejectedPendingValues)  // NOLINT
{
  const SmallQP qp;
  // SmallQP's pattern, explicit zero included, negative enough that the constraint rows' rho does not mask it
  const trajopt_ifopt::Jacobian indefinite =
      makeMatrix(2, 2, { { 0, 0, -4.0 }, { 0, 1, 0.0 }, { 1, 0, 0.0 }, { 1, 1, 2.0 } });

  const std::vector<std::pair<std::string, std::function<bool(OSQPEigenSolver&)>>> updates{
    { "updateBounds", [&](OSQPEigenSolver& s) { return s.updateBounds(qp.lower, qp.upper); } },
    { "updateLowerBound", [&](OSQPEigenSolver& s) { return s.updateLowerBound(qp.lower); } },
    { "updateUpperBound", [&](OSQPEigenSolver& s) { return s.updateUpperBound(qp.upper); } },
  };
  for (const auto& [name, update] : updates)
  {
    OSQPEigenSolver solver;
    ASSERT_NO_FATAL_FAILURE(loadQP(solver, qp));
    ASSERT_TRUE(solver.solve());
    ASSERT_TRUE(solver.updateHessianMatrix(indefinite)) << name;
    EXPECT_FALSE(update(solver)) << name;
  }
}
