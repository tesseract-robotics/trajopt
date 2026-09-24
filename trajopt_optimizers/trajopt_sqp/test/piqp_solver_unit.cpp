/**
 * @file piqp_solver_unit.cpp
 * @brief Tests the row partition and status handling of PIQPSolver
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
#include <limits>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/eigen_types.h>
#include <trajopt_sqp/piqp_solver.h>

using trajopt_sqp::PIQPSolver;
using trajopt_sqp::QPSolverStatus;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTol = 1e-4;

/** @brief Build a row-major matrix from triplets, keeping explicitly given zeros as stored entries */
trajopt_ifopt::Jacobian makeMatrix(Eigen::Index rows,
                                   Eigen::Index cols,
                                   const std::vector<Eigen::Triplet<double>>& triplets)
{
  trajopt_ifopt::Jacobian matrix(rows, cols);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  return matrix;
}

/**
 * @brief Minimize x'x + g'x subject to l <= A x <= u
 * @return Whether the solve succeeded; the solution is written to x
 */
bool solveQP(PIQPSolver& solver,
             const Eigen::VectorXd& g,
             const trajopt_ifopt::Jacobian& A,
             const Eigen::VectorXd& l,
             const Eigen::VectorXd& u,
             Eigen::VectorXd& x)
{
  const Eigen::Index n = g.size();
  trajopt_ifopt::Jacobian hessian(n, n);
  hessian.setIdentity();

  solver.clear();
  solver.init(n, A.rows());
  solver.updateHessianMatrix(hessian);
  solver.updateGradient(g);
  solver.updateLinearConstraintsMatrix(A);
  solver.updateBounds(l, u);
  const bool solved = solver.solve();
  x = solver.getSolution();
  return solved;
}
}  // namespace

TEST(PIQPSolverUnit, EqualityRow)  // NOLINT
{
  PIQPSolver solver;
  const auto A = makeMatrix(1, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 } });
  Eigen::VectorXd x;
  ASSERT_TRUE(solveQP(
      solver, Eigen::Vector2d::Zero(), A, Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(2.0), x));
  EXPECT_TRUE(x.isApprox(Eigen::Vector2d(1.0, 1.0), kTol));
  EXPECT_EQ(solver.getSolverStatus(), QPSolverStatus::kInitialized);
}

TEST(PIQPSolverUnit, TwoSidedInequalityRow)  // NOLINT
{
  // Unconstrained optimum (2, 2) violates x0 + x1 <= 1
  PIQPSolver solver;
  const auto A = makeMatrix(1, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 } });
  Eigen::VectorXd x;
  ASSERT_TRUE(solveQP(
      solver, Eigen::Vector2d(-4.0, -4.0), A, Eigen::Matrix<double, 1, 1>(-1.0), Eigen::Matrix<double, 1, 1>(1.0), x));
  EXPECT_TRUE(x.isApprox(Eigen::Vector2d(0.5, 0.5), kTol));
}

TEST(PIQPSolverUnit, NegativeCoefficientBoundRows)  // NOLINT
{
  // -2 x0 in [-2, 10] means x0 in [-5, 1]; -x1 <= 1 means x1 >= -1
  PIQPSolver solver;
  const auto A = makeMatrix(2, 2, { { 0, 0, -2.0 }, { 1, 1, -1.0 } });
  Eigen::VectorXd x;
  ASSERT_TRUE(
      solveQP(solver, Eigen::Vector2d(-4.0, 4.0), A, Eigen::Vector2d(-2.0, -kInf), Eigen::Vector2d(10.0, 1.0), x));
  EXPECT_TRUE(x.isApprox(Eigen::Vector2d(1.0, -1.0), kTol));
}

TEST(PIQPSolverUnit, StoredZeroCoefficients)  // NOLINT
{
  // Stored zeros do not change the solution: row 0 stores only a zero, row 1 a zero next to its bound on x1
  PIQPSolver solver;
  const auto A = makeMatrix(2, 2, { { 0, 0, 0.0 }, { 1, 0, 0.0 }, { 1, 1, 1.0 } });
  ASSERT_EQ(A.nonZeros(), 3);
  Eigen::VectorXd x;
  ASSERT_TRUE(
      solveQP(solver, Eigen::Vector2d(-4.0, -4.0), A, Eigen::Vector2d(-1.0, -kInf), Eigen::Vector2d(1.0, 1.0), x));
  EXPECT_TRUE(x.allFinite());
  EXPECT_TRUE(x.isApprox(Eigen::Vector2d(2.0, 1.0), kTol));
}

TEST(PIQPSolverUnit, FreeRows)  // NOLINT
{
  PIQPSolver solver;
  const auto A = makeMatrix(2, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 } });
  Eigen::VectorXd x;
  ASSERT_TRUE(
      solveQP(solver, Eigen::Vector2d(-4.0, 2.0), A, Eigen::Vector2d(-kInf, -1e40), Eigen::Vector2d(kInf, 1e40), x));
  EXPECT_TRUE(x.isApprox(Eigen::Vector2d(2.0, -1.0), kTol));
}

TEST(PIQPSolverUnit, IntersectingBoundRows)  // NOLINT
{
  // x0 <= 3 and 2 x0 <= 2 on the same variable: the tighter one binds
  PIQPSolver solver;
  const auto A = makeMatrix(2, 1, { { 0, 0, 1.0 }, { 1, 0, 2.0 } });
  Eigen::VectorXd x;
  ASSERT_TRUE(solveQP(
      solver, Eigen::Matrix<double, 1, 1>(-4.0), A, Eigen::Vector2d(-kInf, -kInf), Eigen::Vector2d(3.0, 2.0), x));
  EXPECT_NEAR(x[0], 1.0, kTol);
}

TEST(PIQPSolverUnit, ResolveAfterBoundsChange)  // NOLINT
{
  PIQPSolver solver;
  const auto A = makeMatrix(2, 2, { { 0, 0, 1.0 }, { 1, 0, 1.0 }, { 1, 1, 1.0 } });
  Eigen::VectorXd x;
  ASSERT_TRUE(
      solveQP(solver, Eigen::Vector2d(-4.0, -4.0), A, Eigen::Vector2d(-kInf, -kInf), Eigen::Vector2d(1.0, 3.0), x));
  EXPECT_TRUE(x.isApprox(Eigen::Vector2d(1.0, 2.0), kTol));

  // Loosen the bound row and turn the second row into an equality
  solver.updateBounds(Eigen::Vector2d(-kInf, 2.5), Eigen::Vector2d(1.5, 2.5));
  ASSERT_TRUE(solver.solve());
  EXPECT_TRUE(solver.getSolution().isApprox(Eigen::Vector2d(1.25, 1.25), kTol));
}

TEST(PIQPSolverUnit, InfeasibleProblemFails)  // NOLINT
{
  PIQPSolver solver;
  const auto A = makeMatrix(2, 2, { { 0, 0, 1.0 }, { 0, 1, 1.0 }, { 1, 0, 1.0 }, { 1, 1, 1.0 } });
  Eigen::VectorXd x;
  EXPECT_FALSE(solveQP(solver, Eigen::Vector2d::Zero(), A, Eigen::Vector2d(3.0, -kInf), Eigen::Vector2d(kInf, 1.0), x));
  EXPECT_EQ(solver.getSolverStatus(), QPSolverStatus::kFailed);

  // A later successful solve clears the failure
  solver.updateBounds(Eigen::Vector2d(-kInf, -kInf), Eigen::Vector2d(kInf, 1.0));
  ASSERT_TRUE(solver.solve());
  EXPECT_EQ(solver.getSolverStatus(), QPSolverStatus::kInitialized);
}
