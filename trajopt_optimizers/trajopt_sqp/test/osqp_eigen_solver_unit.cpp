/**
 * @file osqp_eigen_solver_unit.cpp
 * @brief Tests the status handling of OSQPEigenSolver
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
#include <trajopt_sqp/osqp_eigen_solver.h>

using trajopt_sqp::OSQPEigenSolver;
using trajopt_sqp::QPSolverStatus;

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
