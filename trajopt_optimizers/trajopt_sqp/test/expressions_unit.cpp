/**
 * @file expressions_unit.cpp
 * @brief Unit tests for expressions.h/.cpp
 *
 * @author Levi Armstrong
 * @date July 19, 2021
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <trajopt_sqp/expressions.h>
#include <tesseract/common/logging.h>
#include <cmath>
TRAJOPT_IGNORE_WARNINGS_POP
using trajopt_sqp::AffExprs;
using trajopt_sqp::QuadExprs;

TEST(ExpressionsTest, AffExprs)  // NOLINT
{
  TESSERACT_LOG_DEBUG("ExpressionsTest, AffExprs");
  // f = (x(0) - x(1))^2
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = 16;
  Eigen::Vector2d J(8, -8);

  AffExprs aff_exprs;
  aff_exprs.create(e, J.transpose().sparseView(), x);
  EXPECT_NEAR(aff_exprs.constants(0), -16, 1e-8);
  EXPECT_NEAR(aff_exprs.linear_coeffs.coeff(0, 0), 8, 1e-8);
  EXPECT_NEAR(aff_exprs.linear_coeffs.coeff(0, 1), -8, 1e-8);
  Eigen::VectorXd results(1);
  aff_exprs.values(results, x);
  EXPECT_NEAR(results(0), e(0), 1e-8);
}

TEST(ExpressionsTest, AffExprsWithWeights)  // NOLINT
{
  TESSERACT_LOG_DEBUG("ExpressionsTest, AffExprsWithWeights");
  // f = (x(0) - x(1))^2
  // weight = 5
  const double w = 5;
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = 16;
  Eigen::Vector2d J(8, -8);

  AffExprs aff_exprs1;
  aff_exprs1.create(w * e, (w * J).transpose().sparseView(), x);
  AffExprs aff_exprs2;
  aff_exprs2.create(e, J.transpose().sparseView(), x);
  EXPECT_NEAR(aff_exprs1.constants(0), w * aff_exprs2.constants(0), 1e-8);
  EXPECT_NEAR(aff_exprs1.linear_coeffs.coeff(0, 0), w * aff_exprs2.linear_coeffs.coeff(0, 0), 1e-8);
  EXPECT_NEAR(aff_exprs1.linear_coeffs.coeff(0, 1), w * aff_exprs2.linear_coeffs.coeff(0, 1), 1e-8);

  Eigen::VectorXd results(1);
  aff_exprs1.values(results, x);
  EXPECT_NEAR(results(0), w * e(0), 1e-8);
}

TEST(ExpressionsTest, QuadExprs)  // NOLINT
{
  TESSERACT_LOG_DEBUG("ExpressionsTest, QuadExprs");
  // f = (x(0) - x(1))^2
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = 16;
  Eigen::Vector2d J(8, -8);
  Eigen::Matrix2d H;
  H << 2, -2, -2, 2;

  QuadExprs quad_exprs;
  quad_exprs.create(e, J.transpose().sparseView(), { H.sparseView() }, x);
  EXPECT_NEAR(quad_exprs.constants(0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 1), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 1), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), -1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 0), -1, 1e-8);

  Eigen::VectorXd results(1);
  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), e(0), 1e-8);

  // Because the function is a quadratic it should be an exact fit so check another set of values
  x = Eigen::Vector2d(8, 2);
  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), std::pow(x(0) - x(1), 2.0), 1e-8);
}

TEST(ExpressionsTest, squareAffExprs1)  // NOLINT
{
  TESSERACT_LOG_DEBUG("ExpressionsTest, QuadExprs");
  // This should produce the same results as the QuadExprs unit test

  // f = (x(0) - x(1))
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = x(0) - x(1);
  Eigen::Vector2d J(1, -1);

  AffExprs aff_exprs;
  aff_exprs.create(e, J.transpose().sparseView(), x);

  Eigen::VectorXd results(1);
  aff_exprs.values(results, x);
  EXPECT_NEAR(results(0), e(0), 1e-8);

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, Eigen::VectorXd::Ones(aff_exprs.constants.size()));

  EXPECT_NEAR(quad_exprs.constants(0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 1), 0, 1e-8);

  // New representation: quadratic_coeffs[0] stores q as a 1×n row vector,
  // where q = sqrt(w) * b^T. Here w=1 and b=[1, -1], so q=[1, -1].
  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].cols(), 2);

  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), -1, 1e-8);

  // And the aggregated objective quadratic is still the full outer product:
  // H = Bw^T * Bw = [ [1, -1], [-1, 1] ].
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 0), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 1), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 1), -1, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 0), -1, 1e-8);

  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), std::pow(x(0) - x(1), 2.0), 1e-8);

  // Because the function is a quadratic it should be an exact fit so check another set of values
  x = Eigen::Vector2d(8, 2);
  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), std::pow(x(0) - x(1), 2.0), 1e-8);
}

TEST(ExpressionsTest, squareAffExprs2)  // NOLINT
{
  TESSERACT_LOG_DEBUG("ExpressionsTest, squareAffExprs");
  // f = 5 - (x(0) - x(1))
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = 5 - (x(0) - x(1));
  Eigen::Vector2d J(-1, 1);

  AffExprs aff_exprs;
  aff_exprs.create(e, J.transpose().sparseView(), x);

  Eigen::VectorXd results(1);
  aff_exprs.values(results, x);
  EXPECT_NEAR(results(0), e(0), 1e-8);

  // The affine expression = (a + b*x + c*y) where x = x(0) and y = x(1)
  // The squared affine expressions = a^2 + 2*a*b*x + 2*a*c*y + 2*b*c*x*y + b^2 * x^2 + c^2 * y^2
  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, Eigen::VectorXd::Ones(aff_exprs.constants.size()));

  EXPECT_NEAR(quad_exprs.constants(0), std::pow(aff_exprs.constants(0), 2.0), 1e-8);
  EXPECT_NEAR(
      quad_exprs.linear_coeffs.coeff(0, 0), 2 * aff_exprs.constants(0) * aff_exprs.linear_coeffs.coeff(0, 0), 1e-8);
  EXPECT_NEAR(
      quad_exprs.linear_coeffs.coeff(0, 1), 2 * aff_exprs.constants(0) * aff_exprs.linear_coeffs.coeff(0, 1), 1e-8);

  // New representation: quadratic_coeffs[0] stores q as a 1×n row vector,
  // where q = sqrt(w) * b^T. Here w=1, so q == b.
  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].cols(), 2);

  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), aff_exprs.linear_coeffs.coeff(0, 0), 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), aff_exprs.linear_coeffs.coeff(0, 1), 1e-8);

  // And the aggregated objective quadratic is still the full outer product: H = b^T b.
  const double b0 = aff_exprs.linear_coeffs.coeff(0, 0);
  const double b1 = aff_exprs.linear_coeffs.coeff(0, 1);

  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 0), b0 * b0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 1), b1 * b1, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 1), b0 * b1, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 0), b0 * b1, 1e-8);

  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), std::pow(e(0), 2.0), 1e-8);
}

TEST(ExpressionsTest, squareAffExprsMultiRowWeighted)  // NOLINT
{
  // Three weighted affine expressions over three variables, the last with no linear terms:
  //   f0(x) =  1 + ( 2*x0 - 1*x1 )   w0 = 0.5
  //   f1(x) = -2 + ( 3*x1 + 1*x2 )   w1 = 2.0
  //   f2(x) =  4                     w2 = 3.0  (empty Jacobian row)
  const Eigen::Index m = 3;
  const Eigen::Index n = 3;

  AffExprs aff_exprs;
  aff_exprs.constants.resize(m);
  aff_exprs.constants << 1.0, -2.0, 4.0;

  Eigen::MatrixXd B(m, n);
  B << 2.0, -1.0, 0.0, 0.0, 3.0, 1.0, 0.0, 0.0, 0.0;
  aff_exprs.linear_coeffs = B.sparseView();

  Eigen::VectorXd w(m);
  w << 0.5, 2.0, 3.0;

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, w);

  // constants(i) = w_i * a_i^2
  EXPECT_NEAR(quad_exprs.constants(0), 0.5, 1e-8);
  EXPECT_NEAR(quad_exprs.constants(1), 8.0, 1e-8);
  EXPECT_NEAR(quad_exprs.constants(2), 48.0, 1e-8);

  // linear_coeffs.row(i) = 2 * a_i * w_i * b_i
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 0), 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 1), -1.0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(1, 1), -24.0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(1, 2), -8.0, 1e-8);

  // objective_linear_coeffs = column sums of the scaled linear coefficients
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(0), 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(1), -25.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(2), -8.0, 1e-8);

  // quadratic_coeffs[i] = sqrt(w_i) * b_i, stored as a 1 x n row; an expression with no
  // linear terms yields an empty entry.
  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 3);
  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].cols(), n);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), std::sqrt(0.5) * 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), std::sqrt(0.5) * -1.0, 1e-8);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 2);

  ASSERT_EQ(quad_exprs.quadratic_coeffs[1].rows(), 1);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 1), std::sqrt(2.0) * 3.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 2), std::sqrt(2.0) * 1.0, 1e-8);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[1].nonZeros(), 2);

  EXPECT_EQ(quad_exprs.quadratic_coeffs[2].rows(), 0);

  // objective_quadratic_coeffs = sum_i w_i * b_i * b_i^T
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 0), 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 1), -1.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 0), -1.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 1), 18.5, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 2), 6.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(2, 1), 6.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(2, 2), 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 2), 0.0, 1e-8);

  // The model is exact for a squared affine function, so values(x) == w_i * f_i(x)^2 at any x.
  Eigen::VectorXd results(m);
  Eigen::VectorXd x(n);

  x << 1.0, 2.0, 3.0;
  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), 0.5, 1e-8);
  EXPECT_NEAR(results(1), 98.0, 1e-8);
  EXPECT_NEAR(results(2), 48.0, 1e-8);

  x << -2.0, 0.5, 1.0;
  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), 6.125, 1e-8);
  EXPECT_NEAR(results(1), 0.5, 1e-8);
  EXPECT_NEAR(results(2), 48.0, 1e-8);
}

TEST(ExpressionsTest, squareAffExprsReuseAcrossRowCounts)  // NOLINT
{
  const Eigen::Index n = 3;

  // First expression set: 3 rows, matching squareAffExprsMultiRowWeighted.
  AffExprs aff_exprs;
  aff_exprs.constants.resize(3);
  aff_exprs.constants << 1.0, -2.0, 4.0;

  Eigen::MatrixXd B_a(3, n);
  B_a << 2.0, -1.0, 0.0, 0.0, 3.0, 1.0, 0.0, 0.0, 0.0;
  aff_exprs.linear_coeffs = B_a.sparseView();

  Eigen::VectorXd w_a(3);
  w_a << 0.5, 2.0, 3.0;

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, w_a);
  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 3);

  // Second expression set: 2 rows, different sparsity, into the same AffExprs and QuadExprs.
  //   g0(x) =  3 + ( 5*x2 )          v0 = 1.0
  //   g1(x) = -1 + (-2*x0 + 1*x1 )   v1 = 4.0
  aff_exprs.constants.resize(2);
  aff_exprs.constants << 3.0, -1.0;

  Eigen::MatrixXd B_b(2, n);
  B_b << 0.0, 0.0, 5.0, -2.0, 1.0, 0.0;
  aff_exprs.linear_coeffs = B_b.sparseView();

  Eigen::VectorXd v(2);
  v << 1.0, 4.0;

  aff_exprs.square(quad_exprs, v);

  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 2);
  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 1);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 2), 5.0, 1e-8);

  ASSERT_EQ(quad_exprs.quadratic_coeffs[1].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[1].nonZeros(), 2);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 0), -4.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 1), 2.0, 1e-8);

  Eigen::VectorXd results_b(2);
  Eigen::VectorXd x(n);
  x << 1.0, 2.0, 3.0;
  quad_exprs.values(results_b, x);
  EXPECT_NEAR(results_b(0), 324.0, 1e-8);
  EXPECT_NEAR(results_b(1), 4.0, 1e-8);

  // Back to the 3-row set in the same objects; results must match the first squaring.
  aff_exprs.constants.resize(3);
  aff_exprs.constants << 1.0, -2.0, 4.0;
  aff_exprs.linear_coeffs = B_a.sparseView();

  aff_exprs.square(quad_exprs, w_a);

  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 3);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 2);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), std::sqrt(0.5) * 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), std::sqrt(0.5) * -1.0, 1e-8);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[2].rows(), 0);

  Eigen::VectorXd results_a(3);
  quad_exprs.values(results_a, x);
  EXPECT_NEAR(results_a(0), 0.5, 1e-8);
  EXPECT_NEAR(results_a(1), 98.0, 1e-8);
  EXPECT_NEAR(results_a(2), 48.0, 1e-8);
}

TEST(ExpressionsTest, squareAffExprsReuseSameRowCountDifferentPattern)  // NOLINT
{
  // Two consecutive squarings with the same row count but different sparsity. Because the row
  // count is unchanged, quadratic_coeffs is not resized, so every Qi is reused while still
  // holding the previous pattern.
  const Eigen::Index m = 2;
  const Eigen::Index n = 3;

  // First set:
  //   f0(x) =  1 + ( 2*x0 - 1*x1 )   w0 = 1.0   (2 nonzeros, columns 0 and 1)
  //   f1(x) = -1 + ( 4*x2 )          w1 = 1.0   (1 nonzero,  column 2)
  AffExprs aff_exprs;
  aff_exprs.constants.resize(m);
  aff_exprs.constants << 1.0, -1.0;

  Eigen::MatrixXd B_first(m, n);
  B_first << 2.0, -1.0, 0.0, 0.0, 0.0, 4.0;
  aff_exprs.linear_coeffs = B_first.sparseView();

  Eigen::VectorXd w_first(m);
  w_first << 1.0, 1.0;

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, w_first);

  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 2);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 2);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[1].nonZeros(), 1);

  // Second set, same row count and inverted shape: row 0 shrinks to a single nonzero in a column
  // it did not previously occupy, row 1 grows to three.
  //   g0(x) = 2   + ( 3*x2 )                 v0 = 4.0
  //   g1(x) = 0.5 + (-1*x0 + 2*x1 + 1*x2 )   v1 = 1.0
  aff_exprs.constants << 2.0, 0.5;

  Eigen::MatrixXd B_second(m, n);
  B_second << 0.0, 0.0, 3.0, -1.0, 2.0, 1.0;
  aff_exprs.linear_coeffs = B_second.sparseView();

  Eigen::VectorXd v(m);
  v << 4.0, 1.0;

  aff_exprs.square(quad_exprs, v);

  // Row count unchanged, so the container was reused rather than resized.
  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 2);

  // constants(i) = v_i * a_i^2
  EXPECT_NEAR(quad_exprs.constants(0), 16.0, 1e-8);
  EXPECT_NEAR(quad_exprs.constants(1), 0.25, 1e-8);

  // objective_linear_coeffs = sum_i 2 * a_i * v_i * b_i, column-summed across both rows.
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(0), -1.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(1), 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(2), 49.0, 1e-8);

  // q_0 = sqrt(4) * [0, 0, 3]; nothing of the previous pattern survives.
  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 1);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 2), 6.0, 1e-8);

  // q_1 = sqrt(1) * [-1, 2, 1]
  ASSERT_EQ(quad_exprs.quadratic_coeffs[1].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[1].nonZeros(), 3);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 0), -1.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 1), 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 2), 1.0, 1e-8);

  Eigen::VectorXd results(m);
  Eigen::VectorXd x(n);
  x << 1.0, 2.0, 3.0;
  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), 484.0, 1e-8);
  EXPECT_NEAR(results(1), 42.25, 1e-8);
}

TEST(ExpressionsTest, squareAffExprsReuseRowLosesAndRegainsTerms)  // NOLINT
{
  // A reused entry whose expression loses its linear terms becomes empty, stays empty when squared
  // again, and takes terms back afterwards.
  const Eigen::Index m = 2;
  const Eigen::Index n = 3;

  // With terms in both rows:
  //   f0(x) = 1 + ( 2*x0 - 1*x1 )   w0 = 4.0
  //   f1(x) = 2 + ( 3*x2 )          w1 = 1.0
  Eigen::MatrixXd B_full(m, n);
  B_full << 2.0, -1.0, 0.0, 0.0, 0.0, 3.0;

  // Without terms in row 0:
  //   g0(x) = 5                     w0 = 4.0
  //   g1(x) = 2 + ( 3*x2 )          w1 = 1.0
  Eigen::MatrixXd B_partial(m, n);
  B_partial << 0.0, 0.0, 0.0, 0.0, 0.0, 3.0;

  Eigen::VectorXd w(m);
  w << 4.0, 1.0;

  Eigen::VectorXd results(m);
  Eigen::VectorXd x(n);
  x << 2.0, 1.0, 3.0;

  AffExprs aff_exprs;
  aff_exprs.constants.resize(m);
  aff_exprs.constants << 1.0, 2.0;
  aff_exprs.linear_coeffs = B_full.sparseView();

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, w);
  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 2);
  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 2);

  // Row 0 loses its terms. Squared twice, so its entry is also visited while already empty.
  for (int pass = 0; pass < 2; ++pass)
  {
    aff_exprs.constants << 5.0, 2.0;
    aff_exprs.linear_coeffs = B_partial.sparseView();

    aff_exprs.square(quad_exprs, w);

    ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 2);
    EXPECT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 0);
    ASSERT_EQ(quad_exprs.quadratic_coeffs[1].rows(), 1);
    EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 2), 3.0, 1e-8);

    quad_exprs.values(results, x);
    EXPECT_NEAR(results(0), 100.0, 1e-8);
    EXPECT_NEAR(results(1), 121.0, 1e-8);
  }

  // Row 0 takes its terms back.
  aff_exprs.constants << 1.0, 2.0;
  aff_exprs.linear_coeffs = B_full.sparseView();

  aff_exprs.square(quad_exprs, w);

  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].cols(), n);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 2);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), 4.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), -2.0, 1e-8);

  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), 64.0, 1e-8);
  EXPECT_NEAR(results(1), 121.0, 1e-8);
}

TEST(ExpressionsTest, squareAffExprsZeroWeightRow)  // NOLINT
{
  // A weight of exactly zero disables its expression: every term it contributes is zero.
  //   f0(x) = 3 + ( 2*x0 - 1*x1 )   w0 = 0.0
  //   f1(x) = 1 + ( 1*x1 + 4*x2 )   w1 = 2.0
  const Eigen::Index m = 2;
  const Eigen::Index n = 3;

  AffExprs aff_exprs;
  aff_exprs.constants.resize(m);
  aff_exprs.constants << 3.0, 1.0;

  Eigen::MatrixXd B(m, n);
  B << 2.0, -1.0, 0.0, 0.0, 1.0, 4.0;
  aff_exprs.linear_coeffs = B.sparseView();

  Eigen::VectorXd w(m);
  w << 0.0, 2.0;

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, w);

  EXPECT_EQ(quad_exprs.constants(0), 0.0);
  EXPECT_EQ(quad_exprs.linear_coeffs.coeff(0, 0), 0.0);
  EXPECT_EQ(quad_exprs.linear_coeffs.coeff(0, 1), 0.0);
  EXPECT_NEAR(quad_exprs.constants(1), 2.0, 1e-8);

  // Only the weighted expression reaches the aggregate objective.
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(0), 0.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(1), 4.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_linear_coeffs(2), 16.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 0), 0.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(0, 1), 0.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 1), 2.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(1, 2), 8.0, 1e-8);
  EXPECT_NEAR(quad_exprs.objective_quadratic_coeffs.coeff(2, 2), 32.0, 1e-8);

  Eigen::VectorXd results(m);
  Eigen::VectorXd x(n);
  x << 1.0, 2.0, 3.0;
  quad_exprs.values(results, x);
  EXPECT_EQ(results(0), 0.0);
  EXPECT_NEAR(results(1), 450.0, 1e-8);
}

TEST(ExpressionsTest, squareAffExprsUncompressedInput)  // NOLINT
{
  // linear_coeffs filled entry by entry and never compressed squares like its compressed equal.
  //   f0(x) =  1 + ( 2*x1 + 1*x3 )   w0 = 4.0
  //   f1(x) = -2 + (-1*x2 )          w1 = 1.0
  const Eigen::Index m = 2;
  const Eigen::Index n = 4;

  AffExprs aff_exprs;
  aff_exprs.constants.resize(m);
  aff_exprs.constants << 1.0, -2.0;

  aff_exprs.linear_coeffs.resize(m, n);
  aff_exprs.linear_coeffs.reserve(Eigen::VectorXi::Constant(m, 3));
  aff_exprs.linear_coeffs.insert(0, 3) = 1.0;
  aff_exprs.linear_coeffs.insert(0, 1) = 2.0;
  aff_exprs.linear_coeffs.insert(1, 2) = -1.0;
  ASSERT_FALSE(aff_exprs.linear_coeffs.isCompressed());

  Eigen::VectorXd w(m);
  w << 4.0, 1.0;

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, w);

  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 2);
  ASSERT_EQ(quad_exprs.quadratic_coeffs[0].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[0].nonZeros(), 2);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), 4.0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 3), 2.0, 1e-8);
  ASSERT_EQ(quad_exprs.quadratic_coeffs[1].rows(), 1);
  EXPECT_EQ(quad_exprs.quadratic_coeffs[1].nonZeros(), 1);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[1].coeff(0, 2), -1.0, 1e-8);

  Eigen::VectorXd results(m);
  Eigen::VectorXd x(n);
  x << 1.0, 2.0, 3.0, 4.0;
  quad_exprs.values(results, x);
  EXPECT_NEAR(results(0), 324.0, 1e-8);
  EXPECT_NEAR(results(1), 25.0, 1e-8);
}

TEST(ExpressionsTest, squareAffExprsNoExpressions)  // NOLINT
{
  // A set with no expressions squares to an empty model over the same variables, also into a
  // QuadExprs that held expressions before.
  const Eigen::Index n = 3;

  AffExprs aff_exprs;
  aff_exprs.constants.resize(2);
  aff_exprs.constants << 1.0, -1.0;

  Eigen::MatrixXd B(2, n);
  B << 2.0, 0.0, 0.0, 0.0, 0.0, 4.0;
  aff_exprs.linear_coeffs = B.sparseView();

  QuadExprs quad_exprs;
  aff_exprs.square(quad_exprs, Eigen::VectorXd::Ones(2));
  ASSERT_EQ(quad_exprs.quadratic_coeffs.size(), 2);

  aff_exprs.constants.resize(0);
  aff_exprs.linear_coeffs.resize(0, n);
  aff_exprs.square(quad_exprs, Eigen::VectorXd(0));

  EXPECT_EQ(quad_exprs.constants.size(), 0);
  EXPECT_EQ(quad_exprs.linear_coeffs.rows(), 0);
  EXPECT_TRUE(quad_exprs.quadratic_coeffs.empty());
  ASSERT_EQ(quad_exprs.objective_linear_coeffs.size(), n);
  EXPECT_TRUE(quad_exprs.objective_linear_coeffs.isZero());
  EXPECT_EQ(quad_exprs.objective_quadratic_coeffs.rows(), n);
  EXPECT_EQ(quad_exprs.objective_quadratic_coeffs.cols(), n);
  EXPECT_EQ(quad_exprs.objective_quadratic_coeffs.nonZeros(), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
