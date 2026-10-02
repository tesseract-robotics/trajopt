/**
 * @file warm_start_unit.cpp
 * @brief Tests the QP start point computed at a linearization point
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
#include <limits>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/eigen_types.h>
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
}  // namespace

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
