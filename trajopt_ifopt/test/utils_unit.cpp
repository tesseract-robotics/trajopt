#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <console_bridge/console.h>

using namespace trajopt;
using namespace std;

TEST(UtilsUnit, toBounds)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("UtilsUnit, toBounds");

  Eigen::MatrixX2d input_bounds(5, 2);
  input_bounds << -1, 1, -2, 2, -3, 3, -4, 4, -5, 5;
  std::vector<ifopt::Bounds> results = toBounds(input_bounds);

  // Check that the results are the correct size
  EXPECT_EQ(results.size(), input_bounds.rows());

  // Check that all bounds were set correctly
  for (Eigen::Index i = 0; i < input_bounds.rows(); i++)
  {
    EXPECT_EQ(input_bounds(i, 0), results[static_cast<std::size_t>(i)].lower_);
    EXPECT_EQ(input_bounds(i, 1), results[static_cast<std::size_t>(i)].upper_);
  }
}

TEST(UtilsUnit, interpolate)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("UtilsUnit, interpolate");

  Eigen::VectorXd start = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd end = Eigen::VectorXd::Ones(10);
  std::vector<Eigen::VectorXd> results = interpolate(start, end, 11);

  // Check size
  EXPECT_EQ(results.size(), 11);

  // Check that the start and end are correct
  EXPECT_TRUE(start.isApprox(results.front()));
  EXPECT_TRUE(end.isApprox(results.back()));

  // Check that intermediate results are correct
  EXPECT_TRUE(results[2].isApprox(Eigen::VectorXd::Ones(10) * 0.2));
  EXPECT_TRUE(results[5].isApprox(Eigen::VectorXd::Ones(10) * 0.5));
  EXPECT_TRUE(results[7].isApprox(Eigen::VectorXd::Ones(10) * 0.7));
}

/** @brief Tests getClosestValidPoint: Input within Finite bounds*/
TEST(UtilsUnit, getClosestValidPoint1)  // NOLINT
{
  ifopt::Bounds bound(-2.0, 2.0);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * 1.0;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  EXPECT_TRUE(output.isApprox(input));
}
/** @brief Tests getClosestValidPoint: Input greater than Finite bounds*/
TEST(UtilsUnit, getClosestValidPoint2)  // NOLINT
{
  ifopt::Bounds bound(-2.0, 2.0);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * 3.0;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  Eigen::VectorXd desired_results = Eigen::VectorXd::Ones(3) * 2.0;
  EXPECT_TRUE(output.isApprox(desired_results));
}
/** @brief Tests getClosestValidPoint: Input less than Finite bounds*/
TEST(UtilsUnit, getClosestValidPoint3)  // NOLINT
{
  ifopt::Bounds bound(-2.0, 2.0);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * -3.0;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  Eigen::VectorXd desired_results = Eigen::VectorXd::Ones(3) * -2.0;
  EXPECT_TRUE(output.isApprox(desired_results));
}
/** @brief Tests getClosestValidPoint: Input within BoundGreaterZero*/
TEST(UtilsUnit, getClosestValidPoint4)  // NOLINT
{
  ifopt::Bounds bound(ifopt::BoundGreaterZero);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * 3.0e6;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  EXPECT_TRUE(output.isApprox(input));
}
/** @brief Tests getClosestValidPoint: Input within BoundSmallerZero*/
TEST(UtilsUnit, getClosestValidPoint5)  // NOLINT
{
  ifopt::Bounds bound(ifopt::BoundSmallerZero);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * -3.0e6;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  EXPECT_TRUE(output.isApprox(input));
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
