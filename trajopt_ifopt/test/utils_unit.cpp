#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/utils/utils.h>
#include <console_bridge/console.h>

using namespace trajopt;
using namespace std;

TEST(UtilsUnit, toBounds)
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

TEST(UtilsUnit, interpolate)
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

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
