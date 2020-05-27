#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <console_bridge/console.h>

using namespace trajopt;
using namespace std;

/**
 * @brief Tests the Joint Position Constraint
 */
TEST(JointTermsUnit, joint_pos_constraint_1)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointPosConstraint_1");

  std::vector<JointPosition::ConstPtr> position_vars;
  std::vector<std::string> joint_names(10, "name");
  Eigen::VectorXd init1(10);
  init1 << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  position_vars.push_back(std::make_shared<JointPosition>(init1, joint_names, "test_var1"));

  Eigen::VectorXd init2(10);
  init2 << 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
  position_vars.push_back(std::make_shared<JointPosition>(init2, joint_names, "test_var2"));

  Eigen::VectorXd targets(10);
  targets << 20, 21, 22, 23, 24, 25, 26, 27, 28, 29;

  std::string name("test_cnt");
  JointPosConstraint position_cnt(targets, position_vars, name);

  EXPECT_EQ(position_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(position_cnt.GetName(), name);
  EXPECT_EQ(position_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
