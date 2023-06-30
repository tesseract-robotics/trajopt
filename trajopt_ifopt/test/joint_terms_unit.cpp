/**
 * @file joint_term_unit.cpp
 * @brief The joint position constraint unit test
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <ctime>
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <console_bridge/console.h>

using namespace trajopt_ifopt;
using namespace std;

/** @brief Tests the Joint Position Constraint */
TEST(JointTermsUnit, JointPosConstraintUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointPosConstraintUnit");

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
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(10, 1);
  JointPosConstraint position_cnt(targets, position_vars, coeffs, name);

  EXPECT_EQ(position_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(position_cnt.GetName(), name);
  EXPECT_EQ(position_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
}

/** @brief Tests the Joint Velocity Constraint */
TEST(JointTermsUnit, JointVelConstraintUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointVelConstraintUnit");

  // y = x^3 + 5*x^2 + 2*x + 1
  auto f = [](double x) { return (x * x * x + 5 * x * x + 2 * x + 1); };

  auto variables = std::make_shared<ifopt::Composite>("variable-sets", false);
  std::vector<JointPosition::ConstPtr> position_vars;
  position_vars.reserve(27);

  std::vector<int> x_vals;
  x_vals.reserve(27);
  for (int i = -13; i < 14; ++i)
  {
    std::vector<std::string> joint_names{ "x", "y" };
    Eigen::VectorXd val(2);
    val << f(i), f(i);
    auto var = std::make_shared<JointPosition>(val, joint_names, "test_var_" + std::to_string(position_vars.size()));
    position_vars.push_back(var);
    variables->AddComponent(var);
    x_vals.push_back(i);
  }

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  std::string name("test_joint_vel_cnt");
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointVelConstraint velocity_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  velocity_cnt.LinkWithVariables(variables);

  EXPECT_EQ(velocity_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));
  EXPECT_EQ(velocity_cnt.GetName(), name);
  EXPECT_EQ(velocity_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  Eigen::VectorXd velocity_vals = velocity_cnt.GetValues();
  EXPECT_EQ(velocity_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 1; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (f(x_vals[i + 1]) - f(x_vals[i]));
      EXPECT_NEAR(velocity_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  ifopt::ConstraintSet::Jacobian jac = velocity_cnt.GetJacobian();
  ifopt::Problem::Jacobian num_jac = trajopt_ifopt::calcNumericalConstraintGradient(*variables, velocity_cnt);

  EXPECT_EQ(jac.rows(), static_cast<Eigen::Index>(x_vals.size() - 1) * targets.size());
  EXPECT_EQ(jac.cols(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(x_vals.size() - 1) * targets.size(); ++i)
  {
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++j)
    {
      EXPECT_NEAR(jac.coeffRef(i, j), num_jac.coeffRef(i, j), 1e-3);
    }
  }
}

/** @brief Tests the Joint Velocity Constraint */
TEST(JointTermsUnit, JointVelConstraintMinimumUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointVelConstraintMinimumUnit");

  // y = x^3 + 5*x^2 + 2*x + 1
  auto f = [](double x) { return (x * x * x + 5 * x * x + 2 * x + 1); };

  auto variables = std::make_shared<ifopt::Composite>("variable-sets", false);
  std::vector<JointPosition::ConstPtr> position_vars;
  position_vars.reserve(2);

  std::vector<int> x_vals;
  x_vals.reserve(2);
  for (int i = -13; i < -11; ++i)
  {
    std::vector<std::string> joint_names{ "x", "y" };
    Eigen::VectorXd val(2);
    val << f(i), f(i);
    auto var = std::make_shared<JointPosition>(val, joint_names, "test_var_" + std::to_string(position_vars.size()));
    position_vars.push_back(var);
    variables->AddComponent(var);
    x_vals.push_back(i);
  }

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  std::string name("test_joint_vel_cnt");
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointVelConstraint velocity_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  velocity_cnt.LinkWithVariables(variables);

  EXPECT_EQ(velocity_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));
  EXPECT_EQ(velocity_cnt.GetName(), name);
  EXPECT_EQ(velocity_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  Eigen::VectorXd velocity_vals = velocity_cnt.GetValues();
  EXPECT_EQ(velocity_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 1; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (f(x_vals[i + 1]) - f(x_vals[i]));
      EXPECT_NEAR(velocity_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  ifopt::ConstraintSet::Jacobian jac = velocity_cnt.GetJacobian();
  ifopt::Problem::Jacobian num_jac = trajopt_ifopt::calcNumericalConstraintGradient(*variables, velocity_cnt);

  EXPECT_EQ(jac.rows(), static_cast<Eigen::Index>(x_vals.size() - 1) * targets.size());
  EXPECT_EQ(jac.cols(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(x_vals.size() - 1) * targets.size(); ++i)
  {
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++j)
    {
      EXPECT_NEAR(jac.coeffRef(i, j), num_jac.coeffRef(i, j), 1e-3);
    }
  }
}

/** @brief Tests the Joint Acceleration Constraint */
TEST(JointTermsUnit, JointAccelConstraintUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointVelConstraintUnit");

  // y = x^3 + 5*x^2 + 2*x + 1
  auto f = [](double x) { return (x * x * x + 5 * x * x + 2 * x + 1); };

  auto variables = std::make_shared<ifopt::Composite>("variable-sets", false);
  std::vector<JointPosition::ConstPtr> position_vars;
  position_vars.reserve(27);

  std::vector<int> x_vals;
  x_vals.reserve(27);
  for (int i = -13; i < 14; ++i)
  {
    std::vector<std::string> joint_names{ "x", "y" };
    Eigen::VectorXd val(2);
    val << f(i), f(i);
    auto var = std::make_shared<JointPosition>(val, joint_names, "test_var_" + std::to_string(position_vars.size()));
    position_vars.push_back(var);
    variables->AddComponent(var);
    x_vals.push_back(i);
  }

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  std::string name("test_joint_accel_cnt");
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointAccelConstraint accel_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  accel_cnt.LinkWithVariables(variables);

  EXPECT_EQ(accel_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(accel_cnt.GetName(), name);
  EXPECT_EQ(accel_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd accel_vals = accel_cnt.GetValues();
  EXPECT_EQ(accel_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 2; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (f(x_vals[i]) - 2.0 * f(x_vals[i + 1]) + f(x_vals[i + 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 2; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (f(x_vals[i]) - 2.0 * f(x_vals[i - 1]) + f(x_vals[i - 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  ifopt::ConstraintSet::Jacobian jac = accel_cnt.GetJacobian();
  ifopt::Problem::Jacobian num_jac = trajopt_ifopt::calcNumericalConstraintGradient(*variables, accel_cnt);

  EXPECT_EQ(jac.rows(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());
  EXPECT_EQ(jac.cols(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++i)
  {
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++j)
    {
      EXPECT_NEAR(jac.coeffRef(i, j), num_jac.coeffRef(i, j), 1e-3);
    }
  }
}

/** @brief Tests the Joint Acceleration Constraint */
TEST(JointTermsUnit, JointAccelConstraintMinimumUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointAccelConstraintMinimumUnit");

  // y = x^3 + 5*x^2 + 2*x + 1
  auto f = [](double x) { return (x * x * x + 5 * x * x + 2 * x + 1); };

  auto variables = std::make_shared<ifopt::Composite>("variable-sets", false);
  std::vector<JointPosition::ConstPtr> position_vars;
  position_vars.reserve(4);

  std::vector<int> x_vals;
  x_vals.reserve(4);
  for (int i = -13; i < -9; ++i)
  {
    std::vector<std::string> joint_names{ "x", "y" };
    Eigen::VectorXd val(2);
    val << f(i), f(i);
    auto var = std::make_shared<JointPosition>(val, joint_names, "test_var_" + std::to_string(position_vars.size()));
    position_vars.push_back(var);
    variables->AddComponent(var);
    x_vals.push_back(i);
  }

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  std::string name("test_joint_accel_cnt");
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointAccelConstraint accel_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  accel_cnt.LinkWithVariables(variables);

  EXPECT_EQ(accel_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(accel_cnt.GetName(), name);
  EXPECT_EQ(accel_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd accel_vals = accel_cnt.GetValues();
  EXPECT_EQ(accel_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 2; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (f(x_vals[i]) - 2.0 * f(x_vals[i + 1]) + f(x_vals[i + 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 2; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (f(x_vals[i]) - 2.0 * f(x_vals[i - 1]) + f(x_vals[i - 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  ifopt::ConstraintSet::Jacobian jac = accel_cnt.GetJacobian();
  ifopt::Problem::Jacobian num_jac = trajopt_ifopt::calcNumericalConstraintGradient(*variables, accel_cnt);

  EXPECT_EQ(jac.rows(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());
  EXPECT_EQ(jac.cols(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++i)
  {
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++j)
    {
      EXPECT_NEAR(jac.coeffRef(i, j), num_jac.coeffRef(i, j), 1e-3);
    }
  }
}

/** @brief Tests the Joint Jerk Constraint */
TEST(JointTermsUnit, JointJerkConstraintUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointJerkConstraintUnit");

  // y = x^3 + 5*x^2 + 2*x + 1
  auto f = [](double x) { return (x * x * x + 5 * x * x + 2 * x + 1); };

  auto variables = std::make_shared<ifopt::Composite>("variable-sets", false);
  std::vector<JointPosition::ConstPtr> position_vars;
  position_vars.reserve(27);

  std::vector<int> x_vals;
  x_vals.reserve(27);
  for (int i = -13; i < 14; ++i)
  {
    std::vector<std::string> joint_names{ "x", "y" };
    Eigen::VectorXd val(2);
    val << f(i), f(i);
    auto var = std::make_shared<JointPosition>(val, joint_names, "test_var_" + std::to_string(position_vars.size()));
    position_vars.push_back(var);
    variables->AddComponent(var);
    x_vals.push_back(i);
  }

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  std::string name("test_joint_jerk_cnt");
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointJerkConstraint jerk_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  jerk_cnt.LinkWithVariables(variables);

  EXPECT_EQ(jerk_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(jerk_cnt.GetName(), name);
  EXPECT_EQ(jerk_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd jerk_vals = jerk_cnt.GetValues();
  EXPECT_EQ(jerk_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 3; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (-f(x_vals[i]) + 3.0 * f(x_vals[i + 1]) - 3.0 * f(x_vals[i + 2])) + f(x_vals[i + 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 2; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = f(x_vals[i]) - 3.0 * f(x_vals[i - 1]) + 3.0 * f(x_vals[i - 2]) - f(x_vals[i - 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  ifopt::ConstraintSet::Jacobian jac = jerk_cnt.GetJacobian();
  ifopt::Problem::Jacobian num_jac = trajopt_ifopt::calcNumericalConstraintGradient(*variables, jerk_cnt);

  EXPECT_EQ(jac.rows(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());
  EXPECT_EQ(jac.cols(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++i)
  {
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++j)
    {
      EXPECT_NEAR(jac.coeffRef(i, j), num_jac.coeffRef(i, j), 1e-3);
    }
  }
}

/** @brief Tests the Joint Jerk Constraint */
TEST(JointTermsUnit, JointJerkConstraintMinimumUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointJerkConstraintMinimumUnit");

  // y = x^3 + 5*x^2 + 2*x + 1
  auto f = [](double x) { return (x * x * x + 5 * x * x + 2 * x + 1); };

  auto variables = std::make_shared<ifopt::Composite>("variable-sets", false);
  std::vector<JointPosition::ConstPtr> position_vars;
  position_vars.reserve(6);

  std::vector<int> x_vals;
  x_vals.reserve(6);
  for (int i = -13; i < -7; ++i)
  {
    std::vector<std::string> joint_names{ "x", "y" };
    Eigen::VectorXd val(2);
    val << f(i), f(i);
    auto var = std::make_shared<JointPosition>(val, joint_names, "test_var_" + std::to_string(position_vars.size()));
    position_vars.push_back(var);
    variables->AddComponent(var);
    x_vals.push_back(i);
  }

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  std::string name("test_joint_jerk_cnt");
  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointJerkConstraint jerk_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  jerk_cnt.LinkWithVariables(variables);

  EXPECT_EQ(jerk_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(jerk_cnt.GetName(), name);
  EXPECT_EQ(jerk_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd jerk_vals = jerk_cnt.GetValues();
  EXPECT_EQ(jerk_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 3; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = (-f(x_vals[i]) + 3.0 * f(x_vals[i + 1]) - 3.0 * f(x_vals[i + 2])) + f(x_vals[i + 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 2; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      double expected_val = f(x_vals[i]) - 3.0 * f(x_vals[i - 1]) + 3.0 * f(x_vals[i - 2]) - f(x_vals[i - 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  ifopt::ConstraintSet::Jacobian jac = jerk_cnt.GetJacobian();
  ifopt::Problem::Jacobian num_jac = trajopt_ifopt::calcNumericalConstraintGradient(*variables, jerk_cnt);

  EXPECT_EQ(jac.rows(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());
  EXPECT_EQ(jac.cols(), static_cast<Eigen::Index>(x_vals.size()) * targets.size());

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++i)
  {
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(x_vals.size()) * targets.size(); ++j)
    {
      EXPECT_NEAR(jac.coeffRef(i, j), num_jac.coeffRef(i, j), 1e-3);
    }
  }
}
////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
