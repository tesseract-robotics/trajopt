/**
 * @file joint_term_unit.cpp
 * @brief The joint position constraint unit test
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
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
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

using namespace trajopt_ifopt;
using namespace std;

/** @brief Tests the Joint Position Constraint */
TEST(JointTermsUnit, JointPosConstraintUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointPosConstraintUnit");

  const std::vector<std::string> joint_names(10, "name");
  Eigen::VectorXd init_vals(10);
  init_vals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  std::vector<Bounds> bounds(10, NoBound);

  auto node = std::make_unique<Node>();
  std::shared_ptr<const Var> position_var = node->addVar("state", joint_names, init_vals, bounds);
  std::vector<std::unique_ptr<Node>> nodes;
  nodes.push_back(std::move(node));

  auto variables = std::make_shared<CompositeVariables>("variable-sets");
  variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  std::vector<Eigen::VectorXd> targets;
  Eigen::VectorXd target(10);
  target << 20, 21, 22, 23, 24, 25, 26, 27, 28, 29;

  const std::string name("test_cnt");
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(10, 1);
  JointPosConstraint position_cnt(target, position_var, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  position_cnt.linkWithVariables(variables);

  EXPECT_EQ(position_cnt.getRows(), static_cast<std::size_t>(target.size()));
  EXPECT_EQ(position_cnt.getName(), name);
  EXPECT_EQ(position_cnt.getBounds().size(), static_cast<std::size_t>(target.size()));

  Eigen::VectorXd position_vals = position_cnt.getValues();
  EXPECT_EQ(position_vals.size(), static_cast<std::size_t>(target.size()));

  // Test forward diff
  EXPECT_TRUE(position_vals.isApprox(init_vals, 1e-6));

  Jacobian jac = position_cnt.getJacobian();
  Jacobian num_jac = calcNumericalConstraintGradient(*variables, position_cnt);

  EXPECT_EQ(jac.rows(), target.size());
  EXPECT_EQ(jac.cols(), target.size());

  for (Eigen::Index i = 0; i < target.size(); ++i)
  {
    for (Eigen::Index j = 0; j < target.size(); ++j)
    {
      EXPECT_NEAR(jac.coeffRef(i, j), num_jac.coeffRef(i, j), 1e-3);
    }
  }
}

/** @brief Tests the Joint Velocity Constraint */
TEST(JointTermsUnit, JointVelConstraintUnit)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointVelConstraintUnit");

  // y = x^3 + 5*x^2 + 2*x + 1
  auto f = [](double x) { return ((x * x * x) + (5 * x * x) + (2 * x) + 1); };

  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> position_vars;
  position_vars.reserve(27);

  std::vector<int> x_vals;
  x_vals.reserve(27);
  for (int i = -13; i < 14; ++i)
  {
    auto node = std::make_unique<Node>();
    const std::string var_id = "test_var_" + std::to_string(position_vars.size());
    const std::vector<std::string> joint_names{ "x", "y" };
    std::vector<Bounds> bounds(2, NoBound);
    Eigen::VectorXd vals(2);
    vals << f(i), f(i);
    position_vars.push_back(node->addVar(var_id, joint_names, vals, bounds));
    nodes.push_back(std::move(node));

    x_vals.push_back(i);
  }
  auto variables = std::make_shared<CompositeVariables>("variable-sets");
  variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  const std::string name("test_joint_vel_cnt");
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointVelConstraint velocity_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  velocity_cnt.linkWithVariables(variables);

  EXPECT_EQ(velocity_cnt.getRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));
  EXPECT_EQ(velocity_cnt.getName(), name);
  EXPECT_EQ(velocity_cnt.getBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  Eigen::VectorXd velocity_vals = velocity_cnt.getValues();
  EXPECT_EQ(velocity_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 1; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = (f(x_vals[i + 1]) - f(x_vals[i]));
      EXPECT_NEAR(velocity_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  Jacobian jac = velocity_cnt.getJacobian();
  Jacobian num_jac = calcNumericalConstraintGradient(*variables, velocity_cnt);
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
  auto f = [](double x) { return ((x * x * x) + (5 * x * x) + (2 * x) + 1); };

  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> position_vars;
  position_vars.reserve(2);

  std::vector<int> x_vals;
  x_vals.reserve(2);
  for (int i = -13; i < -11; ++i)
  {
    auto node = std::make_unique<Node>();
    const std::string var_id = "test_var_" + std::to_string(position_vars.size());
    const std::vector<std::string> joint_names{ "x", "y" };
    std::vector<Bounds> bounds(2, NoBound);
    Eigen::VectorXd vals(2);
    vals << f(i), f(i);
    position_vars.push_back(node->addVar(var_id, joint_names, vals, bounds));
    nodes.push_back(std::move(node));

    x_vals.push_back(i);
  }

  auto variables = std::make_shared<CompositeVariables>("variable-sets");
  variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  const std::string name("test_joint_vel_cnt");
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointVelConstraint velocity_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  velocity_cnt.linkWithVariables(variables);

  EXPECT_EQ(velocity_cnt.getRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));
  EXPECT_EQ(velocity_cnt.getName(), name);
  EXPECT_EQ(velocity_cnt.getBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  Eigen::VectorXd velocity_vals = velocity_cnt.getValues();
  EXPECT_EQ(velocity_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size() - 1));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 1; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = (f(x_vals[i + 1]) - f(x_vals[i]));
      EXPECT_NEAR(velocity_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  Jacobian jac = velocity_cnt.getJacobian();
  Jacobian num_jac = calcNumericalConstraintGradient(*variables, velocity_cnt);

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
  auto f = [](double x) { return ((x * x * x) + (5 * x * x) + (2 * x) + 1); };

  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> position_vars;
  position_vars.reserve(27);

  std::vector<int> x_vals;
  x_vals.reserve(27);
  for (int i = -13; i < 14; ++i)
  {
    auto node = std::make_unique<Node>();
    const std::string var_id = "test_var_" + std::to_string(position_vars.size());
    const std::vector<std::string> joint_names{ "x", "y" };
    std::vector<Bounds> bounds(2, NoBound);
    Eigen::VectorXd vals(2);
    vals << f(i), f(i);
    position_vars.push_back(node->addVar(var_id, joint_names, vals, bounds));
    nodes.push_back(std::move(node));

    x_vals.push_back(i);
  }

  auto variables = std::make_shared<CompositeVariables>("variable-sets");
  variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  const std::string name("test_joint_accel_cnt");
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointAccelConstraint accel_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  accel_cnt.linkWithVariables(variables);

  EXPECT_EQ(accel_cnt.getRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(accel_cnt.getName(), name);
  EXPECT_EQ(accel_cnt.getBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd accel_vals = accel_cnt.getValues();
  EXPECT_EQ(accel_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 2; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = (f(x_vals[i]) - (2.0 * f(x_vals[i + 1])) + f(x_vals[i + 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 2; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = (f(x_vals[i]) - (2.0 * f(x_vals[i - 1])) + f(x_vals[i - 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  Jacobian jac = accel_cnt.getJacobian();
  Jacobian num_jac = calcNumericalConstraintGradient(*variables, accel_cnt);

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
  auto f = [](double x) { return ((x * x * x) + (5 * x * x) + (2 * x) + 1); };

  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> position_vars;
  position_vars.reserve(4);

  std::vector<int> x_vals;
  x_vals.reserve(4);
  for (int i = -13; i < -9; ++i)
  {
    auto node = std::make_unique<Node>();
    const std::string var_id = "test_var_" + std::to_string(position_vars.size());
    const std::vector<std::string> joint_names{ "x", "y" };
    std::vector<Bounds> bounds(2, NoBound);
    Eigen::VectorXd vals(2);
    vals << f(i), f(i);
    position_vars.push_back(node->addVar(var_id, joint_names, vals, bounds));
    nodes.push_back(std::move(node));

    x_vals.push_back(i);
  }

  auto variables = std::make_shared<CompositeVariables>("variable-sets");
  variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  const std::string name("test_joint_accel_cnt");
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointAccelConstraint accel_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  accel_cnt.linkWithVariables(variables);

  EXPECT_EQ(accel_cnt.getRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(accel_cnt.getName(), name);
  EXPECT_EQ(accel_cnt.getBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd accel_vals = accel_cnt.getValues();
  EXPECT_EQ(accel_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 2; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = (f(x_vals[i]) - (2.0 * f(x_vals[i + 1])) + f(x_vals[i + 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 2; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = (f(x_vals[i]) - (2.0 * f(x_vals[i - 1])) + f(x_vals[i - 2]));
      EXPECT_NEAR(accel_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  Jacobian jac = accel_cnt.getJacobian();
  Jacobian num_jac = calcNumericalConstraintGradient(*variables, accel_cnt);

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
  auto f = [](double x) { return ((x * x * x) + (5 * x * x) + (2 * x) + 1); };

  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> position_vars;
  position_vars.reserve(27);

  std::vector<int> x_vals;
  x_vals.reserve(27);
  for (int i = -13; i < 14; ++i)
  {
    auto node = std::make_unique<Node>();
    const std::string var_id = "test_var_" + std::to_string(position_vars.size());
    const std::vector<std::string> joint_names{ "x", "y" };
    std::vector<Bounds> bounds(2, NoBound);
    Eigen::VectorXd vals(2);
    vals << f(i), f(i);
    position_vars.push_back(node->addVar(var_id, joint_names, vals, bounds));
    nodes.push_back(std::move(node));

    x_vals.push_back(i);
  }

  auto variables = std::make_shared<CompositeVariables>("variable-sets");
  variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  const std::string name("test_joint_jerk_cnt");
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointJerkConstraint jerk_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  jerk_cnt.linkWithVariables(variables);

  EXPECT_EQ(jerk_cnt.getRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(jerk_cnt.getName(), name);
  EXPECT_EQ(jerk_cnt.getBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd jerk_vals = jerk_cnt.getValues();
  EXPECT_EQ(jerk_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 3; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val =
          (-f(x_vals[i]) + (3.0 * f(x_vals[i + 1])) - (3.0 * f(x_vals[i + 2]))) + f(x_vals[i + 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 3; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = f(x_vals[i]) - (3.0 * f(x_vals[i - 1])) + (3.0 * f(x_vals[i - 2])) - f(x_vals[i - 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  Jacobian jac = jerk_cnt.getJacobian();
  Jacobian num_jac = calcNumericalConstraintGradient(*variables, jerk_cnt);

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
  auto f = [](double x) { return ((x * x * x) + (5 * x * x) + (2 * x) + 1); };

  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::shared_ptr<const Var>> position_vars;
  position_vars.reserve(6);

  std::vector<int> x_vals;
  x_vals.reserve(6);
  for (int i = -13; i < -7; ++i)
  {
    auto node = std::make_unique<Node>();
    const std::string var_id = "test_var_" + std::to_string(position_vars.size());
    const std::vector<std::string> joint_names{ "x", "y" };
    std::vector<Bounds> bounds(2, NoBound);
    Eigen::VectorXd vals(2);
    vals << f(i), f(i);
    position_vars.push_back(node->addVar(var_id, joint_names, vals, bounds));
    nodes.push_back(std::move(node));

    x_vals.push_back(i);
  }

  auto variables = std::make_shared<CompositeVariables>("variable-sets");
  variables->addComponent(std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes)));

  Eigen::VectorXd targets(2);
  targets << 0, 0;
  const std::string name("test_joint_jerk_cnt");
  const Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
  JointJerkConstraint jerk_cnt(targets, position_vars, coeffs, name);

  // Must link with variables or GetValues and GetJacobian throw exception.
  jerk_cnt.linkWithVariables(variables);

  EXPECT_EQ(jerk_cnt.getRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(jerk_cnt.getName(), name);
  EXPECT_EQ(jerk_cnt.getBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  Eigen::VectorXd jerk_vals = jerk_cnt.getValues();
  EXPECT_EQ(jerk_vals.size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));

  // Test forward diff
  for (std::size_t i = 0; i < x_vals.size() - 3; ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = (-f(x_vals[i]) + 3.0 * f(x_vals[i + 1]) - 3.0 * f(x_vals[i + 2])) + f(x_vals[i + 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  // Backward diff for last points
  for (std::size_t i = x_vals.size() - 3; i < x_vals.size(); ++i)
  {
    for (Eigen::Index j = 0; j < targets.size(); ++j)
    {
      const double expected_val = f(x_vals[i]) - (3.0 * f(x_vals[i - 1])) + (3.0 * f(x_vals[i - 2])) - f(x_vals[i - 3]);
      EXPECT_NEAR(jerk_vals((static_cast<Eigen::Index>(i) * targets.size()) + j), expected_val, 1e-6);
    }
  }

  Jacobian jac = jerk_cnt.getJacobian();
  Jacobian num_jac = calcNumericalConstraintGradient(*variables, jerk_cnt);

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
