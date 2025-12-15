/**
 * @file variable_sets_unit.cpp
 * @brief TrajOpt IFOPT variable sets unit tests
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
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <console_bridge/console.h>

// -----------------------
// Var tests
// -----------------------

TEST(VarUnit, ScalarVarConstructionAndAccess)
{
  // Single scalar at index 1
  trajopt_ifopt::Var v(0, "my_scalar", 1);

  EXPECT_EQ(v.getParent(), nullptr);
  EXPECT_EQ(v.getIndex(), 0);
  EXPECT_EQ(v.size(), 1);
  EXPECT_EQ(v.value().size(), 1);
  EXPECT_DOUBLE_EQ(v.value()(0), 1.0);
  EXPECT_EQ(v.getIdentifier(), "my_scalar");
  EXPECT_EQ(v.name()[0], "my_scalar");
}

TEST(VarUnit, VectorVarConstructionAndAccess)
{
  Eigen::VectorXd x(3);
  x << 10, 20.0, 30.0;

  const std::vector<std::string> names{ "a", "b", "c" };
  const std::vector<trajopt_ifopt::Bounds> bounds(3, trajopt_ifopt::NoBound);

  // Vector starts at index 1, length 3 -> x[1..3]
  trajopt_ifopt::Var v(0, "position", names, x, bounds);
  EXPECT_EQ(v.getParent(), nullptr);
  EXPECT_EQ(v.getIndex(), 0);
  EXPECT_EQ(v.size(), 3);

  const Eigen::VectorXd& values = v.value();
  ASSERT_EQ(values.size(), 3);
  EXPECT_DOUBLE_EQ(values[0], 10.0);
  EXPECT_DOUBLE_EQ(values[1], 20.0);
  EXPECT_DOUBLE_EQ(values[2], 30.0);

  EXPECT_EQ(v.getIdentifier(), "position");
  EXPECT_EQ(v.name(), names);
}

// -----------------------
// Node tests
// -----------------------

TEST(NodeUnit, NameParentAndVarManagement)
{
  trajopt_ifopt::Node node("node0");

  EXPECT_EQ(node.getName(), "node0");
  EXPECT_EQ(node.getParent(), nullptr);
  EXPECT_EQ(node.size(), 0);

  // Add scalar var
  auto scalar_var = node.addVar("s", 1);
  EXPECT_TRUE(static_cast<bool>(scalar_var));
  EXPECT_TRUE(node.hasVar("s"));
  EXPECT_EQ(node.getVar("s")->getIndex(), 0);
  EXPECT_EQ(node.size(), 1);

  // Add vector var with 3 components
  Eigen::VectorXd vector_var_pos(3);
  vector_var_pos << -1.0, 10.0, 20.0;

  const std::vector<std::string> names{ "a", "b", "c" };
  const std::vector<trajopt_ifopt::Bounds> bounds(3, trajopt_ifopt::NoBound);

  auto vector_var = node.addVar("v", names, vector_var_pos, bounds);
  EXPECT_TRUE(static_cast<bool>(vector_var));
  EXPECT_TRUE(node.hasVar("v"));
  EXPECT_EQ(node.getVar("v")->getIndex(), 1);
  EXPECT_EQ(node.size(), 1 + 3);

  // getVar should return the same objects
  auto fetched_scalar = node.getVar("s");
  auto fetched_vector = node.getVar("v");
  ASSERT_TRUE(static_cast<bool>(fetched_scalar));
  ASSERT_TRUE(static_cast<bool>(fetched_vector));

  // Check that setVariables routes segments correctly to each Var
  Eigen::VectorXd x(node.size());
  x << 0.5, 1.0, 2.0, 3.0;
  node.setVariables(x);

  EXPECT_DOUBLE_EQ(fetched_scalar->value()(0), 0.5);

  const Eigen::VectorXd& vec = fetched_vector->value();
  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);
}

// -----------------------
// NodesVariables tests
// -----------------------

TEST(NodesVariablesUnit, AddNodesAndGetNodes)
{
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;

  nodes.push_back(std::make_unique<trajopt_ifopt::Node>("n0"));
  nodes.back()->addVar("s0", 1);

  Eigen::VectorXd vector_var_pos(2);
  vector_var_pos << -1.0, 10.0;

  const std::vector<std::string> names{ "v0", "v1" };
  const std::vector<trajopt_ifopt::Bounds> bounds(2, trajopt_ifopt::NoBound);
  nodes.push_back(std::make_unique<trajopt_ifopt::Node>("n1"));
  nodes.back()->addVar("v", names, vector_var_pos, bounds);

  // Create variable set
  trajopt_ifopt::NodesVariables vars("nodes", std::move(nodes));

  auto out_nodes = vars.getNodes();
  ASSERT_EQ(out_nodes.size(), 2U);

  EXPECT_EQ(out_nodes[0]->getName(), "n0");
  EXPECT_EQ(out_nodes[1]->getName(), "n1");

  // Parent pointer on each node should be set
  EXPECT_EQ(out_nodes[0]->getParent(), &vars);
  EXPECT_EQ(out_nodes[1]->getParent(), &vars);

  // GetNode should be consistent with GetNodes (assume 0-based indexing)
  auto n0 = vars.getNode(0);
  auto n1 = vars.getNode(1);
  ASSERT_TRUE(static_cast<bool>(n0));
  ASSERT_TRUE(static_cast<bool>(n1));
  EXPECT_EQ(n0->getName(), "n0");
  EXPECT_EQ(n1->getName(), "n1");
}

TEST(NodesVariablesUnit, SetVariablesPropagatesToNodesAndVars)
{
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  // Node 0: one scalar
  nodes.push_back(std::make_unique<trajopt_ifopt::Node>("n0"));
  nodes.back()->addVar("s0", 1);

  // Node 1: one 2D vector
  Eigen::VectorXd vector_var_pos(2);
  vector_var_pos << -1.0, 10.0;

  const std::vector<std::string> names{ "v0", "v1" };
  const std::vector<trajopt_ifopt::Bounds> bounds(2, trajopt_ifopt::NoBound);
  nodes.push_back(std::make_unique<trajopt_ifopt::Node>("n1"));
  nodes.back()->addVar("v", names, vector_var_pos, bounds);

  // Create variable set
  trajopt_ifopt::NodesVariables vars("nodes", std::move(nodes));

  // Total variables: 1 (n0.s0) + 2 (n1.v) = 3
  Eigen::VectorXd x(3);
  x << 5.0, 6.0, 7.0;

  // Set into NodesVariables, which should update internal storage and nodes
  vars.setVariables(x);

  // GetValues should roundtrip the vector
  Eigen::VectorXd out = vars.getValues();
  ASSERT_EQ(out.size(), x.size());
  EXPECT_TRUE(out.isApprox(x));

  // Check that the corresponding node/var views see the same data
  auto n0 = vars.getNode(0);
  auto n1 = vars.getNode(1);
  ASSERT_TRUE(static_cast<bool>(n0));
  ASSERT_TRUE(static_cast<bool>(n1));

  auto n0_scalar = n0->getVar("s0");
  auto n1_vec = n1->getVar("v");
  ASSERT_TRUE(static_cast<bool>(n0_scalar));
  ASSERT_TRUE(static_cast<bool>(n1_vec));

  EXPECT_DOUBLE_EQ(n0_scalar->value()(0), 5.0);

  const Eigen::VectorXd& v_values = n1_vec->value();
  ASSERT_EQ(v_values.size(), 2);
  EXPECT_DOUBLE_EQ(v_values[0], 6.0);
  EXPECT_DOUBLE_EQ(v_values[1], 7.0);
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
