/**
 * @file cost_wrappers_unit.cpp
 * @brief Test the cost wrappers unit
 *
 * @author Levi Armstrong
 * @date May 20, 2021
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
#include <Eigen/Eigen>
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/costs/absolute_cost.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

/**
 * @brief The SimpleTestConstraint class
 * @details
 *    y(x) = x^2 + 4x + 3
 *   dy(x) = 2x + 4
 */
class SimpleTestConstraint : public trajopt_ifopt::ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleTestConstraint(std::shared_ptr<const trajopt_ifopt::Var> position_var,
                       std::string name = "SimpleTestConstraint")
    : ConstraintSet(std::move(name), 1), position_var_(std::move(position_var))
  {
    coeffs_ = Eigen::VectorXd::Constant(1, 1);
    bounds_ = std::vector<trajopt_ifopt::Bounds>(1, trajopt_ifopt::BoundZero);
  }

  int update() final { return rows_; }

  Eigen::VectorXd getValues() const final
  {
    Eigen::VectorXd output(1);
    double v = position_var_->value()(0);
    output(0) = std::pow(v, 2) + (4 * v) + 3;
    return output;
  }

  Eigen::VectorXd getCoefficients() const final { return coeffs_; }

  std::vector<trajopt_ifopt::Bounds> getBounds() const final { return bounds_; }

  void fillJacobianBlock(trajopt_ifopt::Jacobian& jac_block, const std::string& var_set) const final
  {
    // Only modify the jacobian if this constraint uses var_set
    if (var_set != position_var_->getParent()->getParent()->getName())  // NOLINT
      return;

    // dy = 2x + 4;
    // Reserve enough room in the sparse matrix
    jac_block.reserve(1);
    jac_block.coeffRef(0, position_var_->getIndex()) = (2 * position_var_->value()(0)) + 4;
  }

private:
  /** @brief The constraint coefficients */
  Eigen::VectorXd coeffs_;

  /** @brief Bounds on the positions of each joint */
  std::vector<trajopt_ifopt::Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint.
   *
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->getName())->getValues()*/
  std::shared_ptr<const trajopt_ifopt::Var> position_var_;
};

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, SquaredCost)  // NOLINT
{
  // 2) Create the problem
  trajopt_ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::shared_ptr<const trajopt_ifopt::Var> var;
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Node");
    Eigen::VectorXd pos(1);
    pos << -4;
    const std::vector<std::string> var_names = { "x" };
    var = node->addVar("position", { "x" }, pos, { trajopt_ifopt::NoBound });
    nodes.push_back(std::move(node));
  }
  nlp.addVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint-trajectory", std::move(nodes)));

  // 4) add cost
  auto cnt = std::make_shared<SimpleTestConstraint>(var);
  cnt->linkWithVariables(nlp.getOptVariables());
  auto cost = std::make_shared<trajopt_ifopt::SquaredCost>(cnt);
  nlp.addCostSet(cost);

  auto exact_jac = nlp.evaluateCostFunctionGradient(var->value().data());
  auto numerical_jac = trajopt_ifopt::calcNumericalCostGradient(var->value().data(), nlp, 1e-8);

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Squared cost jacobian = 2 * y(x) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  const double x = var->value()(0);
  const double jac = 2 * (std::pow(x, 2) + 4 * x + 3) * (2 * x + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, WeightedSquaredCost)  // NOLINT
{
  // 2) Create the problem
  trajopt_ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::shared_ptr<const trajopt_ifopt::Var> var;
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Node");
    Eigen::VectorXd pos(1);
    pos << -4;
    const std::vector<std::string> var_names = { "x" };
    var = node->addVar("position", { "x" }, pos, { trajopt_ifopt::NoBound });
    nodes.push_back(std::move(node));
  }
  nlp.addVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint-trajectory", std::move(nodes)));

  // 4) add cost
  Eigen::VectorXd weights(1);
  weights(0) = 2;
  auto cnt = std::make_shared<SimpleTestConstraint>(var);
  cnt->linkWithVariables(nlp.getOptVariables());
  auto cost = std::make_shared<trajopt_ifopt::SquaredCost>(cnt, weights);
  nlp.addCostSet(cost);

  auto exact_jac = nlp.evaluateCostFunctionGradient(var->value().data());
  auto numerical_jac = trajopt_ifopt::calcNumericalCostGradient(var->value().data(), nlp, 1e-8);

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Squared cost jacobian = 2 * y(x) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  const double x = var->value()(0);
  const double jac = 2 * weights(0) * (std::pow(x, 2) + 4 * x + 3) * (2 * x + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, AbsoluteCost)  // NOLINT
{
  // 2) Create the problem
  trajopt_ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::shared_ptr<const trajopt_ifopt::Var> var;
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Node");
    Eigen::VectorXd pos(1);
    pos << -4;
    const std::vector<std::string> var_names = { "x" };
    var = node->addVar("position", { "x" }, pos, { trajopt_ifopt::NoBound });
    nodes.push_back(std::move(node));
  }
  nlp.addVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint-trajectory", std::move(nodes)));

  // 4) add cost
  auto cnt = std::make_shared<SimpleTestConstraint>(var);
  cnt->linkWithVariables(nlp.getOptVariables());
  auto cost = std::make_shared<trajopt_ifopt::AbsoluteCost>(cnt);
  nlp.addCostSet(cost);

  auto exact_jac = nlp.evaluateCostFunctionGradient(var->value().data());
  auto numerical_jac = trajopt_ifopt::calcNumericalCostGradient(var->value().data(), nlp, 1e-8);

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Absolute cost jacobian = (y(x) / |y(x)|) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  const double x = var->value()(0);
  const double jac = ((std::pow(x, 2) + (4 * x) + 3) / std::abs(std::pow(x, 2) + (4 * x) + 3)) * ((2 * x) + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, WeightedAbsoluteCost)  // NOLINT
{
  // 2) Create the problem
  trajopt_ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  std::shared_ptr<const trajopt_ifopt::Var> var;
  {
    auto node = std::make_unique<trajopt_ifopt::Node>("Node");
    Eigen::VectorXd pos(1);
    pos << -4;
    const std::vector<std::string> var_names = { "x" };
    var = node->addVar("position", { "x" }, pos, { trajopt_ifopt::NoBound });
    nodes.push_back(std::move(node));
  }
  nlp.addVariableSet(std::make_shared<trajopt_ifopt::NodesVariables>("joint-trajectory", std::move(nodes)));

  // 4) add cost
  Eigen::VectorXd weights(1);
  weights(0) = 2;
  auto cnt = std::make_shared<SimpleTestConstraint>(var);
  cnt->linkWithVariables(nlp.getOptVariables());
  auto cost = std::make_shared<trajopt_ifopt::AbsoluteCost>(cnt, weights);
  nlp.addCostSet(cost);

  auto exact_jac = nlp.evaluateCostFunctionGradient(var->value().data());
  auto numerical_jac = trajopt_ifopt::calcNumericalCostGradient(var->value().data(), nlp, 1e-8);  // NOLINT

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Absolute cost jacobian = (y(x) / |y(x)|) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  const double x = var->value()(0);
  const double jac =
      (weights(0) * (std::pow(x, 2) + (4 * x) + 3) / std::abs(std::pow(x, 2) + (4 * x) + 3)) * ((2 * x) + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
