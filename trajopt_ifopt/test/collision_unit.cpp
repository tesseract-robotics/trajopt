/**
 * @file collision_unit.cpp
 * @brief The collision constraint unit test
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
#include <tesseract/common/logging.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/common/types.h>
#include <trajopt_common/collision_types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_numerical_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_numerical_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

using namespace trajopt_ifopt;
using namespace tesseract::environment;
using namespace tesseract::kinematics;
using namespace tesseract::collision;
using namespace tesseract::scene_graph;
using namespace tesseract::geometry;
using namespace tesseract::common;

class CollisionUnit : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  DiscreteCollisionEvaluator::Ptr collision_evaluator;
  std::shared_ptr<Problem> nlp;
  std::shared_ptr<DiscreteCollisionConstraint> constraint;

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    EXPECT_TRUE(env->init(urdf_file, srdf_file, locator));

    // Set up collision evaluator
    const JointGroup::ConstPtr kin = env->getJointGroup("manipulator");
    trajopt_common::TrajOptCollisionConfig config(0.1, 1);

    collision_evaluator = std::make_shared<SingleTimestepCollisionEvaluator>(kin, env, config);

    // 3) Add Variables
    const std::vector<Bounds> bounds(static_cast<std::size_t>(kin->numJoints()), NoBound);
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    auto node = std::make_unique<Node>("Joint_Position_0");
    const std::vector<std::string> joint_names = tesseract::common::toNames(kin->getJointIds());
    auto var0 = node->addVar("position", joint_names, pos, bounds);

    std::vector<std::unique_ptr<Node>> nodes;
    nodes.push_back(std::move(node));
    auto variables = std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes));
    nlp = std::make_shared<Problem>(variables);

    constraint = std::make_shared<DiscreteCollisionConstraint>(collision_evaluator, var0, 1);
    nlp->addConstraintSet(constraint);
  }
};

/**
 * @brief Tests that GetValues and FillJacobianBlock return the correct values
 */
TEST_F(CollisionUnit, GetValueFillJacobian)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CollisionUnit, GetValueFillJacobian");

  // Not in collision. Outside buffer
  {
    Eigen::VectorXd pos(2);
    // Small offset in y is to avoid numeric instabilites with perfectly aligning edges
    pos << -1.9, 0.01;
    nlp->setVariables(pos.data());
    Eigen::VectorXd values = constraint->getValues();
    EXPECT_NEAR(values[0], -1 * collision_evaluator->getCollisionMarginBuffer(), 1e-6);

    Jacobian jac_block = constraint->getJacobian();
    const double dx = jac_block.coeff(0, 0);
    const double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 0.0, 1e-6);
    EXPECT_NEAR(dy, 0.0, 1e-6);
  }

  // Not in collision. Within buffer
  {
    Eigen::VectorXd pos(2);
    pos << -1.0, 0.01;
    nlp->setVariables(pos.data());
    Eigen::VectorXd values = constraint->getValues();
    EXPECT_NEAR(values[0], 0.1, 1e-6);

    Jacobian jac_block = constraint->getJacobian();
    const double dx = jac_block.coeff(0, 0);
    const double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 1.0, 1e-6);
    EXPECT_NEAR(dy, 0.0, 1e-6);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0.01, 1.0;
    nlp->setVariables(pos.data());
    Eigen::VectorXd values = constraint->getValues();
    EXPECT_NEAR(values[0], 0.1, 1e-6);

    Jacobian jac_block = constraint->getJacobian();
    const double dx = jac_block.coeff(0, 0);
    const double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 0.0, 1e-6);
    EXPECT_NEAR(dy, -1.0, 1e-6);
  }

  // In collision
  {
    Eigen::VectorXd pos(2);
    pos << -0.9, 0.01;
    nlp->setVariables(pos.data());
    Eigen::VectorXd values = constraint->getValues();
    EXPECT_NEAR(values[0], 0.2, 1e-6);

    Jacobian jac_block = constraint->getJacobian();
    const double dx = jac_block.coeff(0, 0);
    const double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 1.0, 1e-6);
    EXPECT_NEAR(dy, 0.0, 1e-6);
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(CollisionUnit, GetSetBounds)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CollisionUnit, GetSetBounds");

  // Check that setting bounds works
  {
    std::vector<Bounds> bounds_vec{ NoBound, NoBound };
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    const std::vector<std::string> joint_names(2, "names");
    auto node = std::make_unique<Node>("Joint_Position_0");
    auto var0 = node->addVar("position", joint_names, pos, bounds_vec);

    auto constraint_2 = std::make_shared<DiscreteCollisionConstraint>(collision_evaluator, var0, 3);
    const Bounds bounds(-0.1234, 0.5678);
    bounds_vec = std::vector<Bounds>(3, bounds);
    constraint_2->setBounds(bounds_vec);
    std::vector<Bounds> results_vec = constraint_2->getBounds();
    for (std::size_t i = 0; i < bounds_vec.size(); i++)
    {
      EXPECT_EQ(bounds_vec[i].getLower(), results_vec[i].getLower());
      EXPECT_EQ(bounds_vec[i].getUpper(), results_vec[i].getUpper());
    }
  }
}

/**
 * @brief The getters of a linked collision constraint throw until update() computes the collision data
 */
TEST_F(CollisionUnit, GettersThrowBeforeUpdate)  // NOLINT
{
  TESSERACT_LOG_DEBUG("CollisionUnit, GettersThrowBeforeUpdate");

  const JointGroup::ConstPtr kin = env->getJointGroup("manipulator");
  const std::vector<std::string> joint_names = tesseract::common::toNames(kin->getJointIds());
  const std::vector<Bounds> bounds(static_cast<std::size_t>(kin->numJoints()), NoBound);

  std::vector<std::unique_ptr<Node>> nodes;
  nodes.push_back(std::make_unique<Node>("Joint_Position_0"));
  auto var0 = nodes.back()->addVar("position", joint_names, Eigen::Vector2d(-1.9, 0.01), bounds);
  nodes.push_back(std::make_unique<Node>("Joint_Position_1"));
  auto var1 = nodes.back()->addVar("position", joint_names, Eigen::Vector2d(-1.0, 0.01), bounds);
  auto variables = std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes));

  trajopt_common::TrajOptCollisionConfig lvs_config(0.1, 1);
  lvs_config.collision_check_config.type = CollisionEvaluatorType::LVS_DISCRETE;
  auto lvs_evaluator = std::make_shared<LVSDiscreteCollisionEvaluator>(kin, env, lvs_config);
  const std::array<std::shared_ptr<const Var>, 2> position_vars{ var0, var1 };

  const std::vector<std::shared_ptr<ConstraintSet>> constraints{
    std::make_shared<DiscreteCollisionConstraint>(collision_evaluator, var0),
    std::make_shared<DiscreteCollisionConstraintD>(collision_evaluator, var0),
    std::make_shared<DiscreteCollisionNumericalConstraint>(collision_evaluator, var0),
    std::make_shared<ContinuousCollisionConstraint>(lvs_evaluator, position_vars, false, false),
    std::make_shared<ContinuousCollisionConstraintD>(lvs_evaluator, position_vars, false, false),
    std::make_shared<ContinuousCollisionNumericalConstraint>(lvs_evaluator, position_vars, false, false),
  };

  for (const auto& cnt : constraints)
  {
    SCOPED_TRACE(cnt->getName());
    cnt->linkWithVariables(variables);

    // The D variants return the values update() cached and never read the collision data
    const bool caches_values = (std::dynamic_pointer_cast<DiscreteCollisionConstraintD>(cnt) != nullptr) ||
                               (std::dynamic_pointer_cast<ContinuousCollisionConstraintD>(cnt) != nullptr);
    if (caches_values)
      EXPECT_NO_THROW(cnt->getValues());  // NOLINT
    else
      EXPECT_THROW(cnt->getValues(), std::runtime_error);  // NOLINT
    EXPECT_THROW(cnt->getJacobian(), std::runtime_error);  // NOLINT

    try
    {
      cnt->getJacobian();
    }
    catch (const std::runtime_error& e)
    {
      EXPECT_EQ(std::string(e.what()).rfind(cnt->getName() + ":", 0), 0);
      EXPECT_NE(std::string(e.what()).find("call update() first"), std::string::npos);
    }

    cnt->update();
    EXPECT_NO_THROW(cnt->getValues());    // NOLINT
    EXPECT_NO_THROW(cnt->getJacobian());  // NOLINT
  }

  // The numerical constraint's block functions take explicit joint values, but compare against the cached data
  auto numerical = std::make_shared<DiscreteCollisionNumericalConstraint>(collision_evaluator, var0);
  numerical->linkWithVariables(variables);
  Jacobian jac_block(numerical->getRows(), variables->getRows());
  EXPECT_THROW(numerical->calcValues(var0->value()), std::runtime_error);                    // NOLINT
  EXPECT_THROW(numerical->calcJacobianBlock(jac_block, var0->value()), std::runtime_error);  // NOLINT
  numerical->update();
  EXPECT_NO_THROW(numerical->calcValues(var0->value()));  // NOLINT
  Jacobian jac_block_updated(numerical->getRows(), variables->getRows());
  EXPECT_NO_THROW(numerical->calcJacobianBlock(jac_block_updated, var0->value()));  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
