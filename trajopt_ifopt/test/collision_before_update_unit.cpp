#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <tesseract/common/types.h>
#include <trajopt_common/collision_types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/problem.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_numerical_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_numerical_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>

using namespace trajopt_ifopt;
using namespace tesseract::environment;
using namespace tesseract::kinematics;
using namespace tesseract::common;

/**
 * @brief Each test links one constraint with the problem's variables but never calls update(), so the constraint has
 * no collision data yet. Its getters must throw instead of dereferencing the missing data.
 */
class CollisionBeforeUpdateUnit : public testing::Test
{
public:
  Environment::Ptr env = std::make_shared<Environment>();
  JointGroup::ConstPtr manip;
  std::shared_ptr<NodesVariables> variables;
  std::array<std::shared_ptr<const Var>, 2> vars;
  std::shared_ptr<Problem> nlp;

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");
    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    ASSERT_TRUE(env->init(urdf_file, srdf_file, locator));

    manip = env->getJointGroup("manipulator");
    const std::vector<Bounds> bounds(static_cast<std::size_t>(manip->numJoints()), NoBound);
    const std::vector<std::string> joint_names = tesseract::common::toNames(manip->getJointIds());

    std::vector<std::unique_ptr<Node>> nodes;
    for (std::size_t i = 0; i < vars.size(); ++i)
    {
      Eigen::VectorXd pos(2);
      pos << -1.9 + static_cast<double>(i), 0;
      nodes.push_back(std::make_unique<Node>("Joint_Position_" + std::to_string(i)));
      vars.at(i) = nodes.back()->addVar("position", joint_names, pos, bounds);
    }
    variables = std::make_shared<NodesVariables>("joint_trajectory", std::move(nodes));
    nlp = std::make_shared<Problem>(variables);
  }

  std::shared_ptr<DiscreteCollisionEvaluator> discreteEvaluator() const
  {
    const trajopt_common::TrajOptCollisionConfig config(0.1, 1);
    return std::make_shared<SingleTimestepCollisionEvaluator>(manip, env, config);
  }

  std::shared_ptr<ContinuousCollisionEvaluator> continuousEvaluator() const
  {
    trajopt_common::TrajOptCollisionConfig config(0.1, 1);
    config.collision_check_config.type = tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    return std::make_shared<LVSContinuousCollisionEvaluator>(manip, env, config);
  }
};

TEST_F(CollisionBeforeUpdateUnit, DiscreteCollisionConstraint)  // NOLINT
{
  auto constraint = std::make_shared<DiscreteCollisionConstraint>(discreteEvaluator(), vars[0]);
  nlp->addConstraintSet(constraint);
  EXPECT_THROW(constraint->getValues(), std::runtime_error);    // NOLINT
  EXPECT_THROW(constraint->getJacobian(), std::runtime_error);  // NOLINT
}

TEST_F(CollisionBeforeUpdateUnit, DiscreteCollisionConstraintD)  // NOLINT
{
  auto constraint = std::make_shared<DiscreteCollisionConstraintD>(discreteEvaluator(), vars[0]);
  nlp->addConstraintSet(constraint);
  EXPECT_THROW(constraint->getJacobian(), std::runtime_error);  // NOLINT
}

TEST_F(CollisionBeforeUpdateUnit, DiscreteCollisionNumericalConstraint)  // NOLINT
{
  auto constraint = std::make_shared<DiscreteCollisionNumericalConstraint>(discreteEvaluator(), vars[0]);
  nlp->addConstraintSet(constraint);
  EXPECT_THROW(constraint->getValues(), std::runtime_error);    // NOLINT
  EXPECT_THROW(constraint->getJacobian(), std::runtime_error);  // NOLINT
}

TEST_F(CollisionBeforeUpdateUnit, ContinuousCollisionConstraint)  // NOLINT
{
  auto constraint = std::make_shared<ContinuousCollisionConstraint>(continuousEvaluator(), vars, false, false);
  nlp->addConstraintSet(constraint);
  EXPECT_THROW(constraint->getValues(), std::runtime_error);    // NOLINT
  EXPECT_THROW(constraint->getJacobian(), std::runtime_error);  // NOLINT
}

TEST_F(CollisionBeforeUpdateUnit, ContinuousCollisionConstraintD)  // NOLINT
{
  auto constraint = std::make_shared<ContinuousCollisionConstraintD>(continuousEvaluator(), vars, false, false);
  nlp->addConstraintSet(constraint);
  EXPECT_THROW(constraint->getJacobian(), std::runtime_error);  // NOLINT
}

TEST_F(CollisionBeforeUpdateUnit, ContinuousCollisionNumericalConstraint)  // NOLINT
{
  auto constraint = std::make_shared<ContinuousCollisionNumericalConstraint>(continuousEvaluator(), vars, false, false);
  nlp->addConstraintSet(constraint);
  EXPECT_THROW(constraint->getValues(), std::runtime_error);    // NOLINT
  EXPECT_THROW(constraint->getJacobian(), std::runtime_error);  // NOLINT
}

/** @brief The guard must not fire once update() has run */
TEST_F(CollisionBeforeUpdateUnit, DiscreteCollisionConstraintAfterUpdate)  // NOLINT
{
  auto constraint = std::make_shared<DiscreteCollisionConstraint>(discreteEvaluator(), vars[0]);
  nlp->addConstraintSet(constraint);
  nlp->setVariables(variables->getValues().data());
  EXPECT_NO_THROW(constraint->getValues());    // NOLINT
  EXPECT_NO_THROW(constraint->getJacobian());  // NOLINT
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
