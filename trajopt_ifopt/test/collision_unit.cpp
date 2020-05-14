#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract/tesseract.h>

#include <tesseract_environment/core/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_scene_graph/utils.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_test_utils.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_ifopt/constraints/collision_constraint.h>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

class CollisionUnit : public testing::TestWithParam<const char*>
{
public:
  Tesseract::Ptr tesseract_ = std::make_shared<Tesseract>(); /**< Tesseract */
  trajopt::SingleTimestepCollisionEvaluator::Ptr collision_evaluator_;
  ifopt::Problem nlp_;
  std::shared_ptr<trajopt::CollisionConstraintIfopt> constraint_;

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(tesseract_->init(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;

    // Set up collision evaluator
    auto env = tesseract_->getEnvironmentConst();
    auto kin = tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
    auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

    double margin_coeff = 1;
    double margin = 0.1;
    trajopt::SafetyMarginData::ConstPtr margin_data = std::make_shared<trajopt::SafetyMarginData>(margin, margin_coeff);
    double safety_margin_buffer = 0.00;
    sco::VarVector var_vector;  // unused

    collision_evaluator_ = std::make_shared<trajopt::SingleTimestepCollisionEvaluator>(
        kin,
        env,
        adj_map,
        Eigen::Isometry3d::Identity(),
        margin_data,
        tesseract_collision::ContactTestType::CLOSEST,
        var_vector,
        trajopt::CollisionExpressionEvaluatorType::SINGLE_TIME_STEP,
        safety_margin_buffer);

    // 3) Add Variables
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    auto var0 = std::make_shared<trajopt::JointPosition>(pos, kin->getJointNames(), "Joint_Position_0");
    nlp_.AddVariableSet(var0);

    constraint_ = std::make_shared<trajopt::CollisionConstraintIfopt>(collision_evaluator_, var0);
    nlp_.AddConstraintSet(constraint_);
  }
};

/**
 * @brief Tests that GetValues and FillJacobianBlock return the correct values
 */
TEST_F(CollisionUnit, GetValueFillJacobian)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CollisionUnit, GetValueFillJacobian");

  // Not in collision. Outside buffer
  {
    Eigen::VectorXd pos(2);
    // Small offset in y is to avoid numeric instabilites with perfectly aligning edges
    pos << -1.9, 0.01;
    nlp_.SetVariables(pos.data());
    Eigen::VectorXd values = constraint_->GetValues();
    EXPECT_NEAR(values[0], 0.0, 1e-6);

    ifopt::ConstraintSet::Jacobian jac_block;
    jac_block.resize(1, 2);
    constraint_->FillJacobianBlock("Joint_Position_0", jac_block);
    double dx = jac_block.coeff(0, 0);
    double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 0.0, 1e-6);
    EXPECT_NEAR(dy, 0.0, 1e-6);
  }

  // Not in collision. Within buffer
  {
    Eigen::VectorXd pos(2);
    pos << -1.0, 0.01;
    nlp_.SetVariables(pos.data());
    Eigen::VectorXd values = constraint_->GetValues();
    EXPECT_NEAR(values[0], 0.1, 1e-6);

    ifopt::ConstraintSet::Jacobian jac_block;
    jac_block.resize(1, 2);
    constraint_->FillJacobianBlock("Joint_Position_0", jac_block);
    double dx = jac_block.coeff(0, 0);
    double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 1.0, 1e-6);
    EXPECT_NEAR(dy, 0.0, 1e-6);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0.01, 1.0;
    nlp_.SetVariables(pos.data());
    Eigen::VectorXd values = constraint_->GetValues();
    EXPECT_NEAR(values[0], 0.1, 1e-6);

    ifopt::ConstraintSet::Jacobian jac_block;
    jac_block.resize(1, 2);
    constraint_->FillJacobianBlock("Joint_Position_0", jac_block);
    double dx = jac_block.coeff(0, 0);
    double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 0.0, 1e-6);
    EXPECT_NEAR(dy, -1.0, 1e-6);
  }

  // In collision
  {
    Eigen::VectorXd pos(2);
    pos << -0.9, 0.01;
    nlp_.SetVariables(pos.data());
    Eigen::VectorXd values = constraint_->GetValues();
    EXPECT_NEAR(values[0], 0.2, 1e-6);

    ifopt::ConstraintSet::Jacobian jac_block;
    jac_block.resize(1, 2);
    constraint_->FillJacobianBlock("Joint_Position_0", jac_block);
    double dx = jac_block.coeff(0, 0);
    double dy = jac_block.coeff(0, 1);
    EXPECT_NEAR(dx, 1.0, 1e-6);
    EXPECT_NEAR(dy, 0.0, 1e-6);
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(CollisionUnit, GetSetBounds)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CollisionUnit, GetSetBounds");

  // Check that setting bounds works
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    std::vector<std::string> joint_names(2, "names");
    auto var0 = std::make_shared<trajopt::JointPosition>(pos, joint_names, "Joint_Position_0");

    auto constraint_2 = std::make_shared<trajopt::CollisionConstraintIfopt>(collision_evaluator_, var0);
    ifopt::Bounds bounds(-0.1234, 0.5678);
    std::vector<ifopt::Bounds> bounds_vec = std::vector<ifopt::Bounds>(1, bounds);
    constraint_2->SetBounds(bounds_vec);
    std::vector<ifopt::Bounds> results_vec = constraint_2->GetBounds();
    for (size_t i = 0; i < bounds_vec.size(); i++)
    {
      EXPECT_EQ(bounds_vec[i].lower_, results_vec[i].lower_);
      EXPECT_EQ(bounds_vec[i].upper_, results_vec[i].upper_);
    }
  }
}

/**
 * @brief Checks that the constraint doesn't change the jacobian when it shouldn't
 */
TEST_F(CollisionUnit, IgnoreVariables)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CollisionUnit, IgnoreVariables");

  // Check that jacobian does not change for variables it shouldn't
  {
    ifopt::ConstraintSet::Jacobian jac_block_input;
    jac_block_input.resize(1, 2);
    constraint_->FillJacobianBlock("another_var", jac_block_input);
    EXPECT_EQ(jac_block_input.nonZeros(), 0);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
