#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <console_bridge/console.h>
#include <ifopt/problem.h>

#include <tesseract/tesseract.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_test_utils.hpp>

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

class CartesianPositionConstraintUnit : public testing::TestWithParam<const char*>
{
public:
  tesseract::Tesseract::Ptr tesseract_ = std::make_shared<Tesseract>();
  ifopt::Problem nlp_;

  tesseract_kinematics::ForwardKinematics::Ptr forward_kinematics_;
  tesseract_kinematics::InverseKinematics::Ptr inverse_kinematics_;
  CartPosKinematicInfo::Ptr kinematic_info_;
  CartPosConstraint::Ptr constraint_;

  Eigen::Index n_dof_;

  void SetUp() override
  {
    // Initialize Tesseract
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    auto tesseract = std::make_shared<tesseract::Tesseract>();
    tesseract->init(urdf_file, srdf_file, locator);
    EXPECT_TRUE(tesseract_->init(urdf_file, srdf_file, locator));

    // Extract necessary kinematic information
    forward_kinematics_ = tesseract->getFwdKinematicsManager()->getFwdKinematicSolver("right_arm");
    inverse_kinematics_ = tesseract_->getInvKinematicsManager()->getInvKinematicSolver("right_arm");
    n_dof_ = forward_kinematics_->numJoints();

    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        tesseract->getEnvironment()->getSceneGraph(),
        forward_kinematics_->getActiveLinkNames(),
        tesseract->getEnvironment()->getCurrentState()->link_transforms);
    kinematic_info_ = std::make_shared<trajopt::CartPosKinematicInfo>(
        forward_kinematics_, adjacency_map, Eigen::Isometry3d::Identity(), forward_kinematics_->getTipLinkName());

    auto pos = Eigen::VectorXd::Ones(forward_kinematics_->numJoints());
    auto var0 = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_0");
    nlp_.AddVariableSet(var0);

    // 4) Add constraints
    auto target_pose = Eigen::Isometry3d::Identity();
    constraint_ = std::make_shared<trajopt::CartPosConstraint>(target_pose, kinematic_info_, var0);
    nlp_.AddConstraintSet(constraint_);
  }
};

/** @brief Checks that the GetValue function is correct */
TEST_F(CartesianPositionConstraintUnit, GetValue)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, GetValue");

  // Run FK to get target pose
  Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof_);
  Eigen::Isometry3d target_pose;
  forward_kinematics_->calcFwdKin(target_pose, joint_position);
  constraint_->SetTargetPose(target_pose);

  // Set the joints to the joint position that should satisfy it
  nlp_.SetVariables(joint_position.data());

  // Given a joint position at the target, the error should be 0
  {
    auto error = constraint_->CalcValues(joint_position);
    EXPECT_LT(error.maxCoeff(), 1e-3);
    EXPECT_GT(error.minCoeff(), -1e-3);
  }
  {
    auto error = constraint_->GetValues();
    EXPECT_LT(error.maxCoeff(), 1e-3);
    EXPECT_GT(error.minCoeff(), -1e-3);
  }

  // Given a joint position a small distance away, check the error for translation
  {
    Eigen::Isometry3d target_pose_mod = target_pose;
    target_pose_mod.translate(Eigen::Vector3d(0.1, 0.0, 0.0));
    constraint_->SetTargetPose(target_pose_mod);
    auto error = constraint_->CalcValues(joint_position);
    EXPECT_NEAR(error[0], -0.1, 1e-3);
  }
  {
    Eigen::Isometry3d target_pose_mod = target_pose;
    target_pose_mod.translate(Eigen::Vector3d(0.0, 0.2, 0.0));
    constraint_->SetTargetPose(target_pose_mod);
    auto error = constraint_->CalcValues(joint_position);
    EXPECT_NEAR(error[1], -0.2, 1e-3);
  }
  {
    Eigen::Isometry3d target_pose_mod = target_pose;
    target_pose_mod.translate(Eigen::Vector3d(0.0, 0.0, -0.3));
    constraint_->SetTargetPose(target_pose_mod);
    auto error = constraint_->CalcValues(joint_position);
    EXPECT_NEAR(error[2], 0.3, 1e-3);
  }

  // TODO: Check error for orientation
}

/** @brief Checks that the FillJacobian function is correct */
TEST_F(CartesianPositionConstraintUnit, FillJacobian)
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, FillJacobian");

  // Run FK to get target pose
  Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof_);
  Eigen::Isometry3d target_pose;
  forward_kinematics_->calcFwdKin(target_pose, joint_position);
  constraint_->SetTargetPose(target_pose);

  // Modify one joint at a time
  for (Eigen::Index i = 0; i < n_dof_; i++)
  {
    // Set the joints
    Eigen::VectorXd joint_position_mod = joint_position;
    joint_position_mod[i] = 2.0;
    nlp_.SetVariables(joint_position_mod.data());

    // Calculate jacobian numerically
    auto error_calculator = [&](const Eigen::Ref<Eigen::VectorXd>& x) { return constraint_->CalcValues(x); };
    trajopt::Jacobian num_jac_block = trajopt::calcForwardNumJac(error_calculator, joint_position_mod, 1e-4);

    // Compare to constraint jacobian
    {
      trajopt::Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
      constraint_->CalcJacobianBlock(joint_position_mod, jac_block);
      EXPECT_TRUE(jac_block.isApprox(num_jac_block, 1e-3));
      //      std::cout << "Numeric:\n" << num_jac_block.toDense() << std::endl;
      //      std::cout << "Analytic:\n" << jac_block.toDense() << std::endl;
    }
    {
      trajopt::Jacobian jac_block(num_jac_block.rows(), num_jac_block.cols());
      constraint_->FillJacobianBlock("Joint_Position_0", jac_block);
      EXPECT_TRUE(jac_block.toDense().isApprox(num_jac_block.toDense(), 1e-3));
      //      std::cout << "Numeric:\n" << num_jac_block.toDense() << std::endl;
      //      std::cout << "Analytic:\n" << jac_block.toDense() << std::endl;
    }
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(CartesianPositionConstraintUnit, GetSetBounds)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, GetSetBounds");

  // Check that setting bounds works
  {
    Eigen::VectorXd pos = Eigen::VectorXd::Ones(forward_kinematics_->numJoints());
    auto var0 = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_0");

    auto target_pose = Eigen::Isometry3d::Identity();
    auto constraint_2 = std::make_shared<trajopt::CartPosConstraint>(target_pose, kinematic_info_, var0);

    ifopt::Bounds bounds(-0.1234, 0.5678);
    std::vector<ifopt::Bounds> bounds_vec = std::vector<ifopt::Bounds>(6, bounds);

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
TEST_F(CartesianPositionConstraintUnit, IgnoreVariables)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CartesianPositionConstraintUnit, IgnoreVariables");

  // Check that jacobian does not change for variables it shouldn't
  {
    ifopt::ConstraintSet::Jacobian jac_block_input;
    jac_block_input.resize(n_dof_, n_dof_);
    constraint_->FillJacobianBlock("another_var", jac_block_input);
    EXPECT_EQ(jac_block_input.nonZeros(), 0);
  }
  // Check that it is fine with jac blocks the wrong size for this constraint
  {
    ifopt::ConstraintSet::Jacobian jac_block_input;
    jac_block_input.resize(3, 5);
    constraint_->FillJacobianBlock("another_var2", jac_block_input);
    EXPECT_EQ(jac_block_input.nonZeros(), 0);
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
