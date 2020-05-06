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

#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>
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

class InverseKinematicsConstraintUnit : public testing::TestWithParam<const char*>
{
public:
  tesseract::Tesseract::Ptr tesseract_ = std::make_shared<Tesseract>();
  ifopt::Problem nlp_;

  tesseract_kinematics::ForwardKinematics::Ptr forward_kinematics_;
  tesseract_kinematics::InverseKinematics::Ptr inverse_kinematics_;
  InverseKinematicsInfo::Ptr kinematic_info_;
  InverseKinematicsConstraint::Ptr constraint_;

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
    kinematic_info_ = std::make_shared<trajopt::InverseKinematicsInfo>(
        inverse_kinematics_, adjacency_map, Eigen::Isometry3d::Identity(), forward_kinematics_->getTipLinkName());

    auto pos = Eigen::VectorXd::Ones(forward_kinematics_->numJoints());
    auto var0 = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_0");
    auto var1 = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_1");
    nlp_.AddVariableSet(var0);
    nlp_.AddVariableSet(var1);

    // 4) Add constraints
    auto target_pose = Eigen::Isometry3d::Identity();
    constraint_ = std::make_shared<trajopt::InverseKinematicsConstraint>(target_pose, kinematic_info_, var0, var1);
    nlp_.AddConstraintSet(constraint_);
  }
};

TEST_F(InverseKinematicsConstraintUnit, GetValue)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("InverseKinematicsConstraintUnit, GetValue");

  // Run FK to get target pose
  Eigen::VectorXd joint_position_single = Eigen::VectorXd::Ones(forward_kinematics_->numJoints());
  auto target_pose = Eigen::Isometry3d::Identity();
  forward_kinematics_->calcFwdKin(target_pose, joint_position_single);
  constraint_->SetTargetPose(target_pose);

  // Set the joints to that joint position
  Eigen::VectorXd joint_position = Eigen::VectorXd::Ones(n_dof_ * 2);
  nlp_.SetVariables(joint_position.data());

  // Get the value (distance from IK position)
  Eigen::VectorXd values = constraint_->GetValues();
  EXPECT_TRUE(values.isApprox(Eigen::VectorXd::Zero(n_dof_)));

  // Check that jac wrt constraint_var is identity
  {
    ifopt::ConstraintSet::Jacobian jac_block;
    jac_block.resize(n_dof_, n_dof_);
    constraint_->FillJacobianBlock("Joint_Position_0", jac_block);
    // Check that the size is correct
    EXPECT_EQ(jac_block.nonZeros(), n_dof_);
    // Check that it is identity
    for (Eigen::Index i = 0; i < n_dof_; i++)
      EXPECT_EQ(jac_block.coeff(i, i), -1.0);
  }

  // Check that jac wrt seed_var is zero
  {
    ifopt::ConstraintSet::Jacobian jac_block;
    jac_block.resize(n_dof_, n_dof_);
    constraint_->FillJacobianBlock("Joint_Position_1", jac_block);
    EXPECT_EQ(jac_block.nonZeros(), 0);
  }
}

/**
 * @brief Checks that the Bounds are set correctly
 */
TEST_F(InverseKinematicsConstraintUnit, GetSetBounds)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("InverseKinematicsConstraintUnit, GetSetBounds");

  // Check that setting bounds works
  {
    Eigen::VectorXd pos = Eigen::VectorXd::Ones(n_dof_);
    auto var0 = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_0");
    auto var1 = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_1");

    auto target_pose = Eigen::Isometry3d::Identity();
    auto constraint_2 =
        std::make_shared<trajopt::InverseKinematicsConstraint>(target_pose, kinematic_info_, var0, var1);

    ifopt::Bounds bounds(-0.1234, 0.5678);
    std::vector<ifopt::Bounds> bounds_vec = std::vector<ifopt::Bounds>(static_cast<std::size_t>(n_dof_), bounds);

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
TEST_F(InverseKinematicsConstraintUnit, IgnoreVariables)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("InverseKinematicsConstraintUnit, IgnoreVariables");

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
