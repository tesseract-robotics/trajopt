#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
InverseKinematicsConstraint::InverseKinematicsConstraint(const Eigen::Isometry3d& target_pose,
                                                         InverseKinematicsInfo::ConstPtr kinematic_info,
                                                         JointPosition::Ptr constraint_var,
                                                         JointPosition::Ptr seed_var,
                                                         const std::string& name)
  : ifopt::ConstraintSet(constraint_var->GetRows(), name)
  , constraint_var_(std::move(constraint_var))
  , seed_var_(std::move(seed_var))
  , target_pose_(target_pose)
  , kinematic_info_(std::move(kinematic_info))
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = constraint_var_->GetRows();
  assert(n_dof_ > 0);
  if (static_cast<std::size_t>(constraint_var_->GetRows()) != kinematic_info_->inverse_kinematics_->numJoints())
    CONSOLE_BRIDGE_logError("Inverse kinematics has a different number of joints than the given variable set");

  bounds_ = std::vector<ifopt::Bounds>(static_cast<std::size_t>(n_dof_), ifopt::BoundZero);
}

Eigen::VectorXd InverseKinematicsConstraint::GetValues() const
{
  // Get joint position using IK and the seed variable
  Eigen::VectorXd target_joint_position;
  Eigen::VectorXd seed_joint_position = this->GetVariables()->GetComponent(seed_var_->GetName())->GetValues();
  kinematic_info_->inverse_kinematics_->calcInvKin(target_joint_position, target_pose_, seed_joint_position);

  // Calculate joint error
  Eigen::VectorXd error =
      target_joint_position - this->GetVariables()->GetComponent(constraint_var_->GetName())->GetValues();
  return error;
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> InverseKinematicsConstraint::GetBounds() const { return bounds_; }

void InverseKinematicsConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  if (bounds.size() != static_cast<std::size_t>(n_dof_))
    CONSOLE_BRIDGE_logError("Bounds is incorrect size. It is %d when it should be %d", bounds.size(), n_dof_);

  bounds_ = bounds;
}

void InverseKinematicsConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == constraint_var_->GetName())
  {
    // Reserve enough room in the sparse matrix
    jac_block.reserve(n_dof_);

    for (int j = 0; j < n_dof_; j++)
    {
      // err = target - x =? derr/dx = -1
      jac_block.coeffRef(j, j) = -1;
    }
  }
}

void InverseKinematicsConstraint::setTargetPose(const Eigen::Isometry3d& target_pose) { target_pose_ = target_pose; }
}  // namespace trajopt
