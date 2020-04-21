#include <trajopt_ifopt/constraints/collision_constraint.h>

#include <tesseract_kinematics/core/utils.h>
#include <trajopt/collision_terms.hpp>

#include <console_bridge/console.h>

namespace trajopt
{
CollisionConstraintIfopt::CollisionConstraintIfopt(SingleTimestepCollisionEvaluator::Ptr collision_evaluator,
                                                   JointPosition::Ptr position_var,
                                                   const std::string& name)
  : ifopt::ConstraintSet(1, name)
  , position_var_(std::move(position_var))
  , collision_evaluator_(std::move(collision_evaluator))
{
  // Set n_dof_ for convenience
  n_dof_ = position_var_->GetRows();
  assert(n_dof_ > 0);

  bounds_ = std::vector<ifopt::Bounds>(1, ifopt::BoundSmallerZero);
}

Eigen::VectorXd CollisionConstraintIfopt::GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  DblVec joint_vals_vec(joint_vals.data(), joint_vals.data() + joint_vals.rows() * joint_vals.cols());

  Eigen::VectorXd err(1);

  // Check the collisions
  tesseract_collision::ContactResultVector dist_results;
  {
    tesseract_collision::ContactResultMap dist_results_map;
    collision_evaluator_->CalcCollisions(joint_vals, dist_results_map);
    tesseract_collision::flattenMoveResults(std::move(dist_results_map), dist_results);
  }

  for (tesseract_collision::ContactResult& dist_result : dist_results)
  {
    // Contains the contact distance threshold and coefficient for the given link pair
    const Eigen::Vector2d& data = collision_evaluator_->getSafetyMarginData()->getPairSafetyMarginData(
        dist_result.link_names[0], dist_result.link_names[1]);
    // distance will be distance from threshold with negative being greater (further) than the threshold times the
    // coeff
    err[0] += sco::pospart((data[0] - dist_result.distance) * data[1]);
  }
  return err;
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> CollisionConstraintIfopt::GetBounds() const { return bounds_; }

void CollisionConstraintIfopt::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void CollisionConstraintIfopt::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var_->GetName())
  {
    // Reserve enough room in the sparse matrix
    jac_block.reserve(Eigen::VectorXd::Constant(1, n_dof_));

    // Get current joint values
    VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
    DblVec joint_vals_vec(joint_vals.data(), joint_vals.data() + joint_vals.rows() * joint_vals.cols());

    // Calculate collisions
    tesseract_collision::ContactResultVector dist_results;
    {
      tesseract_collision::ContactResultMap dist_results_map;
      collision_evaluator_->CalcCollisions(joint_vals, dist_results_map);
      tesseract_collision::flattenMoveResults(std::move(dist_results_map), dist_results);
    }

    // Get gradients for all contacts
    std::vector<trajopt::GradientResults> grad_results;
    grad_results.reserve(dist_results.size());
    for (tesseract_collision::ContactResult& dist_result : dist_results)
    {
      // Contains the contact distance threshold and coefficient for the given link pair
      const Eigen::Vector2d& data = collision_evaluator_->getSafetyMarginData()->getPairSafetyMarginData(
          dist_result.link_names[0], dist_result.link_names[1]);
      grad_results.push_back(collision_evaluator_->GetGradient(joint_vals, dist_result, data, true));
    }

    // Convert GradientResults to jacobian
    int idx = 0;
    Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(n_dof_);
    for (auto& grad : grad_results)
    {
      if (grad.gradients[0].has_gradient)
        grad_vec += grad.gradients[0].gradient;
      if (grad.gradients[1].has_gradient)
        grad_vec += grad.gradients[1].gradient;
      idx++;
    }

    // This does work but could be faster
    for (int j = 0; j < n_dof_; j++)
    {
      // Collision is 1 x n_dof
      jac_block.coeffRef(0, j) = -1 * grad_vec[j];
    }
  }
}
}  // namespace trajopt
