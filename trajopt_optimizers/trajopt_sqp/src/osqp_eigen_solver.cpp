#include <trajopt_sqp/osqp_eigen_solver.h>

#include <OsqpEigen/OsqpEigen.h>

namespace trajopt
{
void OSQPEigenSolver::init(ifopt::Problem& nlp)
{
  nlp_ = &nlp;
  solver_.clearSolver();

  num_vars_ = nlp_->GetNumberOfOptimizationVariables();
  num_cnts_ = nlp_->GetNumberOfConstraints();

  // set the initial data of the QP solver_
  solver_.data()->setNumberOfVariables(static_cast<int>(num_vars_));
  // OSQP does not have variable limits, so we set constraints on them
  solver_.data()->setNumberOfConstraints(static_cast<int>(num_cnts_ + num_vars_));

  box_size_ = Eigen::VectorXd::Ones(num_vars_) * 1e-1;
  full_bounds_lower_ = Eigen::VectorXd::Ones(num_cnts_ + num_vars_) * -OSQP_INFTY;
  full_bounds_upper_ = Eigen::VectorXd::Ones(num_cnts_ + num_vars_) * OSQP_INFTY;
}

void OSQPEigenSolver::convexifyAndSolve()
{
  assert(nlp_->GetNumberOfConstraints() == num_cnts_);
  assert(nlp_->GetNumberOfOptimizationVariables() == num_vars_);

  solver_.clearSolver();

  convexify();
  updateConstraintBounds();
  updateVariableBounds();

  ////////////////////////////////////////////////////////
  // Instantiate the solver
  ////////////////////////////////////////////////////////
  solver_.initSolver();

  ////////////////////////////////////////////////////////
  // Solve and save the solution
  ////////////////////////////////////////////////////////
  solver_.solve();
  results_ = solver_.getSolution();
}

void OSQPEigenSolver::solve()
{
  assert(nlp_->GetNumberOfConstraints() == num_cnts_);
  assert(nlp_->GetNumberOfOptimizationVariables() == num_vars_);

  solver_.clearSolver();
  solver_.initSolver();
  solver_.solve();
  results_ = solver_.getSolution();
  //  solver_.setWarmStart
}

void OSQPEigenSolver::convexify()
{
  assert(nlp_->GetNumberOfConstraints() == num_cnts_);
  assert(nlp_->GetNumberOfOptimizationVariables() == num_vars_);

  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  solver_.data()->clearHessianMatrix();
  hessian_.resize(num_vars_, num_vars_);
  solver_.data()->setHessianMatrix(hessian_);

  ////////////////////////////////////////////////////////
  // Set the gradient
  ////////////////////////////////////////////////////////
  gradient_.resize(num_vars_);
  ifopt::ConstraintSet::Jacobian cost_jac = nlp_->GetJacobianOfCosts();
  if (cost_jac.nonZeros() > 0)
    gradient_ << cost_jac.toDense().transpose();
  else
    gradient_ = Eigen::VectorXd::Zero(num_vars_);
  solver_.data()->setGradient(gradient_);

  ////////////////////////////////////////////////////////
  // Linearize Constraints
  ////////////////////////////////////////////////////////
  Eigen::SparseMatrix<double> jac = nlp_->GetJacobianOfConstraints();

  // Create triplet list of nonzero constraints
  using T = Eigen::Triplet<double>;
  std::vector<T> tripletList;
  tripletList.reserve(static_cast<std::size_t>(jac.nonZeros()));

  // Iterate over nonzero elements of jac and add them to triplet list
  for (int k = 0; k < jac.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jac, k); it; ++it)
    {
      tripletList.emplace_back(it.row(), it.col(), it.value());
    }
  }
  // Add a diagonal matrix for the variable limits below the actual constraints
  for (Eigen::Index i = 0; i < num_vars_; i++)
    tripletList.emplace_back(i + jac.rows(), i, 1);

  // Insert the triplet list into the sparse matrix
  Eigen::SparseMatrix<double> linearMatrix(num_cnts_ + num_vars_, num_vars_);
  linearMatrix.reserve(jac.nonZeros() + num_vars_);
  linearMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

  // Set linear constraints
  solver_.data()->clearLinearConstraintsMatrix();
  solver_.data()->setLinearConstraintsMatrix(linearMatrix);
}

double OSQPEigenSolver::evaluateConvexCost(const Eigen::Ref<Eigen::VectorXd>& var_vals)
{
  double result_quad = var_vals.transpose() * hessian_ * var_vals;
  double result_lin = gradient_.transpose() * var_vals;
  return result_quad + result_lin;
}

Eigen::VectorXd OSQPEigenSolver::getConstraintViolations()
{
  Eigen::VectorXd cnt_eval = nlp_->EvaluateConstraints(nlp_->GetOptVariables()->GetValues().data());

  // Values will be negative if they violate the constraint
  Eigen::VectorXd dist_from_lower = -(full_bounds_lower_.topRows(num_cnts_) - cnt_eval);
  Eigen::VectorXd dist_from_upper = full_bounds_upper_.topRows(num_cnts_) - cnt_eval;
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(num_cnts_);

  // Now we put those values into a matrix
  Eigen::MatrixXd tmp(num_cnts_, 3);
  tmp << dist_from_lower, dist_from_upper, zero;

  // We return the worst violation and flip it so violations are positive
  Eigen::VectorXd violation = -1 * tmp.rowwise().minCoeff();
  return violation;
}

void OSQPEigenSolver::updateConstraintBounds()
{
  assert(nlp_->GetNumberOfConstraints() == num_cnts_);
  assert(nlp_->GetNumberOfOptimizationVariables() == num_vars_);
  if (num_cnts_)
  {
    ////////////////////////////////////////////////////////
    // Set the bounds of the constraints
    ////////////////////////////////////////////////////////
    Eigen::VectorXd cnt_bound_lower(num_cnts_);
    Eigen::VectorXd cnt_bound_upper(num_cnts_);

    // Convert constraint bounds to VectorXd
    std::vector<ifopt::Bounds> cnt_bounds = nlp_->GetBoundsOnConstraints();
    for (Eigen::Index i = 0; i < num_cnts_; i++)
    {
      cnt_bound_lower[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
      cnt_bound_upper[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
    }

    // Get values about which we will linearize
    Eigen::VectorXd x_initial = nlp_->GetVariableValues();
    Eigen::VectorXd cnt_initial_value = nlp_->EvaluateConstraints(x_initial.data());

    // Our error is now represented as dy(x0)/dx * x + (y(x0) - dy(xo)/dx * x0)
    // This accounts for moving (error - dy/dx*x) term to other side of equation
    Eigen::SparseMatrix<double> jac = nlp_->GetJacobianOfConstraints();
    // TODO: I think I was messing with joint limits here. I need to only do this change on the cnts (top part)
    Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - (cnt_initial_value - jac * x_initial);
    Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - (cnt_initial_value - jac * x_initial);

    // Insert linearized constraint bounds
    full_bounds_lower_.topRows(num_cnts_) =
        linearized_cnt_lower.cwiseMax(Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY);
    full_bounds_upper_.topRows(num_cnts_) =
        linearized_cnt_upper.cwiseMin(Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY);

    solver_.data()->setLowerBound(full_bounds_lower_);
    solver_.data()->setUpperBound(full_bounds_upper_);
  }
}

void OSQPEigenSolver::updateVariableBounds()
{
  assert(nlp_->GetNumberOfConstraints() == num_cnts_);
  assert(nlp_->GetNumberOfOptimizationVariables() == num_vars_);
  Eigen::VectorXd x_initial = nlp_->GetVariableValues();

  // Calculate box constraints
  Eigen::VectorXd lower_box_cnt = x_initial - box_size_;
  Eigen::VectorXd upper_box_cnt = x_initial + box_size_;

  // Set the variable limits once
  std::vector<ifopt::Bounds> var_bounds = nlp_->GetBoundsOnOptimizationVariables();
  Eigen::VectorXd var_bounds_lower(num_vars_);
  Eigen::VectorXd var_bounds_upper(num_vars_);
  for (Eigen::Index i = 0; i < num_vars_; i++)
  {
    var_bounds_lower[i] = var_bounds[static_cast<std::size_t>(i)].lower_;
    var_bounds_upper[i] = var_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Apply box constraints and variable limits
  Eigen::VectorXd var_bounds_lower_final =
      var_bounds_lower.cwiseMax(Eigen::VectorXd::Ones(num_vars_) * -OSQP_INFTY).cwiseMax(lower_box_cnt);
  Eigen::VectorXd var_bounds_upper_final =
      var_bounds_upper.cwiseMin(Eigen::VectorXd::Ones(num_vars_) * OSQP_INFTY).cwiseMin(upper_box_cnt);
  full_bounds_lower_.bottomRows(num_vars_) = var_bounds_lower_final;
  full_bounds_upper_.bottomRows(num_vars_) = var_bounds_upper_final;

  // Send the full bounds vector to OSQP
  solver_.data()->setLowerBound(full_bounds_lower_);
  solver_.data()->setUpperBound(full_bounds_upper_);
}

void OSQPEigenSolver::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size)
{
  assert(box_size.size() == num_vars_);
  box_size_ = box_size;
}

}  // namespace trajopt
