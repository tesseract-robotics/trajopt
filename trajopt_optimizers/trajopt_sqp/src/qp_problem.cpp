/**
 * @file qp_problem.cpp
 * @brief Converts general NLP to QP for SQP routine
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
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
#include <trajopt_sqp/qp_problem.h>
#include <iostream>

namespace trajopt_sqp
{
void QPProblem::init(ifopt::Problem& nlp)
{
  nlp_ = &nlp;

  num_nlp_vars_ = nlp_->GetNumberOfOptimizationVariables();
  num_nlp_cnts_ = nlp_->GetNumberOfConstraints();
  num_nlp_costs_ = nlp_->GetCosts().GetRows();
  cost_constant_ = Eigen::VectorXd::Zero(1);

  num_qp_vars_ = num_nlp_vars_;
  num_qp_cnts_ = num_nlp_cnts_ + num_nlp_vars_;
  box_size_ = Eigen::VectorXd::Constant(num_nlp_vars_, 1e-1);
  constraint_merit_coeff_ = Eigen::VectorXd::Constant(num_nlp_cnts_, 10);

  ////////////////////////////////////////////////////////
  // Get NLP bounds and detect constraint type
  ////////////////////////////////////////////////////////
  Eigen::VectorXd nlp_bounds_l(num_nlp_cnts_);
  Eigen::VectorXd nlp_bounds_u(num_nlp_cnts_);
  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = nlp_->GetBoundsOnConstraints();
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    nlp_bounds_l[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    nlp_bounds_u[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Detect constraint type
  Eigen::VectorXd nlp_bounds_diff = nlp_bounds_u - nlp_bounds_l;
  constraint_types_.resize(static_cast<std::size_t>(num_nlp_cnts_));
  for (std::size_t i = 0; i < static_cast<std::size_t>(nlp_bounds_diff.size()); i++)
  {
    if (std::abs(nlp_bounds_diff[static_cast<Eigen::Index>(i)]) < 1e-3)
    {
      constraint_types_[i] = ConstraintType::EQ;
      // Add 2 slack variables for L1 loss
      num_qp_vars_ += 2;
      num_qp_cnts_ += 2;
    }
    else
    {
      constraint_types_[i] = ConstraintType::INEQ;
      // Add 1 slack variable for hinge loss
      num_qp_vars_ += 1;
      num_qp_cnts_ += 1;
    }
  }

  // Initialize the constraint bounds
  bounds_lower_ = Eigen::VectorXd::Constant(num_qp_cnts_, -double(INFINITY));
  bounds_upper_ = Eigen::VectorXd::Constant(num_qp_cnts_, double(INFINITY));
}

void QPProblem::convexify()
{
  updateHessian();

  updateGradient();

  updateCostsConstantExpression();

  updateConstraintsConstantExpression();

  linearizeConstraints();

  updateNLPConstraintBounds();

  updateNLPVariableBounds();

  updateSlackVariableBounds();
}

void QPProblem::updateHessian()
{
  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  hessian_.resize(num_qp_vars_, num_qp_vars_);
}

void QPProblem::updateGradient()
{
  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  gradient_ = Eigen::VectorXd::Zero(num_qp_vars_);
  ifopt::ConstraintSet::Jacobian cost_jac = nlp_->GetJacobianOfCosts();
  if (cost_jac.nonZeros() > 0)
    gradient_.topRows(num_nlp_vars_) = cost_jac.toDense().transpose();

  ////////////////////////////////////////////////////////
  // Set the gradient of the constraint slack variables
  ////////////////////////////////////////////////////////
  {
    Eigen::Index current_var_index = num_nlp_vars_;
    for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
    {
      if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
      {
        gradient_[current_var_index] = constraint_merit_coeff_[i];
        gradient_[current_var_index + 1] = constraint_merit_coeff_[i];

        current_var_index += 2;
      }
      else
      {
        gradient_[current_var_index] = constraint_merit_coeff_[i];
        current_var_index++;
      }
    }
  }
}

void QPProblem::linearizeConstraints()
{
  Eigen::SparseMatrix<double> jac = nlp_->GetJacobianOfConstraints();

  // Create triplet list of nonzero constraints
  using T = Eigen::Triplet<double>;
  std::vector<T> tripletList;
  tripletList.reserve(static_cast<std::size_t>(jac.nonZeros() + num_qp_vars_) * 3);

  // Add jacobian to triplet list
  for (int k = 0; k < jac.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jac, k); it; ++it)
    {
      tripletList.emplace_back(it.row(), it.col(), it.value());
    }
  }

  // Add the slack variables to each constraint
  Eigen::Index current_column_index = num_nlp_vars_;
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      tripletList.emplace_back(i, current_column_index, 1);
      tripletList.emplace_back(i, current_column_index + 1, -1);
      current_column_index += 2;
    }
    else
    {
      tripletList.emplace_back(i, current_column_index, -1);
      current_column_index++;
    }
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  for (Eigen::Index i = 0; i < num_qp_vars_; i++)
    tripletList.emplace_back(i + jac.rows(), i, 1);

  // Insert the triplet list into the sparse matrix
  constraint_matrix_.resize(num_qp_cnts_, num_qp_vars_);
  constraint_matrix_.reserve(jac.nonZeros() + num_qp_vars_);
  constraint_matrix_.setFromTriplets(tripletList.begin(), tripletList.end());
}

void QPProblem::updateCostsConstantExpression()
{
  if (num_nlp_costs_ == 0)
    return;

  // Get values about which we will linearize
  Eigen::VectorXd x_initial = nlp_->GetVariableValues().head(num_nlp_vars_);
  Eigen::VectorXd cost_initial_value = nlp_->GetCosts().GetValues();

  // The block excludes the slack variables
  Eigen::VectorXd result_quad = x_initial.transpose() * hessian_.block(0, 0, num_nlp_vars_, num_nlp_vars_) * x_initial;
  Eigen::VectorXd result_lin = x_initial.transpose() * gradient_.block(0, 0, num_nlp_vars_, num_nlp_costs_);
  cost_constant_ = cost_initial_value - result_quad - result_lin;
}

void QPProblem::updateConstraintsConstantExpression()
{
  if (num_nlp_cnts_ == 0)
    return;

  // Get values about which we will linearize
  Eigen::VectorXd x_initial = nlp_->GetVariableValues().head(num_nlp_vars_);
  /** @todo Why not use nlp_->GetConstraints().GetValues()? */
  Eigen::VectorXd cnt_initial_value = nlp_->EvaluateConstraints(x_initial.data());

  // d = collision distance
  // T = distance threshhold
  // J = Jacobian
  // x = initial values
  // Original trajopt Affine constant = T - d - (J * x)
  // Trajopt ifopt cnt_initial_value = T - d, so the following is correct.

  // The block excludes the slack variables
  Eigen::SparseMatrix<double> jac = nlp_->GetJacobianOfConstraints().block(0, 0, num_nlp_cnts_, num_nlp_vars_);
  constraint_constant_ = (cnt_initial_value - jac * x_initial);
}

void QPProblem::updateNLPConstraintBounds()
{
  if (num_nlp_cnts_ == 0)
    return;

  Eigen::VectorXd cnt_bound_lower(num_nlp_cnts_);
  Eigen::VectorXd cnt_bound_upper(num_nlp_cnts_);

  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = nlp_->GetBoundsOnConstraints();
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    cnt_bound_lower[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    cnt_bound_upper[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - constraint_constant_;
  Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - constraint_constant_;

  // Insert linearized constraint bounds
  bounds_lower_.topRows(num_nlp_cnts_) = linearized_cnt_lower;
  bounds_upper_.topRows(num_nlp_cnts_) = linearized_cnt_upper;
}

void QPProblem::updateNLPVariableBounds()
{
  // This is eqivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  Eigen::VectorXd x_initial = nlp_->GetVariableValues();

  // Calculate box constraints
  Eigen::VectorXd lower_box_cnt = x_initial - box_size_;
  Eigen::VectorXd upper_box_cnt = x_initial + box_size_;

  // Set the variable limits once
  std::vector<ifopt::Bounds> var_bounds = nlp_->GetBoundsOnOptimizationVariables();
  Eigen::VectorXd var_bounds_lower(num_nlp_vars_);
  Eigen::VectorXd var_bounds_upper(num_nlp_vars_);
  for (Eigen::Index i = 0; i < num_nlp_vars_; i++)
  {
    var_bounds_lower[i] = var_bounds[static_cast<std::size_t>(i)].lower_;
    var_bounds_upper[i] = var_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Apply box constraints and variable limits
  Eigen::VectorXd var_bounds_lower_final = var_bounds_lower.cwiseMax(lower_box_cnt);
  // Add the extra check here that the upper is bigger than the lower. It seems that there can be issues when the
  // numbers get close to 0.
  Eigen::VectorXd var_bounds_upper_final = var_bounds_upper.cwiseMin(upper_box_cnt).cwiseMax(var_bounds_lower);
  bounds_lower_.block(num_nlp_cnts_, 0, var_bounds_lower_final.size(), 1) = var_bounds_lower_final;
  bounds_upper_.block(num_nlp_cnts_, 0, var_bounds_upper_final.size(), 1) = var_bounds_upper_final;
}

void QPProblem::updateSlackVariableBounds()
{
  Eigen::Index current_cnt_index = num_nlp_cnts_ + num_nlp_vars_;
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index] = double(INFINITY);
      bounds_lower_[current_cnt_index + 1] = 0;
      bounds_upper_[current_cnt_index + 1] = double(INFINITY);

      current_cnt_index += 2;
    }
    else
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index] = double(INFINITY);

      current_cnt_index++;
    }
  }
}

double QPProblem::evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return evaluateConvexCosts(var_vals).sum();
}

Eigen::VectorXd QPProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (num_nlp_costs_ == 0)
    return Eigen::VectorXd();

  auto var_block = var_vals.head(num_nlp_vars_);
  Eigen::VectorXd result_quad = var_block.transpose() * hessian_.block(0, 0, num_nlp_vars_, num_nlp_vars_) * var_block;
  Eigen::VectorXd result_lin = var_block.transpose() * gradient_.block(0, 0, num_nlp_vars_, num_nlp_costs_);
  return cost_constant_ + result_lin + result_quad;
}

Eigen::VectorXd QPProblem::calcConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& cnt_vals)
{
  std::vector<ifopt::Bounds> cnt_bounds = nlp_->GetBoundsOnConstraints();
  Eigen::VectorXd violation(num_nlp_cnts_);
  for (Eigen::Index i = 0; i < num_nlp_cnts_; ++i)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      // The lower and upper should be the same because it is an Equality constraint
      violation[i] = std::abs(cnt_vals[i] - cnt_bounds[static_cast<std::size_t>(i)].lower_);
    }
    else  // ConstraintType::INEQ
    {
      // For inequality constraint the valid range is between [0, -inf] so anything greater than
      // zero is a violation
      violation[i] = std::max<double>(cnt_vals[i] - cnt_bounds[static_cast<std::size_t>(i)].upper_, 0);
    }
  }

  return violation;
}

Eigen::VectorXd QPProblem::evaluateConvexConstraintViolation(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  Eigen::VectorXd result_lin =
      constraint_matrix_.block(0, 0, num_nlp_cnts_, num_nlp_vars_) * var_vals.head(num_nlp_vars_);
  Eigen::VectorXd constraint_value = constraint_constant_ + result_lin;
  return calcConstraintViolations(constraint_value);
}

Eigen::VectorXd QPProblem::evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  Eigen::VectorXd cnt_vals = nlp_->EvaluateConstraints(var_vals.data());
  return calcConstraintViolations(cnt_vals);
}

Eigen::VectorXd QPProblem::getExactConstraintViolations()
{
  return evaluateExactConstraintViolations(nlp_->GetOptVariables()->GetValues());

  //  // Convert constraint bounds to VectorXd
  //  Eigen::VectorXd cnt_bound_lower(num_nlp_cnts_);
  //  Eigen::VectorXd cnt_bound_upper(num_nlp_cnts_);
  //  std::vector<ifopt::Bounds> cnt_bounds = nlp_->GetBoundsOnConstraints();
  //  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  //  {
  //    cnt_bound_lower[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
  //    cnt_bound_upper[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  //  }

  //  // Values will be negative if they violate the constraint
  //  Eigen::VectorXd zero = Eigen::VectorXd::Zero(num_nlp_cnts_);
  //  Eigen::VectorXd dist_from_lower = -(cnt_bound_lower - cnt_eval);
  //  Eigen::VectorXd dist_from_upper = cnt_bound_upper - cnt_eval;

  //  // Now we put those values into a matrix
  //  Eigen::MatrixXd tmp(num_nlp_cnts_, 3);
  //  tmp << dist_from_lower, dist_from_upper, zero;

  //  // We return the worst violation and flip it so violations are positive
  //  Eigen::VectorXd violation = -1 * tmp.rowwise().minCoeff();
  //  return violation;
}

void QPProblem::scaleBoxSize(double& scale) { box_size_ = box_size_ * scale; }

void QPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size)
{
  assert(box_size.size() == num_nlp_vars_);
  box_size_ = box_size;
}

Eigen::VectorXd QPProblem::getBoxSize() const { return box_size_; }

void QPProblem::print() const
{
  Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << std::endl;
  std::cout << "Num NLP Vars: " << num_nlp_vars_ << std::endl;
  std::cout << "Num QP Vars: " << num_qp_vars_ << std::endl;
  std::cout << "Num NLP Constraints: " << num_qp_cnts_ << std::endl;
  std::cout << "Detected Constraint Type: ";
  for (const auto& cnt : constraint_types_)
    std::cout << static_cast<int>(cnt) << ", ";

  std::cout << std::endl;
  std::cout << "box_size_: " << box_size_.transpose().format(format) << std::endl;
  std::cout << "constraint_merit_coeff_: " << constraint_merit_coeff_.transpose().format(format) << std::endl;

  std::cout << "Hessian:\n" << hessian_.toDense().format(format) << std::endl;
  std::cout << "Gradient: " << gradient_.transpose().format(format) << std::endl;
  std::cout << "Constraint Matrix:\n" << constraint_matrix_.toDense().format(format) << std::endl;
  std::cout << "bounds_lower: " << bounds_lower_.transpose().format(format) << std::endl;
  std::cout << "bounds_upper: " << bounds_upper_.transpose().format(format) << std::endl;
  std::cout << "NLP values: " << nlp_->GetVariableValues().transpose().format(format) << std::endl;
}
}  // namespace trajopt_sqp
