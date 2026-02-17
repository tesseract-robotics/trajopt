/**
 * @file qp_problem.cpp
 * @brief Converts general NLP to QP for SQP routine
 *
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
#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/costs/absolute_cost.h>
#include <trajopt_ifopt/core/problem.h>
#include <iostream>

namespace trajopt_sqp
{
IfoptQPProblem::IfoptQPProblem(std::shared_ptr<trajopt_ifopt::Variables> variables)
  : nlp_(std::make_shared<trajopt_ifopt::Problem>(std::move(variables)))
{
}
IfoptQPProblem::IfoptQPProblem(std::shared_ptr<trajopt_ifopt::Problem> nlp) : nlp_(std::move(nlp)) {}

void IfoptQPProblem::addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set)
{
  if (constraint_set->isDynamic())
    throw std::runtime_error("IfoptQPProblem, dynamic constraint sets are not supported");

  nlp_->addConstraintSet(std::move(constraint_set));
}

void IfoptQPProblem::addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                                CostPenaltyType penalty_type)
{
  if (constraint_set->isDynamic())
    throw std::runtime_error("IfoptQPProblem, dynamic cost sets are not supported");

  switch (penalty_type)
  {
    case CostPenaltyType::kSquared:
    {
      // Must link the variables to the constraint since that happens in AddConstraintSet
      constraint_set->linkWithVariables(nlp_->getOptVariables());
      auto cost = std::make_shared<trajopt_ifopt::SquaredCost>(constraint_set);
      nlp_->addCostSet(std::move(cost));
      break;
    }
    case CostPenaltyType::kAbsolute:
    {
      // Must link the variables to the constraint since that happens in AddConstraintSet
      constraint_set->linkWithVariables(nlp_->getOptVariables());
      auto cost = std::make_shared<trajopt_ifopt::AbsoluteCost>(constraint_set);
      nlp_->addCostSet(std::move(cost));
      break;
    }
    default:
      throw std::runtime_error("IfoptQPProblem: Unsupported cost penalty type!");
  }
}

void IfoptQPProblem::setup()
{
  auto start_variables = nlp_->getVariableValues();
  nlp_->setVariables(start_variables.data());

  num_nlp_vars_ = nlp_->getNumberOfOptimizationVariables();
  num_nlp_cnts_ = nlp_->getNumberOfConstraints();
  num_nlp_costs_ = nlp_->getCosts().getRows();
  cost_constant_ = Eigen::VectorXd::Zero(1);

  num_qp_vars_ = num_nlp_vars_;
  num_qp_cnts_ = num_nlp_cnts_ + num_nlp_vars_;
  box_size_ = Eigen::VectorXd::Constant(num_nlp_vars_, 1e-1);
  constraint_merit_coeff_ = Eigen::VectorXd::Constant(num_nlp_cnts_, 10);
  constraint_types_.resize(static_cast<std::size_t>(num_nlp_cnts_));

  // Get NLP Cost and Constraint Names for Debug Print
  for (const auto& cnt : nlp_->getConstraints().getComponents())
  {
    for (Eigen::Index j = 0; j < cnt->getRows(); j++)
      constraint_names_.push_back(cnt->getName() + "_" + std::to_string(j));
  }

  for (const auto& cost : nlp_->getCosts().getComponents())
  {
    for (Eigen::Index j = 0; j < cost->getRows(); j++)
      cost_names_.push_back(cost->getName() + "_" + std::to_string(j));
  }

  // Get bounds
  Eigen::VectorXd nlp_bounds_l(num_nlp_cnts_);
  Eigen::VectorXd nlp_bounds_u(num_nlp_cnts_);
  // Convert constraint bounds to VectorXd
  std::vector<trajopt_ifopt::Bounds> cnt_bounds = nlp_->getBoundsOnConstraints();
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    const auto& b = cnt_bounds[static_cast<std::size_t>(i)];
    nlp_bounds_l[i] = b.getLower();
    nlp_bounds_u[i] = b.getUpper();

    constraint_types_[static_cast<std::size_t>(i)] = b.getType();
    if (b.getType() == trajopt_ifopt::BoundsType::kEquality)
    {
      // Add 2 slack variables for L1 loss
      num_qp_vars_ += 2;
      num_qp_cnts_ += 2;
    }
    else if (b.getType() == trajopt_ifopt::BoundsType::kLowerBound ||
             b.getType() == trajopt_ifopt::BoundsType::kUpperBound)
    {
      // Add 1 slack variable for hinge loss
      num_qp_vars_ += 1;
      num_qp_cnts_ += 1;
    }
    else
      throw std::runtime_error("Unsupport bounds type!");
  }

  // Initialize the constraint bounds
  bounds_lower_ = Eigen::VectorXd::Constant(num_qp_cnts_, -double(INFINITY));
  bounds_upper_ = Eigen::VectorXd::Constant(num_qp_cnts_, double(INFINITY));
}

void IfoptQPProblem::setVariables(const double* x) { nlp_->setVariables(x); }
Eigen::VectorXd IfoptQPProblem::getVariableValues() const { return nlp_->getVariableValues(); }

void IfoptQPProblem::convexify()
{
  // This must be called prior to updateGradient
  updateHessian();

  updateGradient();

  linearizeConstraints();  // NOLINT

  // The three above must be called before rest to update internal data

  updateCostsConstantExpression();  // NOLINT

  updateConstraintsConstantExpression();

  updateNLPConstraintBounds();

  updateNLPVariableBounds();

  updateSlackVariableBounds();
}

void IfoptQPProblem::updateHessian()
{
  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  hessian_.resize(num_qp_vars_, num_qp_vars_);
  /**
   * @note See CostFromFunc::convex in modeling_utils.cpp.
   * This should be multiplied by 0.5 when implemented
   * hessian_ = 0.5 * nlp_->GetHessianOfCosts();
   */
}

void IfoptQPProblem::updateGradient()
{
  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  gradient_ = Eigen::VectorXd::Zero(num_qp_vars_);
  const trajopt_ifopt::Jacobian cost_jac = nlp_->getJacobianOfCosts();
  /**
   * @note See CostFromFunc::convex in modeling_utils.cpp. Once Hessian has been implemented
   *
   * if (!full_hessian_) // Does not contain bilinear terms, eg. x1 * x2, only x1^2 and x2^2
   * {
   *   double val;
   *   Eigen::VectorXd grad, hess;
   *   calcGradAndDiagHess(*f_, x_eigen, epsilon_, val, grad, hess);
   *   quad = a + b*x + c*x^2
   *   a = val - grad.dot(x_eigen) + .5 * x_eigen.dot(hess.cwiseProduct(x_eigen));
   *   b = grad - hess.cwiseProduct(x_eigen)
   *   c = 0.5 * hess
   * }
   * else // contains bilinear terms
   * {
   *   double val;
   *   Eigen::VectorXd grad;
   *   Eigen::MatrixXd hess;
   *   calcGradHess(f_, x_eigen, epsilon_, val, grad, hess);
   *
   *   Eigen::MatrixXd pos_hess = Eigen::MatrixXd::Zero(x_eigen.size(), x_eigen.size());
   *   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(hess);
   *   Eigen::VectorXd eigvals = es.eigenvalues();
   *   Eigen::MatrixXd eigvecs = es.eigenvectors();
   *   for (long int i = 0, end = x_eigen.size(); i != end; ++i)
   *   {  // tricky --- eigen size() is signed
   *     if (eigvals(i) > 0)
   *       pos_hess += eigvals(i) * eigvecs.col(i) * eigvecs.col(i).transpose();
   *   }
   *   quad = a + b*x + c*x^2 +
   *   a = val - grad.dot(x_eigen) + .5 * x_eigen.dot(pos_hess * x_eigen);
   *   b = grad - pos_hess * x_eigen
   *   for (long int i = 0, end = x_eigen.size(); i != end; ++i)
   *   {  // tricky --- eigen size() is signed
   *     c(i,i) = (pos_hess(i, i) / 2);
   *     for (long int j = i + 1; j != end; ++j)
   *     {  // tricky --- eigen size() is signed
   *       c(i, j) = pos_hess(i, j);
   *     }
   *   }
   * }
   */

  if (cost_jac.nonZeros() > 0)
    gradient_.topRows(num_nlp_vars_) = cost_jac.toDense().transpose();

  ////////////////////////////////////////////////////////
  // Set the gradient of the constraint slack variables
  ////////////////////////////////////////////////////////
  {
    Eigen::Index current_var_index = num_nlp_vars_;
    for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
    {
      const auto bounds_type = constraint_types_[static_cast<std::size_t>(i)];
      if (bounds_type == trajopt_ifopt::BoundsType::kEquality)
      {
        gradient_[current_var_index] = constraint_merit_coeff_[i];
        gradient_[current_var_index + 1] = constraint_merit_coeff_[i];

        current_var_index += 2;
      }
      else if (bounds_type == trajopt_ifopt::BoundsType::kLowerBound ||
               bounds_type == trajopt_ifopt::BoundsType::kUpperBound)
      {
        gradient_[current_var_index] = constraint_merit_coeff_[i];
        current_var_index++;
      }
      else
        throw std::runtime_error("Unsupported bounds type!");
    }
  }
}

void IfoptQPProblem::linearizeConstraints()
{
  const trajopt_ifopt::Jacobian jac = nlp_->getJacobianOfConstraints();

  // Create triplet list of nonzero constraints
  using T = Eigen::Triplet<double>;
  std::vector<T> tripletList;
  tripletList.reserve(static_cast<std::size_t>(jac.nonZeros() + num_qp_vars_) * 3);

  // Add jacobian to triplet list
  for (int k = 0; k < jac.outerSize(); ++k)  // NOLINT
  {
    for (trajopt_ifopt::Jacobian::InnerIterator it(jac, k); it; ++it)
    {
      // Originally it pruned these but it changes sparsity so we now set to zero
      if (std::abs(it.value()) < 1e-7)
        tripletList.emplace_back(it.row(), it.col(), 0.0);
      else
        tripletList.emplace_back(it.row(), it.col(), it.value());
    }
  }

  // Add the slack variables to each constraint
  Eigen::Index current_column_index = num_nlp_vars_;
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    const auto bounds_type = constraint_types_[static_cast<std::size_t>(i)];
    if (bounds_type == trajopt_ifopt::BoundsType::kEquality)
    {
      tripletList.emplace_back(i, current_column_index, 1);
      tripletList.emplace_back(i, current_column_index + 1, -1);
      current_column_index += 2;
    }
    else if (bounds_type == trajopt_ifopt::BoundsType::kLowerBound)
    {
      tripletList.emplace_back(i, current_column_index, 1);
      current_column_index++;
    }
    else if (bounds_type == trajopt_ifopt::BoundsType::kUpperBound)
    {
      tripletList.emplace_back(i, current_column_index, -1);
      current_column_index++;
    }
    else
      throw std::runtime_error("Unsupported bounds type!");
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  for (Eigen::Index i = 0; i < num_qp_vars_; i++)
    tripletList.emplace_back(i + jac.rows(), i, 1);

  // Insert the triplet list into the sparse matrix
  constraint_matrix_.resize(num_qp_cnts_, num_qp_vars_);
  constraint_matrix_.reserve(jac.nonZeros() + num_qp_vars_);
  constraint_matrix_.setFromTriplets(tripletList.begin(), tripletList.end());  // NOLINT
}

void IfoptQPProblem::updateCostsConstantExpression()
{
  if (num_nlp_costs_ == 0)
    return;

  // Get values about which we will linearize
  Eigen::VectorXd x_initial = nlp_->getVariableValues().head(num_nlp_vars_);
  const Eigen::VectorXd cost_initial_value = nlp_->getCosts().getValues();

  // In the case of a QP problem the costs and constraints are represented as
  // quadratic functions is f(x) = a + b * x + c * x^2.
  // When convexifying the function it need to produce the same cost values at the values used to calculate
  // the Jacobian and Hessian, so f(x_initial) = a + b * x + c * x^2 = cost_initial_value.
  // Therefore a = cost_initial_value - b * x - c * x^2
  //     where: b = gradient_
  //            c = hessian_
  //            x = x_initial
  //            a = quadratic constant (cost_constant_)
  //
  // Note: This is not used by the QP solver directly but by the Trust Regions Solver
  //       to calculate the merit of the solve.

  // The block excludes the slack variables
  /** @todo I am not sure this is correct because the gradient_ is not each individual cost function gradient */
  const double result_quad = x_initial.transpose() * hessian_.block(0, 0, num_nlp_vars_, num_nlp_vars_) * x_initial;
  const double result_lin = x_initial.dot(gradient_.head(num_nlp_vars_));
  cost_constant_ = cost_initial_value.array() - result_quad - result_lin;
}

void IfoptQPProblem::updateConstraintsConstantExpression()
{
  if (num_nlp_cnts_ == 0)
    return;

  // Get values about which we will linearize
  const Eigen::VectorXd x_initial = nlp_->getVariableValues().head(num_nlp_vars_);
  const Eigen::VectorXd cnt_initial_value = nlp_->getConstraints().getValues();

  // In the case of a QP problem the costs and constraints are represented as
  // quadratic functions is f(x) = a + b * x + c * x^2.
  // Currently for constraints we do not leverage the Hessian so the quadratic
  // function is f(x) = a + b * x
  // When convexifying the function it need to produce the same constraint values at the values used to calculate
  // the jacobian, so f(x_initial) = a + b * x = cnt_initial_value.
  // Therefore a = cnt_initial_value - b * x
  //     where: b = jac (calculated below)
  //            x = x_initial
  //            a = quadratic constant (constraint_constant_)
  //
  // Note: This is not used by the QP solver directly but by the Trust Regions Solver
  //       to calculate the merit of the solve.

  // The block excludes the slack variables
  const trajopt_ifopt::Jacobian jac = constraint_matrix_.block(0, 0, num_nlp_cnts_, num_nlp_vars_);
  constraint_constant_ = (cnt_initial_value - jac * x_initial);
}

void IfoptQPProblem::updateNLPConstraintBounds()
{
  if (num_nlp_cnts_ == 0)
    return;

  Eigen::VectorXd cnt_bound_lower(num_nlp_cnts_);
  Eigen::VectorXd cnt_bound_upper(num_nlp_cnts_);

  // Convert constraint bounds to VectorXd
  std::vector<trajopt_ifopt::Bounds> cnt_bounds = nlp_->getBoundsOnConstraints();
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    cnt_bound_lower[i] = cnt_bounds[static_cast<std::size_t>(i)].getLower();
    cnt_bound_upper[i] = cnt_bounds[static_cast<std::size_t>(i)].getUpper();
  }

  const Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - constraint_constant_;
  const Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - constraint_constant_;

  // Insert linearized constraint bounds
  bounds_lower_.topRows(num_nlp_cnts_) = linearized_cnt_lower;
  bounds_upper_.topRows(num_nlp_cnts_) = linearized_cnt_upper;
}

void IfoptQPProblem::updateNLPVariableBounds()
{
  // This is equivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  const Eigen::VectorXd x_initial = nlp_->getVariableValues();

  // Set the variable limits once
  const std::vector<trajopt_ifopt::Bounds> var_bounds = nlp_->getBoundsOnOptimizationVariables();
  Eigen::VectorXd var_bounds_lower(num_nlp_vars_);
  Eigen::VectorXd var_bounds_upper(num_nlp_vars_);
  for (Eigen::Index i = 0; i < num_nlp_vars_; i++)
  {
    const auto& bounds = var_bounds[static_cast<std::size_t>(i)];
    var_bounds_lower[i] = bounds.getLower();
    var_bounds_upper[i] = bounds.getUpper();
  }

  // Calculate box constraints, while limiting to variable bounds and maintaining the trust region size
  const Eigen::VectorXd var_bounds_lower_final =
      (x_initial.cwiseMin(var_bounds_upper - box_size_) - box_size_).cwiseMax(var_bounds_lower);
  const Eigen::VectorXd var_bounds_upper_final =
      (x_initial.cwiseMax(var_bounds_lower + box_size_) + box_size_).cwiseMin(var_bounds_upper);
  bounds_lower_.block(num_nlp_cnts_, 0, var_bounds_lower_final.size(), 1) = var_bounds_lower_final;
  bounds_upper_.block(num_nlp_cnts_, 0, var_bounds_upper_final.size(), 1) = var_bounds_upper_final;
}

void IfoptQPProblem::updateSlackVariableBounds()
{
  Eigen::Index current_cnt_index = num_nlp_cnts_ + num_nlp_vars_;
  for (Eigen::Index i = 0; i < num_nlp_cnts_; i++)
  {
    const auto bounds_type = constraint_types_[static_cast<std::size_t>(i)];
    if (bounds_type == trajopt_ifopt::BoundsType::kEquality)
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index] = double(INFINITY);
      bounds_lower_[current_cnt_index + 1] = 0;
      bounds_upper_[current_cnt_index + 1] = double(INFINITY);

      current_cnt_index += 2;
    }
    else if (bounds_type == trajopt_ifopt::BoundsType::kLowerBound ||
             bounds_type == trajopt_ifopt::BoundsType::kUpperBound)
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index] = double(INFINITY);

      current_cnt_index++;
    }
    else
      throw std::runtime_error("Unsupported bounds type!");
  }
}

double IfoptQPProblem::evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  return evaluateConvexCosts(var_vals).sum();
}

Eigen::VectorXd IfoptQPProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  if (num_nlp_costs_ == 0)
    return {};

  auto var_block = var_vals.head(num_nlp_vars_);
  const double result_quad = var_block.transpose() * hessian_.block(0, 0, num_nlp_vars_, num_nlp_vars_) * var_block;
  const double result_lin = var_block.dot(gradient_.head(num_nlp_vars_));
  return cost_constant_.array() + result_lin + result_quad;
}

double IfoptQPProblem::getTotalExactCost() const { return nlp_->evaluateCostFunction(); }

Eigen::VectorXd IfoptQPProblem::getExactCosts() const
{
  if (!nlp_->hasCostTerms())
    return {};

  return nlp_->getCosts().getValues();
}

Eigen::VectorXd
IfoptQPProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  const Eigen::VectorXd result_lin =
      constraint_matrix_.block(0, 0, num_nlp_cnts_, num_nlp_vars_) * var_vals.head(num_nlp_vars_);  // NOLINT
  const Eigen::VectorXd constraint_value = constraint_constant_ + result_lin;

  Eigen::VectorXd violations(constraint_value.rows());
  trajopt_ifopt::calcBoundsViolations(violations, constraint_value, nlp_->getBoundsOnConstraints());
  return violations;
}

Eigen::VectorXd IfoptQPProblem::getExactConstraintViolations() const
{
  const Eigen::VectorXd cnt_vals = nlp_->evaluateConstraints();

  Eigen::VectorXd violations(cnt_vals.rows());
  trajopt_ifopt::calcBoundsViolations(violations, cnt_vals, nlp_->getBoundsOnConstraints());
  return violations;
}

void IfoptQPProblem::scaleBoxSize(double& scale)
{
  box_size_ = box_size_ * scale;
  updateNLPVariableBounds();
}

void IfoptQPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size)
{
  assert(box_size.size() == num_nlp_vars_);
  box_size_ = box_size;
  updateNLPVariableBounds();
}

void IfoptQPProblem::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  assert(merit_coeff.size() == num_nlp_cnts_);
  constraint_merit_coeff_ = merit_coeff;
}

void IfoptQPProblem::print() const
{
  const Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << '\n';
  std::cout << "Num NLP Vars: " << num_nlp_vars_ << '\n';
  std::cout << "Num QP Vars: " << num_qp_vars_ << '\n';
  std::cout << "Num NLP Constraints: " << num_qp_cnts_ << '\n';
  std::cout << "Detected Constraint Type: ";
  for (const auto& cnt : constraint_types_)
    std::cout << static_cast<int>(cnt) << ", ";

  std::cout << '\n';
  std::cout << "box_size_: " << box_size_.transpose().format(format) << '\n';  // NOLINT
  std::cout << "constraint_merit_coeff_: " << constraint_merit_coeff_.transpose().format(format) << '\n';

  std::cout << "Hessian:\n" << hessian_.toDense().format(format) << '\n';
  std::cout << "Gradient: " << gradient_.transpose().format(format) << '\n';
  std::cout << "Constraint Matrix:\n" << constraint_matrix_.toDense().format(format) << '\n';
  std::cout << "bounds_lower: " << bounds_lower_.transpose().format(format) << '\n';
  std::cout << "bounds_upper: " << bounds_upper_.transpose().format(format) << '\n';
  std::cout << "NLP values: " << nlp_->getVariableValues().transpose().format(format) << '\n';
}
}  // namespace trajopt_sqp
