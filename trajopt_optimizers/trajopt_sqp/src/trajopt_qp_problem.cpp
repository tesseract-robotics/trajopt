
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/expressions.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <iostream>

/**
 * In the previous version of trajopt it handled hinge cost and absolute costs differently.
 *
 * In the case of a hing cost which is used to convert an inequality constraint to a cost.
 * It would still add the inequality constraint to the QP Problem but the high level solver
 * was not aware so it did not factor into the merit calcuation so it did not affect convergance.
 *
 * In the case of a abs cost which is used to convert an equality constraint to a cost.
 * It would still add the equality constraint to the QP Problem but the high level solver
 * was not aware so it did not factor into the merit calcuation so it did not affect convergance.
 *
 * @todo Need to creat another version of this which converts hinge and absolute to cost functions
 * so this does not have to manage this internally. This would allow for other high level solvers
 * SNOPT to be leveraged.
 *
 * QP Variables: |NLP Vars, Hinge Cnt Cost Slack Variable, Absolute Cnt Cost Slack Variable, NLP Constraint Slack Vars |
 *
 * Constraint Matrix
 * | Hinge Cost Cnt Jacobian   , Hinge cost cnt slack variable jacobian    |
 * | Absolute Cost Cnt Jacobian, Absolute cost cnt slack variable jacobian |
 * | NLP constraint jacobian   , NLP constraint slack variable jacobian    |
 * |        QP Variable Jacobian (Diag Matrix of ones)                     |
 */

namespace trajopt_sqp
{
TrajOptQPProblem::TrajOptQPProblem()
  : constraints_("constraint-sets", false)
  , squared_costs_("squared-cost-terms", false)
  , hinge_costs_("hinge-cost-terms", false)
  , hinge_constraints_("hinge-constraint-sets", false)
  , abs_costs_("abs-cost-terms", false)
  , abs_constraints_("abs-constraint-sets", false)
{
  variables_ = std::make_shared<ifopt::Composite>("variable-sets", false);
}

void TrajOptQPProblem::addVariableSet(ifopt::VariableSet::Ptr variable_set)
{
  variables_->AddComponent(variable_set);
  initialized_ = false;
}

void TrajOptQPProblem::addConstraintSet(ifopt::ConstraintSet::Ptr constraint_set)
{
  constraint_set->LinkWithVariables(variables_);
  constraints_.AddComponent(constraint_set);
  initialized_ = false;
}

void TrajOptQPProblem::addCostSet(ifopt::ConstraintSet::Ptr constraint_set, CostPenaltyType penalty_type)
{
  constraint_set->LinkWithVariables(variables_);
  std::vector<ifopt::Bounds> cost_bounds = constraint_set->GetBounds();
  switch (penalty_type)
  {
    case CostPenaltyType::SQUARED:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt squared cost must have equality bounds!");
      }

      squared_costs_.AddComponent(constraint_set);
      break;
    }
    case CostPenaltyType::ABSOLUTE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt absolute cost must have equality bounds!");
      }

      abs_costs_.AddComponent(constraint_set);
      break;
    }
    case CostPenaltyType::HINGE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsInEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt hinge cost must have inequality bounds!");
      }

      hinge_costs_.AddComponent(constraint_set);
      break;
    }
    default:
    {
      throw std::runtime_error("Unsupport CostPenaltyType!");
    }
  }

  initialized_ = false;
}

void TrajOptQPProblem::setup()
{
  hinge_constraints_.ClearComponents();
  abs_constraints_.ClearComponents();
  squared_costs_target_ = Eigen::VectorXd::Zero(squared_costs_.GetRows());
  // Hinge cost adds a variable and an inequality constraint which equals two constraints added to the qp problem
  // Absolute cost add two variables and an equality constraint which equals three constraints added to the qp problem
  num_qp_vars_ = getNumNLPVars() + hinge_costs_.GetRows() + (2 * abs_costs_.GetRows());
  num_qp_cnts_ = getNumNLPConstraints() + getNumNLPVars() + (2 * hinge_costs_.GetRows()) + (3 * abs_costs_.GetRows());
  box_size_ = Eigen::VectorXd::Constant(getNumNLPVars(), 1e-1);
  constraint_merit_coeff_ = Eigen::VectorXd::Constant(getNumNLPConstraints(), 10);
  constraint_constant_ = Eigen::VectorXd::Zero(getNumNLPConstraints() + hinge_costs_.GetRows() + abs_costs_.GetRows());

  // Get NLP Cost and Constraint Names for Debug Print
  for (const auto& cnt : constraints_.GetComponents())
  {
    // Loop over each constraint in the set
    for (Eigen::Index j = 0; j < cnt->GetRows(); j++)
      constraint_names_.push_back(cnt->GetName() + "_" + std::to_string(j));
  }

  for (const auto& cost : squared_costs_.GetComponents())
  {
    std::vector<ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); j++)
    {
      assert(trajopt_ifopt::isBoundsEquality(cost_bounds[static_cast<std::size_t>(j)]));

      squared_costs_target_(j) = cost_bounds[static_cast<std::size_t>(j)].lower_;
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  for (const auto& cost : abs_costs_.GetComponents())
  {
    // Add to the qp problem solver constraints
    abs_constraints_.AddComponent(cost);

    std::vector<ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); j++)
    {
      assert(trajopt_ifopt::isBoundsEquality(cost_bounds[static_cast<std::size_t>(j)]));
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  for (const auto& cost : hinge_costs_.GetComponents())
  {
    // Add to the qp problem solver constraints
    hinge_constraints_.AddComponent(cost);

    std::vector<ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); j++)
    {
      assert(trajopt_ifopt::isBoundsInEquality(cost_bounds[static_cast<std::size_t>(j)]));
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  ////////////////////////////////////////////////////////
  // Get NLP bounds and detect constraint type
  ////////////////////////////////////////////////////////
  Eigen::VectorXd nlp_bounds_l(getNumNLPConstraints());
  Eigen::VectorXd nlp_bounds_u(getNumNLPConstraints());
  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = constraints_.GetBounds();
  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    nlp_bounds_l[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    nlp_bounds_u[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Detect constraint type
  Eigen::VectorXd nlp_bounds_diff = nlp_bounds_u - nlp_bounds_l;
  constraint_types_.resize(static_cast<std::size_t>(getNumNLPConstraints()));
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

  // Set initialized
  initialized_ = true;
}

void TrajOptQPProblem::setVariables(const double* x)
{
  variables_->SetVariables(Eigen::Map<const Eigen::VectorXd>(x, variables_->GetRows()));
}

Eigen::VectorXd TrajOptQPProblem::getVariableValues() const { return variables_->GetValues(); }

void TrajOptQPProblem::convexify()
{
  assert(initialized_);  // NOLINT

  // This must be called prior to updateGradient
  convexifyCosts();  // NOLINT

  linearizeConstraints();  // NOLINT

  // The three above must be called before rest to update internal data

  updateConstraintsConstantExpression();

  updateNLPConstraintBounds();

  updateNLPVariableBounds();

  updateSlackVariableBounds();
}

void TrajOptQPProblem::convexifyCosts()
{
  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  hessian_.resize(num_qp_vars_, num_qp_vars_);

  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  gradient_ = Eigen::VectorXd::Zero(num_qp_vars_);

  Eigen::VectorXd x_initial = variables_->GetValues().head(getNumNLPVars());
  Eigen::VectorXd cost_initial_value(getNumNLPCosts());

  // Create triplet list of nonzero gradients
  std::vector<Eigen::Triplet<double>> grad_triplet_list;
  grad_triplet_list.reserve(static_cast<std::size_t>(getNumNLPVars() * getNumNLPCosts()) * 3);

  // Process Squared Costs
  /** @note See CostFromFunc::convex in modeling_utils.cpp. */
  if (squared_costs_.GetRows() > 0)
  {
    squared_objective_nlp_ = QuadExprs(squared_costs_.GetRows(), getNumNLPVars());
    SparseMatrix cnt_jac = squared_costs_.GetJacobian();
    Eigen::VectorXd cnt_vals = squared_costs_.GetValues();

    // This is not correct should pass the value to createAffExprs then use bound to which could change the sign of the
    // affine expression
    //    Eigen::VectorXd cnt_error = trajopt_ifopt::calcBoundsErrors(cnt_vals, squared_costs_.GetBounds());

    // This should be correct now
    AffExprs cnt_aff_expr = createAffExprs(cnt_vals, cnt_jac, x_initial);
    cnt_aff_expr.constants = (squared_costs_target_ - cnt_aff_expr.constants);
    cnt_aff_expr.linear_coeffs *= -1;
    QuadExprs cost_quad_expr = squareAffExprs(cnt_aff_expr);

    // Sum objective function linear coefficients
    squared_objective_nlp_.objective_linear_coeffs += cost_quad_expr.objective_linear_coeffs;

    // Sum objective function quadratic coefficients
    squared_objective_nlp_.objective_quadratic_coeffs += cost_quad_expr.objective_quadratic_coeffs;

    // store individual equations constant
    squared_objective_nlp_.constants.topRows(squared_costs_.GetRows()) = cost_quad_expr.constants;

    // store individual equations linear coefficients in a Triplet list to update equation linear coefficients later
    if (cost_quad_expr.linear_coeffs.nonZeros() > 0)
    {
      // Add jacobian to triplet list
      for (int k = 0; k < cost_quad_expr.linear_coeffs.outerSize(); ++k)
      {
        for (SparseMatrix::InnerIterator it(cost_quad_expr.linear_coeffs, k); it; ++it)
          grad_triplet_list.emplace_back(it.row(), it.col(), it.value());
      }
    }

    // Store individual equations quadratic coefficients
    squared_objective_nlp_.quadratic_coeffs.insert(squared_objective_nlp_.quadratic_coeffs.end(),
                                                   cost_quad_expr.quadratic_coeffs.begin(),
                                                   cost_quad_expr.quadratic_coeffs.end());

    // Store individual equations linear coefficients
    squared_objective_nlp_.linear_coeffs.setFromTriplets(grad_triplet_list.begin(), grad_triplet_list.end());  // NOLINT

    // Insert QP Problem Objective Linear Coefficients
    gradient_.head(getNumNLPVars()) = squared_objective_nlp_.objective_linear_coeffs;

    // Insert QP Problem Objective Quadratic Coefficients
    if (squared_objective_nlp_.objective_quadratic_coeffs.nonZeros() > 0)
    {
      hessian_.reserve(squared_objective_nlp_.objective_quadratic_coeffs.nonZeros());
      for (int k = 0; k < squared_objective_nlp_.objective_quadratic_coeffs.outerSize(); ++k)
      {
        for (SparseMatrix::InnerIterator it(squared_objective_nlp_.objective_quadratic_coeffs, k); it; ++it)
          hessian_.coeffRef(it.row(), it.col()) += it.value();
      }
    }
  }

  // Hinge and Asolute costs are handled differently than squared cost because they add constraints to the qp problem

  Eigen::Index current_var_index = getNumNLPVars();
  ////////////////////////////////////////////////////////
  // Set the gradient of the hinge cost variables
  ////////////////////////////////////////////////////////
  for (Eigen::Index i = 0; i < hinge_costs_.GetRows(); i++)
  {
    gradient_[current_var_index++] = 1;  // This should be multiplied by the weight
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the absolute cost variables
  ////////////////////////////////////////////////////////
  for (Eigen::Index i = 0; i < 2 * abs_costs_.GetRows(); i++)
  {
    gradient_[current_var_index++] = 1;  // This should be multiplied by the weight
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the constraint slack variables
  ////////////////////////////////////////////////////////
  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      gradient_[current_var_index++] = constraint_merit_coeff_[i];
      gradient_[current_var_index++] = constraint_merit_coeff_[i];
    }
    else
    {
      gradient_[current_var_index++] = constraint_merit_coeff_[i];
    }
  }
}

void TrajOptQPProblem::linearizeConstraints()
{
  SparseMatrix nlp_cnt_jac = constraints_.GetJacobian();
  SparseMatrix hinge_cnt_jac = hinge_constraints_.GetJacobian();
  SparseMatrix abs_cnt_jac = abs_constraints_.GetJacobian();

  // Create triplet list of nonzero constraints
  using T = Eigen::Triplet<double>;
  std::vector<T> tripletList;
  tripletList.reserve(static_cast<std::size_t>(nlp_cnt_jac.nonZeros() + hinge_cnt_jac.nonZeros() +
                                               abs_cnt_jac.nonZeros() + num_qp_vars_) *
                      3);

  // Add hinge solver constraint jacobian to triplet list
  for (int k = 0; k < hinge_cnt_jac.outerSize(); ++k)
  {
    for (SparseMatrix::InnerIterator it(hinge_cnt_jac, k); it; ++it)
    {
      tripletList.emplace_back(it.row(), it.col(), it.value());
    }
  }

  // Add abs solver constraint jacobian to triplet list
  Eigen::Index current_row_index = hinge_constraints_.GetRows();
  for (int k = 0; k < abs_cnt_jac.outerSize(); ++k)
  {
    for (SparseMatrix::InnerIterator it(abs_cnt_jac, k); it; ++it)
    {
      tripletList.emplace_back(current_row_index + it.row(), it.col(), it.value());
    }
  }

  // Add nlp constraint jacobian to triplet list
  current_row_index += abs_constraints_.GetRows();
  for (int k = 0; k < nlp_cnt_jac.outerSize(); ++k)
  {
    for (SparseMatrix::InnerIterator it(nlp_cnt_jac, k); it; ++it)
    {
      tripletList.emplace_back(current_row_index + it.row(), it.col(), it.value());
    }
  }

  // Add the hinge variables to each hinge constraint
  Eigen::Index current_column_index = getNumNLPVars();
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(hinge_costs_.GetRows()); i++)
    tripletList.emplace_back(i, current_column_index++, -1);

  // Add the absolute variables to each hinge constraint
  current_row_index = hinge_costs_.GetRows();
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(abs_costs_.GetRows()); i++)
  {
    tripletList.emplace_back(current_row_index + i, current_column_index++, 1);
    tripletList.emplace_back(current_row_index + i, current_column_index++, -1);
  }

  // Add the slack variables to each constraint
  current_row_index += abs_costs_.GetRows();
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(constraint_types_.size()); i++)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      tripletList.emplace_back(current_row_index + i, current_column_index++, 1);
      tripletList.emplace_back(current_row_index + i, current_column_index++, -1);
    }
    else
    {
      tripletList.emplace_back(current_row_index + i, current_column_index++, -1);
    }
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  current_row_index = nlp_cnt_jac.rows() + hinge_cnt_jac.rows() + abs_cnt_jac.rows();
  for (Eigen::Index i = 0; i < num_qp_vars_; i++)
    tripletList.emplace_back(current_row_index + i, i, 1);

  // Insert the triplet list into the sparse matrix
  constraint_matrix_.resize(num_qp_cnts_, num_qp_vars_);
  constraint_matrix_.reserve(nlp_cnt_jac.nonZeros() + hinge_cnt_jac.nonZeros() + abs_cnt_jac.nonZeros() + num_qp_vars_);
  constraint_matrix_.setFromTriplets(tripletList.begin(), tripletList.end());  // NOLINT
}

void TrajOptQPProblem::updateConstraintsConstantExpression()
{
  long total_num_cnt = (getNumNLPConstraints() + hinge_constraints_.GetRows() + abs_constraints_.GetRows());
  if (total_num_cnt == 0)
    return;

  // Get values about which we will linearize
  Eigen::VectorXd x_initial = variables_->GetValues().head(getNumNLPVars());
  Eigen::Index current_row_index = 0;
  if (hinge_constraints_.GetRows() > 0)
  {  // Get values about which we will linearize
    Eigen::VectorXd cnt_initial_value = hinge_constraints_.GetValues();

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
    SparseMatrix jac = constraint_matrix_.block(current_row_index, 0, hinge_constraints_.GetRows(), getNumNLPVars());
    constraint_constant_.middleRows(current_row_index, hinge_constraints_.GetRows()) =
        (cnt_initial_value - jac * x_initial);
    current_row_index += hinge_constraints_.GetRows();
  }

  if (abs_constraints_.GetRows() > 0)
  {  // Get values about which we will linearize
    Eigen::VectorXd cnt_initial_value = abs_constraints_.GetValues();

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
    SparseMatrix jac = constraint_matrix_.block(current_row_index, 0, abs_constraints_.GetRows(), getNumNLPVars());
    constraint_constant_.middleRows(current_row_index, abs_constraints_.GetRows()) =
        (cnt_initial_value - jac * x_initial);
    current_row_index += abs_constraints_.GetRows();
  }

  if (constraints_.GetRows() > 0)
  {
    Eigen::VectorXd cnt_initial_value = constraints_.GetValues();

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
    SparseMatrix jac = constraint_matrix_.block(current_row_index, 0, getNumNLPConstraints(), getNumNLPVars());
    constraint_constant_.middleRows(current_row_index, getNumNLPConstraints()) = (cnt_initial_value - jac * x_initial);
  }
}

void TrajOptQPProblem::updateNLPConstraintBounds()
{
  long total_num_cnt = (getNumNLPConstraints() + hinge_constraints_.GetRows() + abs_constraints_.GetRows());
  if (total_num_cnt == 0)
    return;

  Eigen::VectorXd cnt_bound_lower(total_num_cnt);
  Eigen::VectorXd cnt_bound_upper(total_num_cnt);
  Eigen::Index current_row_index{ 0 };

  // Convert hinge constraint bounds to VectorXd
  std::vector<ifopt::Bounds> hinge_cnt_bounds = hinge_constraints_.GetBounds();
  for (Eigen::Index i = 0; i < hinge_constraints_.GetRows(); i++)
  {
    cnt_bound_lower[current_row_index + i] = hinge_cnt_bounds[static_cast<std::size_t>(i)].lower_;
    cnt_bound_upper[current_row_index + i] = hinge_cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }
  current_row_index += hinge_constraints_.GetRows();

  // Convert absolute constraint bounds to VectorXd
  std::vector<ifopt::Bounds> absolute_cnt_bounds = abs_constraints_.GetBounds();
  for (Eigen::Index i = 0; i < abs_constraints_.GetRows(); i++)
  {
    cnt_bound_lower[current_row_index + i] = absolute_cnt_bounds[static_cast<std::size_t>(i)].lower_;
    cnt_bound_upper[current_row_index + i] = absolute_cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }
  current_row_index += abs_constraints_.GetRows();

  // Convert nlp constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = constraints_.GetBounds();
  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    cnt_bound_lower[current_row_index + i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    cnt_bound_upper[current_row_index + i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - constraint_constant_;
  Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - constraint_constant_;

  // Insert linearized constraint bounds
  bounds_lower_.topRows(total_num_cnt) = linearized_cnt_lower;
  bounds_upper_.topRows(total_num_cnt) = linearized_cnt_upper;
}

void TrajOptQPProblem::updateNLPVariableBounds()
{
  // This is eqivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  Eigen::VectorXd x_initial = variables_->GetValues();

  // Calculate box constraints
  Eigen::VectorXd lower_box_cnt = x_initial - box_size_;
  Eigen::VectorXd upper_box_cnt = x_initial + box_size_;

  // Set the variable limits once
  std::vector<ifopt::Bounds> var_bounds = variables_->GetBounds();
  Eigen::VectorXd var_bounds_lower(getNumNLPVars());
  Eigen::VectorXd var_bounds_upper(getNumNLPVars());
  for (Eigen::Index i = 0; i < getNumNLPVars(); i++)
  {
    var_bounds_lower[i] = var_bounds[static_cast<std::size_t>(i)].lower_;
    var_bounds_upper[i] = var_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Apply box constraints and variable limits
  Eigen::VectorXd var_bounds_lower_final = var_bounds_lower.cwiseMax(lower_box_cnt);
  // Add the extra check here that the upper is bigger than the lower. It seems that there can be issues when the
  // numbers get close to 0.
  Eigen::VectorXd var_bounds_upper_final = var_bounds_upper.cwiseMin(upper_box_cnt).cwiseMax(var_bounds_lower);
  Eigen::Index var_row_index = getNumNLPConstraints() + hinge_constraints_.GetRows() + abs_constraints_.GetRows();
  bounds_lower_.block(var_row_index, 0, var_bounds_lower_final.size(), 1) = var_bounds_lower_final;
  bounds_upper_.block(var_row_index, 0, var_bounds_upper_final.size(), 1) = var_bounds_upper_final;
}

void TrajOptQPProblem::updateSlackVariableBounds()
{
  Eigen::Index current_cnt_index =
      getNumNLPConstraints() + hinge_constraints_.GetRows() + abs_constraints_.GetRows() + getNumNLPVars();

  for (Eigen::Index i = 0; i < hinge_costs_.GetRows(); i++)
  {
    bounds_lower_[current_cnt_index] = 0;
    bounds_upper_[current_cnt_index++] = double(INFINITY);
  }

  for (Eigen::Index i = 0; i < abs_costs_.GetRows(); i++)
  {
    bounds_lower_[current_cnt_index] = 0;
    bounds_upper_[current_cnt_index++] = double(INFINITY);
    bounds_lower_[current_cnt_index] = 0;
    bounds_upper_[current_cnt_index++] = double(INFINITY);
  }

  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index++] = double(INFINITY);
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index++] = double(INFINITY);
    }
    else
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index++] = double(INFINITY);
    }
  }
}

double TrajOptQPProblem::evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return evaluateConvexCosts(var_vals).sum();
}

Eigen::VectorXd TrajOptQPProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return Eigen::VectorXd();

  Eigen::VectorXd var_block = var_vals.head(getNumNLPVars());
  Eigen::VectorXd costs = Eigen::VectorXd::Zero(getNumNLPCosts());
  if (squared_costs_.GetRows() > 0)
  {
    costs.head(squared_costs_.GetRows()) = squared_objective_nlp_.values(var_block);
    assert(!(costs.head(squared_costs_.GetRows()).array() < 0).any());
  }

  if (hinge_costs_.GetRows() > 0)
  {
    Eigen::VectorXd hinge_cnt_constant = constraint_constant_.topRows(hinge_costs_.GetRows());
    auto hinge_cnt_jac = constraint_matrix_.block(0, 0, hinge_constraints_.GetRows(), getNumNLPVars());

    Eigen::VectorXd hinge_convex_value = hinge_cnt_constant + hinge_cnt_jac * var_block;
    Eigen::VectorXd hinge_cost = trajopt_ifopt::calcBoundsViolations(hinge_convex_value, hinge_costs_.GetBounds());

    costs.middleRows(squared_costs_.GetRows(), hinge_costs_.GetRows()) = hinge_cost;
    assert(!(costs.middleRows(squared_costs_.GetRows(), hinge_costs_.GetRows()).array() < 0).any());
  }

  if (abs_costs_.GetRows() > 0)
  {
    Eigen::VectorXd abs_cnt_constant = constraint_constant_.middleRows(hinge_costs_.GetRows(), abs_costs_.GetRows());
    auto abs_cnt_jac = constraint_matrix_.block(hinge_costs_.GetRows(), 0, abs_constraints_.GetRows(), getNumNLPVars());

    Eigen::VectorXd abs_convex_value = abs_cnt_constant + abs_cnt_jac * var_block;
    Eigen::VectorXd abs_cost = trajopt_ifopt::calcBoundsViolations(abs_convex_value, abs_costs_.GetBounds()).cwiseAbs();

    costs.middleRows(squared_costs_.GetRows() + hinge_costs_.GetRows(), abs_costs_.GetRows()) = abs_cost;
    assert(
        !(costs.middleRows(squared_costs_.GetRows() + hinge_costs_.GetRows(), abs_costs_.GetRows()).array() < 0).any());
  }
  return costs;
}

double TrajOptQPProblem::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return 0;

  double g{ 0 };
  setVariables(var_vals.data());

  if (squared_costs_.GetRows() > 0)
  {
    Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(squared_costs_.GetValues(), squared_costs_.GetBounds());
    assert((error.array() < 0).any() == false);
    g += error.squaredNorm();
  }

  if (abs_costs_.GetRows() > 0)
  {
    Eigen::VectorXd error =
        trajopt_ifopt::calcBoundsViolations(abs_costs_.GetValues(), abs_costs_.GetBounds()).cwiseAbs();
    assert((error.array() < 0).any() == false);
    g += error.sum();
  }

  if (hinge_costs_.GetRows() > 0)
  {
    Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(hinge_costs_.GetValues(), hinge_costs_.GetBounds());
    assert((error.array() < 0).any() == false);
    g += error.sum();
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return Eigen::VectorXd();

  setVariables(var_vals.data());

  Eigen::VectorXd g(getNumNLPCosts());
  Eigen::Index start_index = 0;
  if (squared_costs_.GetRows() > 0)
  {
    g.topRows(squared_costs_.GetRows()) =
        trajopt_ifopt::calcBoundsViolations(squared_costs_.GetValues(), squared_costs_.GetBounds()).array().square();
    start_index += squared_costs_.GetRows();
  }

  if (abs_costs_.GetRows() > 0)
  {
    g.middleRows(start_index, abs_costs_.GetRows()) =
        trajopt_ifopt::calcBoundsViolations(abs_costs_.GetValues(), abs_costs_.GetBounds()).cwiseAbs();
    start_index += abs_costs_.GetRows();
  }

  if (hinge_costs_.GetRows() > 0)
  {
    g.middleRows(start_index, hinge_costs_.GetRows()) =
        trajopt_ifopt::calcBoundsViolations(hinge_costs_.GetValues(), hinge_costs_.GetBounds());
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::getExactCosts() { return evaluateExactCosts(variables_->GetValues()); }

Eigen::VectorXd TrajOptQPProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  Eigen::Index row_index = hinge_constraints_.GetRows() + abs_costs_.GetRows();
  Eigen::VectorXd result_lin =
      constraint_matrix_.block(row_index, 0, getNumNLPConstraints(), getNumNLPVars()) * var_vals.head(getNumNLPVars());
  Eigen::VectorXd constraint_value = constraint_constant_.middleRows(row_index, getNumNLPConstraints()) + result_lin;
  return trajopt_ifopt::calcBoundsViolations(constraint_value, constraints_.GetBounds());
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  setVariables(var_vals.data());
  Eigen::VectorXd cnt_vals = constraints_.GetValues();
  return trajopt_ifopt::calcBoundsViolations(cnt_vals, constraints_.GetBounds());
}

Eigen::VectorXd TrajOptQPProblem::getExactConstraintViolations()
{
  return evaluateExactConstraintViolations(variables_->GetValues());  // NOLINT
}

void TrajOptQPProblem::scaleBoxSize(double& scale)
{
  box_size_ = box_size_ * scale;
  updateNLPVariableBounds();
}

void TrajOptQPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size)
{
  assert(box_size.size() == getNumNLPVars());
  box_size_ = box_size;
  updateNLPVariableBounds();
}

Eigen::VectorXd TrajOptQPProblem::getBoxSize() const { return box_size_; }

void TrajOptQPProblem::print() const
{
  Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << std::endl;
  std::cout << "Num NLP Vars: " << getNumNLPVars() << std::endl;
  std::cout << "Num QP Vars: " << num_qp_vars_ << std::endl;
  std::cout << "Num NLP Constraints: " << num_qp_cnts_ << std::endl;
  std::cout << "Detected Constraint Type: ";
  for (const auto& cnt : constraint_types_)
    std::cout << static_cast<int>(cnt) << ", ";

  std::cout << std::endl;
  std::cout << "Box Size: " << box_size_.transpose().format(format) << std::endl;
  std::cout << "Constraint Merit Coeff: " << constraint_merit_coeff_.transpose().format(format) << std::endl;

  std::cout << "Hessian:\n" << hessian_.toDense().format(format) << std::endl;
  std::cout << "Gradient: " << gradient_.transpose().format(format) << std::endl;
  std::cout << "Constraint Matrix:\n" << constraint_matrix_.toDense().format(format) << std::endl;
  std::cout << "Constraint Lower Bounds: "
            << bounds_lower_.head(getNumNLPConstraints() + hinge_constraints_.GetRows()).transpose().format(format)
            << std::endl;
  std::cout << "Constraint Upper Bounds: "
            << bounds_upper_.head(getNumNLPConstraints() + hinge_constraints_.GetRows()).transpose().format(format)
            << std::endl;
  std::cout << "Variable Lower Bounds: "
            << bounds_lower_.tail(bounds_lower_.rows() - getNumNLPConstraints() - hinge_constraints_.GetRows())
                   .transpose()
                   .format(format)
            << std::endl;
  std::cout << "Variable Upper Bounds: "
            << bounds_upper_.tail(bounds_upper_.rows() - getNumNLPConstraints() - hinge_constraints_.GetRows())
                   .transpose()
                   .format(format)
            << std::endl;
  std::cout << "All Lower Bounds: " << bounds_lower_.transpose().format(format) << std::endl;
  std::cout << "All Upper Bounds: " << bounds_upper_.transpose().format(format) << std::endl;
  std::cout << "NLP values: " << std::endl;
  for (const auto& v_set : variables_->GetComponents())
    std::cout << v_set->GetValues().transpose().format(format) << std::endl;
}

Eigen::Index TrajOptQPProblem::getNumNLPVars() const { return variables_->GetRows(); }
Eigen::Index TrajOptQPProblem::getNumNLPConstraints() const
{
  return static_cast<Eigen::Index>(constraints_.GetBounds().size());
}
Eigen::Index TrajOptQPProblem::getNumNLPCosts() const
{
  return (squared_costs_.GetRows() + abs_costs_.GetRows() + hinge_costs_.GetRows());
}
Eigen::Index TrajOptQPProblem::getNumQPVars() const { return num_qp_vars_; }
Eigen::Index TrajOptQPProblem::getNumQPConstraints() const { return num_qp_cnts_; }

const std::vector<std::string>& TrajOptQPProblem::getNLPConstraintNames() const { return constraint_names_; }
const std::vector<std::string>& TrajOptQPProblem::getNLPCostNames() const { return cost_names_; }

Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoxSize() { return box_size_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getConstraintMeritCoeff() { return constraint_merit_coeff_; }

Eigen::Ref<const SparseMatrix> TrajOptQPProblem::getHessian() { return hessian_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getGradient() { return gradient_; }

Eigen::Ref<const SparseMatrix> TrajOptQPProblem::getConstraintMatrix() { return constraint_matrix_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsLower() { return bounds_lower_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsUpper() { return bounds_upper_; }

}  // namespace trajopt_sqp
