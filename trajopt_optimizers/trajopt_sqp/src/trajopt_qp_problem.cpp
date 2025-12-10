
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/expressions.h>
#include <trajopt_sqp/types.h>

#include <trajopt_ifopt/utils/ifopt_utils.h>

#include <utility>
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
struct TrajOptQPProblem::Implementation
{
  Implementation()
    : constraints_("constraint-sets", false)
    , squared_costs_("squared-cost-terms", false)
    , hinge_costs_("hinge-cost-terms", false)
    , hinge_constraints_("hinge-constraint-sets", false)
    , abs_costs_("abs-cost-terms", false)
    , abs_constraints_("abs-constraint-sets", false)
  {
    variables_ = std::make_shared<ifopt::Composite>("variable-sets", false);
  }

  bool initialized_{ false };
  ifopt::Composite::Ptr variables_;
  ifopt::Composite constraints_;
  ifopt::Composite squared_costs_;
  ifopt::Composite hinge_costs_;
  ifopt::Composite hinge_constraints_;
  ifopt::Composite abs_costs_;
  ifopt::Composite abs_constraints_;
  Eigen::VectorXd squared_costs_target_;

  std::vector<ConstraintType> constraint_types_;

  Eigen::Index num_qp_vars_{ 0 };
  Eigen::Index num_qp_cnts_{ 0 };

  std::vector<std::string> constraint_names_;
  std::vector<std::string> cost_names_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;
  Eigen::VectorXd constraint_merit_coeff_;

  SparseMatrix hessian_;
  Eigen::VectorXd gradient_;
  QuadExprs squared_objective_nlp_;

  SparseMatrix constraint_matrix_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  // This should be the center of the bounds
  Eigen::VectorXd constraint_constant_;

  // Cached bounds (assumed static over SQP iterations)
  std::vector<ifopt::Bounds> nlp_cnt_bounds_;
  std::vector<ifopt::Bounds> hinge_cnt_bounds_;
  std::vector<ifopt::Bounds> abs_cnt_bounds_;

  std::vector<ifopt::Bounds> squared_cost_bounds_;
  std::vector<ifopt::Bounds> abs_cost_bounds_;
  std::vector<ifopt::Bounds> hinge_cost_bounds_;

  std::vector<ifopt::Bounds> var_bounds_;

  void addVariableSet(const std::shared_ptr<ifopt::VariableSet>& variable_set);

  void addConstraintSet(const std::shared_ptr<ifopt::ConstraintSet>& constraint_set);

  void addCostSet(const std::shared_ptr<ifopt::ConstraintSet>& constraint_set, CostPenaltyType penalty_type);

  void setup();

  void setVariables(const double* x);

  Eigen::VectorXd getVariableValues() const;

  void convexify();

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  double evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  void scaleBoxSize(double& scale);

  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size);

  void setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff);

  void print() const;

  Eigen::Index getNumNLPVars() const;
  Eigen::Index getNumNLPConstraints() const;
  Eigen::Index getNumNLPCosts() const;

  /**
   * @brief Helper that updates the objective function constant and linear and quadratic coefficients of the QP Problem
   * @details Called by convexify()
   */
  void convexifyCosts();

  /**
   * @brief Helper that linearizes the constraints about the current point, storing the
   * jacobian as the constraint matrix and adding slack variables.
   * @details Called by convexify()
   */
  void linearizeConstraints();

  /**
   * @brief Helper that updates the NLP constraint bounds (top section)
   * @details Called by convexify()
   */
  void updateNLPConstraintBounds();

  /**
   * @brief Helper that updates the NLP variable bounds (middle section)
   * @details Called by convexify()
   */
  void updateNLPVariableBounds();

  /**
   * @brief Called by convexify() - helper that updates the slack variable bounds (bottom section)
   * @details A slack variable is referred to as an additional variable that has been introduced
   * to the optimization problem to turn a inequality constraint into an equality constraint.
   * Information taken from: https://en.wikipedia.org/wiki/Slack_variable
   *
   * As with the other variables in the augmented constraints, the slack variable cannot take on
   * negative values, as the simplex algorithm requires them to be positive or zero.
   *
   *   - If a slack variable associated with a constraint is zero at a particular candidate solution,
   *     the constraint is binding there, as the constraint restricts the possible changes from that point.
   *   - If a slack variable is positive at a particular candidate solution, the constraint is non-binding
   *     there, as the constraint does not restrict the possible changes from that point.
   *   - If a slack variable is negative at some point, the point is infeasible (not allowed), as it does
   *     not satisfy the constraint.
   *
   * Terminology
   *
   *   - If an inequality constraint holds with equality at the optimal point, the constraint is said to
   *     be binding, as the point cannot be varied in the direction of the constraint even though doing
   *     so would improve the value of the objective function.
   *   - If an inequality constraint holds as a strict inequality at the optimal point (that is, does
   *     not hold with equality), the constraint is said to be non-binding, as the point could be varied
   *     in the direction of the constraint, although it would not be optimal to do so. Under certain
   *     conditions, as for example in convex optimization, if a constraint is non-binding, the optimization
   *     problem would have the same solution even in the absence of that constraint.
   *   - If a constraint is not satisfied at a given point, the point is said to be infeasible.
   *
   * @example By introducing the slack variable y >= 0, the inequality Ax <= b can be converted
   * to the equation Ax + y = b.
   */
  void updateSlackVariableBounds();

  /** @brief This calculates the constant expression in the quadratic expression for the constraints */
  void updateConstraintsConstantExpression();
};

void TrajOptQPProblem::Implementation::addVariableSet(const std::shared_ptr<ifopt::VariableSet>& variable_set)
{
  variables_->AddComponent(variable_set);
  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addConstraintSet(const std::shared_ptr<ifopt::ConstraintSet>& constraint_set)
{
  constraint_set->LinkWithVariables(variables_);
  constraints_.AddComponent(constraint_set);
  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addCostSet(const std::shared_ptr<ifopt::ConstraintSet>& constraint_set,
                                                  CostPenaltyType penalty_type)
{
  constraint_set->LinkWithVariables(variables_);
  const std::vector<ifopt::Bounds> cost_bounds = constraint_set->GetBounds();
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

void TrajOptQPProblem::Implementation::setup()
{
  // Local counts to avoid repeated virtual / composite queries
  const Eigen::Index n_nlp_vars = getNumNLPVars();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_nlp_cnts = getNumNLPConstraints();
  const Eigen::Index n_nlp_costs = getNumNLPCosts();

  hinge_constraints_.ClearComponents();
  abs_constraints_.ClearComponents();

  squared_costs_target_ = Eigen::VectorXd::Zero(squared_costs_.GetRows());

  // Hinge cost adds a variable and an inequality constraint (→ 2 constraints)
  // Absolute cost adds two variables and an equality constraint (→ 3 constraints)
  num_qp_vars_ = n_nlp_vars + n_hinge + (2L * n_abs);
  num_qp_cnts_ = n_nlp_cnts + n_nlp_vars + (2L * n_hinge) + (3L * n_abs);

  box_size_ = Eigen::VectorXd::Constant(n_nlp_vars, 1e-1);
  constraint_merit_coeff_ = Eigen::VectorXd::Constant(n_nlp_cnts, 10.0);
  constraint_constant_ = Eigen::VectorXd::Zero(n_nlp_cnts + n_hinge + n_abs);

  // Reset and reserve name buffers (avoid accumulation across multiple setup() calls)
  constraint_names_.clear();
  cost_names_.clear();
  constraint_names_.reserve(static_cast<std::size_t>(n_nlp_cnts));
  cost_names_.reserve(static_cast<std::size_t>(n_nlp_costs));

  // Get NLP Cost and Constraint Names for Debug Print
  for (const auto& cnt : constraints_.GetComponents())
  {
    for (Eigen::Index j = 0; j < cnt->GetRows(); ++j)
      constraint_names_.push_back(cnt->GetName() + "_" + std::to_string(j));
  }

  for (const auto& cost : squared_costs_.GetComponents())
  {
    const std::vector<ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); ++j)
    {
      assert(trajopt_ifopt::isBoundsEquality(cost_bounds[static_cast<std::size_t>(j)]));
      squared_costs_target_(j) = cost_bounds[static_cast<std::size_t>(j)].lower_;
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  for (const auto& cost : abs_costs_.GetComponents())
  {
    abs_constraints_.AddComponent(cost);

    const std::vector<ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); ++j)
    {
      assert(trajopt_ifopt::isBoundsEquality(cost_bounds[static_cast<std::size_t>(j)]));
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  for (const auto& cost : hinge_costs_.GetComponents())
  {
    hinge_constraints_.AddComponent(cost);

    const std::vector<ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); ++j)
    {
      assert(trajopt_ifopt::isBoundsInEquality(cost_bounds[static_cast<std::size_t>(j)]));
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  // Cache flattened bounds for costs (used in exact cost evaluation)
  squared_cost_bounds_ = squared_costs_.GetBounds();
  abs_cost_bounds_ = abs_costs_.GetBounds();
  hinge_cost_bounds_ = hinge_costs_.GetBounds();

  ////////////////////////////////////////////////////////
  // Get NLP bounds and detect constraint type
  ////////////////////////////////////////////////////////
  nlp_cnt_bounds_ = constraints_.GetBounds();

  Eigen::VectorXd nlp_bounds_l(n_nlp_cnts);
  Eigen::VectorXd nlp_bounds_u(n_nlp_cnts);

  const std::vector<ifopt::Bounds> cnt_bounds = constraints_.GetBounds();
  for (Eigen::Index i = 0; i < n_nlp_cnts; ++i)
  {
    nlp_bounds_l[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    nlp_bounds_u[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  const Eigen::VectorXd nlp_bounds_diff = nlp_bounds_u - nlp_bounds_l;
  constraint_types_.resize(static_cast<std::size_t>(n_nlp_cnts));

  for (std::size_t i = 0; i < static_cast<std::size_t>(nlp_bounds_diff.size()); ++i)
  {
    if (std::abs(nlp_bounds_diff[static_cast<Eigen::Index>(i)]) < 1e-3)
    {
      constraint_types_[i] = ConstraintType::EQ;
      num_qp_vars_ += 2;  // L1 slack pair
      num_qp_cnts_ += 2;
    }
    else
    {
      constraint_types_[i] = ConstraintType::INEQ;
      num_qp_vars_ += 1;  // hinge slack
      num_qp_cnts_ += 1;
    }
  }

  // Cache constraint bounds for hinge / abs constraint composites
  hinge_cnt_bounds_ = hinge_constraints_.GetBounds();
  abs_cnt_bounds_ = abs_constraints_.GetBounds();

  // Cache variable bounds (used in updateNLPVariableBounds)
  var_bounds_ = variables_->GetBounds();

  // Initialize the constraint bounds
  bounds_lower_ = Eigen::VectorXd::Constant(num_qp_cnts_, -double(INFINITY));
  bounds_upper_ = Eigen::VectorXd::Constant(num_qp_cnts_, double(INFINITY));

  initialized_ = true;
}

void TrajOptQPProblem::Implementation::setVariables(const double* x)
{
  variables_->SetVariables(Eigen::Map<const Eigen::VectorXd>(x, variables_->GetRows()));
}

Eigen::VectorXd TrajOptQPProblem::Implementation::getVariableValues() const { return variables_->GetValues(); }

void TrajOptQPProblem::Implementation::convexify()
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

Eigen::VectorXd TrajOptQPProblem::Implementation::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return {};

  const Eigen::Index nlp_vars = getNumNLPVars();
  const Eigen::Index n_sq = squared_costs_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index total_cost = n_sq + n_hinge + n_abs;

  Eigen::VectorXd costs = Eigen::VectorXd::Zero(total_cost);
  Eigen::VectorXd var_block = var_vals.head(nlp_vars);

  // Squared costs (already convexified into squared_objective_nlp_)
  if (n_sq > 0)
  {
    costs.head(n_sq) = squared_objective_nlp_.values(var_block);
    assert(!(costs.head(n_sq).array() < -1e-8).any());
  }

  // Hinge costs
  if (n_hinge > 0)
  {
    const Eigen::VectorXd hinge_cnt_constant = constraint_constant_.topRows(n_hinge);
    const auto hinge_cnt_jac = constraint_matrix_.block(0, 0, hinge_constraints_.GetRows(), nlp_vars);

    const Eigen::VectorXd hinge_convex_value = hinge_cnt_constant + hinge_cnt_jac * var_block;
    const Eigen::VectorXd hinge_cost = trajopt_ifopt::calcBoundsViolations(hinge_convex_value, hinge_cost_bounds_);

    costs.segment(n_sq, n_hinge) = hinge_cost;
    assert(!(costs.segment(n_sq, n_hinge).array() < -1e-8).any());
  }

  // Absolute costs
  if (n_abs > 0)
  {
    const Eigen::Index abs_row_offset = n_hinge;
    const Eigen::VectorXd abs_cnt_constant = constraint_constant_.segment(abs_row_offset, n_abs);
    const auto abs_cnt_jac = constraint_matrix_.block(abs_row_offset, 0, abs_constraints_.GetRows(), nlp_vars);
    const Eigen::VectorXd abs_convex_value = abs_cnt_constant + abs_cnt_jac * var_block;

    // calcBoundsViolations already returns |violation|
    const Eigen::VectorXd abs_cost = trajopt_ifopt::calcBoundsViolations(abs_convex_value, abs_cost_bounds_);

    costs.segment(n_sq + n_hinge, n_abs) = abs_cost;
    assert(!(costs.segment(n_sq + n_hinge, n_abs).array() < -1e-8).any());
  }

  return costs;
}

double TrajOptQPProblem::Implementation::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)  // NOLINT
    return 0;

  double g{ 0 };
  setVariables(var_vals.data());

  if (squared_costs_.GetRows() > 0)
  {
    const Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(squared_costs_.GetValues(), squared_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.squaredNorm();
  }

  if (abs_costs_.GetRows() > 0)
  {
    const Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(abs_costs_.GetValues(), abs_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.sum();
  }

  if (hinge_costs_.GetRows() > 0)
  {
    const Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(hinge_costs_.GetValues(), hinge_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.sum();
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::Implementation::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return {};

  setVariables(var_vals.data());

  Eigen::VectorXd g(getNumNLPCosts());
  Eigen::Index idx = 0;

  if (squared_costs_.GetRows() > 0)
  {
    const Eigen::VectorXd err = trajopt_ifopt::calcBoundsViolations(squared_costs_.GetValues(), squared_cost_bounds_);
    g.segment(idx, squared_costs_.GetRows()) = err.array().square().matrix();
    idx += squared_costs_.GetRows();
  }

  if (abs_costs_.GetRows() > 0)
  {
    const Eigen::VectorXd err = trajopt_ifopt::calcBoundsViolations(abs_costs_.GetValues(), abs_cost_bounds_);
    g.segment(idx, abs_costs_.GetRows()) = err;
    idx += abs_costs_.GetRows();
  }

  if (hinge_costs_.GetRows() > 0)
  {
    const Eigen::VectorXd err = trajopt_ifopt::calcBoundsViolations(hinge_costs_.GetValues(), hinge_cost_bounds_);
    g.segment(idx, hinge_costs_.GetRows()) = err;
  }

  return g;
}

Eigen::VectorXd
TrajOptQPProblem::Implementation::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  const Eigen::Index row_index = hinge_constraints_.GetRows() + abs_costs_.GetRows();
  // NOLINTNEXTLINE
  Eigen::VectorXd result_lin = constraint_matrix_.block(row_index, 0, getNumNLPConstraints(), getNumNLPVars()) *
                               var_vals.head(getNumNLPVars());  // NOLINT
  const Eigen::VectorXd constraint_value =
      constraint_constant_.middleRows(row_index, getNumNLPConstraints()) + result_lin;
  return trajopt_ifopt::calcBoundsViolations(constraint_value, nlp_cnt_bounds_);
}

Eigen::VectorXd
TrajOptQPProblem::Implementation::evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  setVariables(var_vals.data());
  const Eigen::VectorXd cnt_vals = constraints_.GetValues();
  return trajopt_ifopt::calcBoundsViolations(cnt_vals, nlp_cnt_bounds_);
}

void TrajOptQPProblem::Implementation::scaleBoxSize(double& scale)
{
  box_size_ = box_size_ * scale;
  updateNLPVariableBounds();
}

void TrajOptQPProblem::Implementation::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size)
{
  assert(box_size.size() == getNumNLPVars());
  box_size_ = box_size;
  updateNLPVariableBounds();
}

void TrajOptQPProblem::Implementation::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  assert(merit_coeff.size() == getNumNLPConstraints());
  constraint_merit_coeff_ = merit_coeff;
}

void TrajOptQPProblem::Implementation::print() const
{
  const Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << '\n';
  std::cout << "Num NLP Vars: " << getNumNLPVars() << '\n';
  std::cout << "Num QP Vars: " << num_qp_vars_ << '\n';
  std::cout << "Num NLP Constraints: " << num_qp_cnts_ << '\n';
  std::cout << "Detected Constraint Type: ";
  for (const auto& cnt : constraint_types_)
    std::cout << static_cast<int>(cnt) << ", ";

  std::cout << '\n';
  std::cout << "Box Size: " << box_size_.transpose().format(format) << '\n';  // NOLINT
  std::cout << "Constraint Merit Coeff: " << constraint_merit_coeff_.transpose().format(format) << '\n';

  std::cout << "Hessian:\n" << hessian_.toDense().format(format) << '\n';
  std::cout << "Gradient: " << gradient_.transpose().format(format) << '\n';
  std::cout << "Constraint Matrix:\n" << constraint_matrix_.toDense().format(format) << '\n';
  std::cout << "Constraint Lower Bounds: "
            << bounds_lower_.head(getNumNLPConstraints() + hinge_constraints_.GetRows()).transpose().format(format)
            << '\n';
  std::cout << "Constraint Upper Bounds: "
            << bounds_upper_.head(getNumNLPConstraints() + hinge_constraints_.GetRows()).transpose().format(format)
            << '\n';
  std::cout << "Variable Lower Bounds: "
            << bounds_lower_.tail(bounds_lower_.rows() - getNumNLPConstraints() - hinge_constraints_.GetRows())
                   .transpose()
                   .format(format)
            << '\n';
  std::cout << "Variable Upper Bounds: "
            << bounds_upper_.tail(bounds_upper_.rows() - getNumNLPConstraints() - hinge_constraints_.GetRows())
                   .transpose()
                   .format(format)
            << '\n';
  std::cout << "All Lower Bounds: " << bounds_lower_.transpose().format(format) << '\n';
  std::cout << "All Upper Bounds: " << bounds_upper_.transpose().format(format) << '\n';
  std::cout << "NLP values: " << '\n';
  for (const auto& v_set : variables_->GetComponents())
    std::cout << v_set->GetValues().transpose().format(format) << '\n';
}

Eigen::Index TrajOptQPProblem::Implementation::getNumNLPVars() const { return variables_->GetRows(); }

Eigen::Index TrajOptQPProblem::Implementation::getNumNLPConstraints() const { return constraints_.GetRows(); }

Eigen::Index TrajOptQPProblem::Implementation::getNumNLPCosts() const
{
  return (squared_costs_.GetRows() + abs_costs_.GetRows() + hinge_costs_.GetRows());
}

void TrajOptQPProblem::Implementation::convexifyCosts()
{
  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  hessian_.resize(num_qp_vars_, num_qp_vars_);

  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  gradient_ = Eigen::VectorXd::Zero(num_qp_vars_);

  const Eigen::VectorXd x_initial = variables_->GetValues().head(getNumNLPVars());

  // Create triplet list of nonzero gradients
  std::vector<Eigen::Triplet<double>> grad_triplet_list;
  grad_triplet_list.reserve(static_cast<std::size_t>(getNumNLPVars() * getNumNLPCosts()) * 3);

  // Process Squared Costs
  /** @note See CostFromFunc::convex in modeling_utils.cpp. */
  if (squared_costs_.GetRows() > 0)
  {
    squared_objective_nlp_ = QuadExprs(squared_costs_.GetRows(), getNumNLPVars());
    const SparseMatrix cnt_jac = squared_costs_.GetJacobian();
    const Eigen::VectorXd cnt_vals = squared_costs_.GetValues();

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
    squared_objective_nlp_.quadratic_coeffs.reserve(squared_objective_nlp_.quadratic_coeffs.size() +
                                                    cost_quad_expr.quadratic_coeffs.size());
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
    gradient_[current_var_index++] = 1; /** @todo This should be multiplied by the weight */
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the absolute cost variables
  ////////////////////////////////////////////////////////
  for (Eigen::Index i = 0; i < 2L * abs_costs_.GetRows(); i++)
  {
    gradient_[current_var_index++] = 1; /** @todo This should be multiplied by the weight */
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

void TrajOptQPProblem::Implementation::linearizeConstraints()
{
  const SparseMatrix nlp_cnt_jac = constraints_.GetJacobian();
  const SparseMatrix hinge_cnt_jac = hinge_constraints_.GetJacobian();
  const SparseMatrix abs_cnt_jac = abs_constraints_.GetJacobian();

  std::vector<Eigen::Triplet<double>> triplets;
  const Eigen::Index nnz_base = nlp_cnt_jac.nonZeros() + hinge_cnt_jac.nonZeros() + abs_cnt_jac.nonZeros();
  // Rough but closer than *3
  triplets.reserve(static_cast<std::size_t>(nnz_base + num_qp_vars_ +  // diag
                                            hinge_costs_.GetRows() + (2L * abs_costs_.GetRows()) +
                                            (2L * getNumNLPConstraints())));

  // hinge constraints
  for (int k = 0; k < hinge_cnt_jac.outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(hinge_cnt_jac, k); it; ++it)
      triplets.emplace_back(it.row(), it.col(), it.value());

  // abs constraints (shifted rows)
  Eigen::Index current_row_index = hinge_constraints_.GetRows();
  for (int k = 0; k < abs_cnt_jac.outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(abs_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // nlp constraints (shift again)
  current_row_index += abs_constraints_.GetRows();
  for (int k = 0; k < nlp_cnt_jac.outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(nlp_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // hinge slack vars
  Eigen::Index current_col_index = getNumNLPVars();
  for (Eigen::Index i = 0; i < hinge_costs_.GetRows(); ++i)
    triplets.emplace_back(i, current_col_index++, -1.0);

  // abs slack vars
  current_row_index = hinge_costs_.GetRows();
  for (Eigen::Index i = 0; i < abs_costs_.GetRows(); ++i)
  {
    triplets.emplace_back(current_row_index + i, current_col_index++, 1.0);
    triplets.emplace_back(current_row_index + i, current_col_index++, -1.0);
  }

  // constraint slack vars
  current_row_index += abs_costs_.GetRows();
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(constraint_types_.size()); ++i)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      triplets.emplace_back(current_row_index + i, current_col_index++, 1.0);
      triplets.emplace_back(current_row_index + i, current_col_index++, -1.0);
    }
    else
    {
      triplets.emplace_back(current_row_index + i, current_col_index++, -1.0);
    }
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  current_row_index = nlp_cnt_jac.rows() + hinge_cnt_jac.rows() + abs_cnt_jac.rows();
  for (Eigen::Index i = 0; i < num_qp_vars_; ++i)
    triplets.emplace_back(current_row_index + i, i, 1.0);

  // Insert the triplet list into the sparse matrix
  constraint_matrix_.resize(num_qp_cnts_, num_qp_vars_);
  constraint_matrix_.reserve(nnz_base + num_qp_vars_);
  constraint_matrix_.setFromTriplets(triplets.begin(), triplets.end());  // NOLINT
}

void TrajOptQPProblem::Implementation::updateConstraintsConstantExpression()
{
  const Eigen::Index n_nlp = getNumNLPVars();
  const Eigen::Index n_hinge = hinge_constraints_.GetRows();
  const Eigen::Index n_abs = abs_constraints_.GetRows();
  const Eigen::Index n_nlp_cnt = getNumNLPConstraints();
  const Eigen::Index total_num_cnt = n_hinge + n_abs + n_nlp_cnt;

  if (total_num_cnt == 0)
    return;

  // Get values about which we will linearize
  const Eigen::VectorXd x_initial = variables_->GetValues().head(n_nlp);

  // One mat-vec for all constraint rows (excluding slack columns)
  const Eigen::VectorXd lin = constraint_matrix_.block(0, 0, total_num_cnt, n_nlp) * x_initial;

  Eigen::Index row = 0;
  if (n_hinge > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = hinge_constraints_.GetValues();

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
    constraint_constant_.segment(row, n_hinge) = cnt_initial_value - lin.segment(row, n_hinge);
    row += n_hinge;
  }

  if (n_abs > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = abs_constraints_.GetValues();

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
    constraint_constant_.segment(row, n_abs) = cnt_initial_value - lin.segment(row, n_abs);
    row += n_abs;
  }

  if (n_nlp_cnt > 0)
  {
    const Eigen::VectorXd cnt_initial_value = constraints_.GetValues();

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
    constraint_constant_.segment(row, n_nlp_cnt) = cnt_initial_value - lin.segment(row, n_nlp_cnt);
  }
}

void TrajOptQPProblem::Implementation::updateNLPConstraintBounds()
{
  const Eigen::Index n_hinge = hinge_constraints_.GetRows();
  const Eigen::Index n_abs = abs_constraints_.GetRows();
  const Eigen::Index n_nlp_cnt = getNumNLPConstraints();
  const Eigen::Index total_num_cnt = n_hinge + n_abs + n_nlp_cnt;

  if (total_num_cnt == 0)
    return;

  Eigen::VectorXd cnt_bound_lower(total_num_cnt);
  Eigen::VectorXd cnt_bound_upper(total_num_cnt);
  Eigen::Index row = 0;

  // Hinge constraint bounds
  for (Eigen::Index i = 0; i < n_hinge; ++i)
  {
    const auto& b = hinge_cnt_bounds_[static_cast<std::size_t>(i)];
    cnt_bound_lower[row + i] = b.lower_;
    cnt_bound_upper[row + i] = b.upper_;
  }
  row += n_hinge;

  // Absolute constraint bounds
  for (Eigen::Index i = 0; i < n_abs; ++i)
  {
    const auto& b = abs_cnt_bounds_[static_cast<std::size_t>(i)];
    cnt_bound_lower[row + i] = b.lower_;
    cnt_bound_upper[row + i] = b.upper_;
  }
  row += n_abs;

  // NLP constraint bounds
  for (Eigen::Index i = 0; i < n_nlp_cnt; ++i)
  {
    const auto& b = nlp_cnt_bounds_[static_cast<std::size_t>(i)];
    cnt_bound_lower[row + i] = b.lower_;
    cnt_bound_upper[row + i] = b.upper_;
  }

  const Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - constraint_constant_;
  const Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - constraint_constant_;

  bounds_lower_.topRows(total_num_cnt) = linearized_cnt_lower;
  bounds_upper_.topRows(total_num_cnt) = linearized_cnt_upper;
}

void TrajOptQPProblem::Implementation::updateNLPVariableBounds()
{
  // Equivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  const Eigen::Index n_nlp_vars = getNumNLPVars();
  const Eigen::Index n_hinge = hinge_constraints_.GetRows();
  const Eigen::Index n_abs = abs_constraints_.GetRows();

  const Eigen::VectorXd x_initial = variables_->GetValues();

  // Set the variable limits once
  Eigen::VectorXd var_bounds_lower(n_nlp_vars);
  Eigen::VectorXd var_bounds_upper(n_nlp_vars);

  for (Eigen::Index i = 0; i < n_nlp_vars; ++i)
  {
    const auto& b = var_bounds_[static_cast<std::size_t>(i)];
    var_bounds_lower[i] = b.lower_;
    var_bounds_upper[i] = b.upper_;
  }

  // Calculate box constraints, while limiting to variable bounds and maintaining the trust region size
  const Eigen::VectorXd var_bounds_lower_final =
      (x_initial.cwiseMin(var_bounds_upper - box_size_) - box_size_).cwiseMax(var_bounds_lower);
  const Eigen::VectorXd var_bounds_upper_final =
      (x_initial.cwiseMax(var_bounds_lower + box_size_) + box_size_).cwiseMin(var_bounds_upper);

  const Eigen::Index var_row_index = getNumNLPConstraints() + n_hinge + n_abs;

  bounds_lower_.segment(var_row_index, n_nlp_vars) = var_bounds_lower_final;
  bounds_upper_.segment(var_row_index, n_nlp_vars) = var_bounds_upper_final;
}

void TrajOptQPProblem::Implementation::updateSlackVariableBounds()
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

TrajOptQPProblem::TrajOptQPProblem() : impl_(std::make_unique<Implementation>()) {}

TrajOptQPProblem::~TrajOptQPProblem() = default;

void TrajOptQPProblem::addVariableSet(std::shared_ptr<ifopt::VariableSet> variable_set)
{
  impl_->addVariableSet(variable_set);
}

void TrajOptQPProblem::addConstraintSet(std::shared_ptr<ifopt::ConstraintSet> constraint_set)
{
  impl_->addConstraintSet(constraint_set);
}

void TrajOptQPProblem::addCostSet(std::shared_ptr<ifopt::ConstraintSet> constraint_set, CostPenaltyType penalty_type)
{
  impl_->addCostSet(constraint_set, penalty_type);
}

void TrajOptQPProblem::setup() { impl_->setup(); }

void TrajOptQPProblem::setVariables(const double* x) { impl_->setVariables(x); }

Eigen::VectorXd TrajOptQPProblem::getVariableValues() const
{
  return std::as_const<Implementation>(*impl_).getVariableValues();
}

// NOLINTNEXTLINE
void TrajOptQPProblem::convexify() { impl_->convexify(); }

double TrajOptQPProblem::evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return evaluateConvexCosts(var_vals).sum();
}

Eigen::VectorXd TrajOptQPProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateConvexCosts(var_vals);
}

double TrajOptQPProblem::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateTotalExactCost(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateExactCosts(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::getExactCosts() { return evaluateExactCosts(impl_->variables_->GetValues()); }

Eigen::VectorXd TrajOptQPProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateConvexConstraintViolations(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateExactConstraintViolations(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::getExactConstraintViolations()
{
  return evaluateExactConstraintViolations(impl_->variables_->GetValues());  // NOLINT
}

void TrajOptQPProblem::scaleBoxSize(double& scale) { impl_->scaleBoxSize(scale); }

void TrajOptQPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) { impl_->setBoxSize(box_size); }

void TrajOptQPProblem::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  impl_->setConstraintMeritCoeff(merit_coeff);
}

Eigen::VectorXd TrajOptQPProblem::getBoxSize() const { return std::as_const<Implementation>(*impl_).box_size_; }

void TrajOptQPProblem::print() const { std::as_const<Implementation>(*impl_).print(); }

Eigen::Index TrajOptQPProblem::getNumNLPVars() const { return std::as_const<Implementation>(*impl_).getNumNLPVars(); }
Eigen::Index TrajOptQPProblem::getNumNLPConstraints() const
{
  return std::as_const<Implementation>(*impl_).getNumNLPConstraints();
}
Eigen::Index TrajOptQPProblem::getNumNLPCosts() const { return std::as_const<Implementation>(*impl_).getNumNLPCosts(); }

Eigen::Index TrajOptQPProblem::getNumQPVars() const { return std::as_const<Implementation>(*impl_).num_qp_vars_; }
Eigen::Index TrajOptQPProblem::getNumQPConstraints() const
{
  return std::as_const<Implementation>(*impl_).num_qp_cnts_;
}

const std::vector<std::string>& TrajOptQPProblem::getNLPConstraintNames() const
{
  return std::as_const<Implementation>(*impl_).constraint_names_;
}
const std::vector<std::string>& TrajOptQPProblem::getNLPCostNames() const
{
  return std::as_const<Implementation>(*impl_).cost_names_;
}

Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoxSize() { return impl_->box_size_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getConstraintMeritCoeff() { return impl_->constraint_merit_coeff_; }

Eigen::Ref<const SparseMatrix> TrajOptQPProblem::getHessian() { return impl_->hessian_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getGradient() { return impl_->gradient_; }

Eigen::Ref<const SparseMatrix> TrajOptQPProblem::getConstraintMatrix() { return impl_->constraint_matrix_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsLower() { return impl_->bounds_lower_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsUpper() { return impl_->bounds_upper_; }

}  // namespace trajopt_sqp
