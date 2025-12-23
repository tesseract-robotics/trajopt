
#include <trajopt_ifopt/core/variable_set.h>
#include <trajopt_ifopt/core/constraint_set.h>

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/expressions.h>
#include <trajopt_sqp/types.h>

#include <trajopt_ifopt/core/dynamic_constraint_set.h>
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
    : constraints_("constraint-terms", false, false)
    , squared_costs_("squared-cost-terms", false, false)
    , hinge_costs_("hinge-cost-terms", false, false)
    , abs_costs_("abs-cost-terms", false, false)
    , dyn_constraints_("dyn-constraint-terms", false, true)
    , dyn_squared_costs_("dyn-squared-cost-terms", false, true)
    , dyn_hinge_costs_("dyn-hinge-cost-terms", false, true)
    , dyn_abs_costs_("dyn-abs-cost-terms", false, true)
  {
    variables_ = std::make_shared<trajopt_ifopt::Composite>("variable-sets", false, false);
  }

  bool initialized_{ false };
  trajopt_ifopt::Composite::Ptr variables_;

  ///////////////////////////////
  // These will never change size
  ///////////////////////////////
  Eigen::VectorXd constraint_merit_coeff_;
  trajopt_ifopt::Composite constraints_;
  std::vector<ConstraintType> constraint_types_;
  std::vector<std::string> constraint_names_;
  Eigen::Index num_constraint_qp_vars_{ 0 };
  Eigen::Index num_constraint_qp_cnts_{ 0 };

  trajopt_ifopt::Composite squared_costs_;
  trajopt_ifopt::Composite hinge_costs_;
  trajopt_ifopt::Composite abs_costs_;

  Eigen::VectorXd squared_costs_target_;
  std::vector<std::string> cost_names_;

  // Cached bounds (assumed static over SQP iterations)
  std::vector<trajopt_ifopt::Bounds> constraint_bounds_;

  std::vector<trajopt_ifopt::Bounds> squared_cost_bounds_;
  std::vector<trajopt_ifopt::Bounds> abs_cost_bounds_;
  std::vector<trajopt_ifopt::Bounds> hinge_cost_bounds_;

  std::vector<trajopt_ifopt::Bounds> var_bounds_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;

  ////////////////////////////////////////////////////////////////////
  // These objects can changes size if dynamic constraints are present
  ////////////////////////////////////////////////////////////////////

  trajopt_ifopt::Composite dyn_constraints_;
  std::vector<ConstraintType> dyn_constraint_types_;

  trajopt_ifopt::Composite dyn_squared_costs_;
  trajopt_ifopt::Composite dyn_hinge_costs_;
  trajopt_ifopt::Composite dyn_abs_costs_;

  std::vector<trajopt_ifopt::Bounds> dyn_constraint_bounds_;
  std::vector<trajopt_ifopt::Bounds> dyn_squared_cost_bounds_;
  std::vector<trajopt_ifopt::Bounds> dyn_abs_cost_bounds_;
  std::vector<trajopt_ifopt::Bounds> dyn_hinge_cost_bounds_;

  // These quantities are computed in the update() method
  Eigen::Index num_nlp_cnts_{ 0 };
  Eigen::Index num_qp_vars_{ 0 };
  Eigen::Index num_qp_cnts_{ 0 };

  // These objects are computed in the convexifyCosts() method
  trajopt_ifopt::Jacobian hessian_;
  Eigen::VectorXd gradient_;
  QuadExprs squared_objective_nlp_;

  // This object is computed in the linearizeConstraints() method
  trajopt_ifopt::Jacobian constraint_matrix_;

  // This object is computed in the updateConstraintsConstantExpression() method
  Eigen::VectorXd constraint_constant_;  // This should be the center of the bounds

  // These objects are computed in the updateNLPConstraintBounds(), updateNLPVariableBounds() and
  // updateSlackVariableBounds() methods
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;

  void addVariableSet(std::shared_ptr<trajopt_ifopt::VariableSet> variable_set);

  void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set);

  void addDynamicConstraintSet(std::shared_ptr<trajopt_ifopt::DynamicConstraintSet> dyn_constraint_set);

  void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set, CostPenaltyType penalty_type);

  void addDynamicCostSet(std::shared_ptr<trajopt_ifopt::DynamicConstraintSet> dyn_constraint_set,
                         CostPenaltyType penalty_type);

  void setup();

  void update();

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

void TrajOptQPProblem::Implementation::addVariableSet(std::shared_ptr<trajopt_ifopt::VariableSet> variable_set)
{
  variables_->AddComponent(std::move(variable_set));
  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set)
{
  constraint_set->LinkWithVariables(variables_);
  constraints_.AddComponent(std::move(constraint_set));
  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addDynamicConstraintSet(
    std::shared_ptr<trajopt_ifopt::DynamicConstraintSet> dyn_constraint_set)
{
  dyn_constraint_set->LinkWithVariables(variables_);
  dyn_constraints_.AddComponent(std::move(dyn_constraint_set));
  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                                                  CostPenaltyType penalty_type)
{
  constraint_set->LinkWithVariables(variables_);
  const std::vector<trajopt_ifopt::Bounds> cost_bounds = constraint_set->GetBounds();
  switch (penalty_type)
  {
    case CostPenaltyType::SQUARED:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt squared cost must have equality bounds!");
      }

      squared_costs_.AddComponent(std::move(constraint_set));
      break;
    }
    case CostPenaltyType::ABSOLUTE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt absolute cost must have equality bounds!");
      }

      abs_costs_.AddComponent(std::move(constraint_set));
      break;
    }
    case CostPenaltyType::HINGE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsInEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt hinge cost must have inequality bounds!");
      }

      hinge_costs_.AddComponent(std::move(constraint_set));
      break;
    }
    default:
    {
      throw std::runtime_error("Unsupport CostPenaltyType!");
    }
  }

  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addDynamicCostSet(
    std::shared_ptr<trajopt_ifopt::DynamicConstraintSet> dyn_constraint_set,
    CostPenaltyType penalty_type)
{
  dyn_constraint_set->LinkWithVariables(variables_);
  switch (penalty_type)
  {
    case CostPenaltyType::SQUARED:
    {
      dyn_squared_costs_.AddComponent(std::move(dyn_constraint_set));
      break;
    }
    case CostPenaltyType::ABSOLUTE:
    {
      dyn_abs_costs_.AddComponent(std::move(dyn_constraint_set));
      break;
    }
    case CostPenaltyType::HINGE:
    {
      dyn_hinge_costs_.AddComponent(std::move(dyn_constraint_set));
      break;
    }
    default:
    {
      throw std::runtime_error("Unsupport CostPenaltyType!");
    }
  }
}

void TrajOptQPProblem::Implementation::update()
{
  if (initialized_ && dyn_constraints_.Empty() && dyn_squared_costs_.Empty() && dyn_abs_costs_.Empty() &&
      dyn_hinge_costs_.Empty())
    return;

  // Local counts to avoid repeated virtual / composite queries
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_squared = squared_costs_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();

  num_nlp_cnts_ = constraints_.GetRows();

  // Hinge cost adds a variable and an inequality constraint (→ 2 constraints)
  // Absolute cost adds two variables and an equality constraint (→ 3 constraints)
  num_qp_vars_ = num_constraint_qp_vars_ + n_nlp_vars + n_hinge + (2L * n_abs);
  num_qp_cnts_ = num_nlp_cnts_ + num_constraint_qp_cnts_ + n_nlp_vars + (2L * n_hinge) + (3L * n_abs);

  if (!dyn_squared_costs_.Empty())
  {
    int num_dyn_squared = dyn_squared_costs_.Update();
    Eigen::VectorXd squared_costs_target = Eigen::VectorXd::Zero(n_squared + num_dyn_squared);
    if (n_squared > 0)
      squared_costs_target.head(n_squared) = squared_costs_target_.head(n_squared);

    dyn_squared_cost_bounds_ = dyn_constraints_.GetBounds();
    for (std::size_t j = 0; j < dyn_squared_cost_bounds_.size(); ++j)
    {
      assert(trajopt_ifopt::isBoundsEquality(dyn_squared_cost_bounds_[j]));
      squared_costs_target(n_squared + static_cast<Eigen::Index>(j)) = dyn_squared_cost_bounds_[j].lower;
    }
    squared_costs_target_ = squared_costs_target;
  }

  int n_dyn_abs{ 0 };
  if (!dyn_abs_costs_.Empty())
  {
    n_dyn_abs = dyn_abs_costs_.Update();
    dyn_abs_cost_bounds_ = dyn_abs_costs_.GetBounds();

    num_qp_vars_ += (2L * n_dyn_abs);
    num_qp_cnts_ += (3L * n_dyn_abs);
  }

  int n_dyn_hinge{ 0 };
  if (!dyn_hinge_costs_.Empty())
  {
    int n_dyn_hinge = dyn_hinge_costs_.Update();
    dyn_hinge_cost_bounds_ = dyn_hinge_costs_.GetBounds();

    num_qp_vars_ += n_dyn_hinge;
    num_qp_cnts_ += (2L * n_dyn_hinge);
  }

  if (!dyn_constraints_.Empty())
  {
    int num_nlp_dyn_cnts = dyn_constraints_.Update();
    num_nlp_cnts_ += num_nlp_dyn_cnts;
    dyn_constraint_bounds_ = dyn_constraints_.GetBounds();
    dyn_constraint_types_.resize(static_cast<std::size_t>(num_nlp_dyn_cnts));
    assert(dyn_constraint_bounds_.size() == static_cast<std::size_t>(num_nlp_dyn_cnts));

    for (std::size_t i = 0; i < num_nlp_dyn_cnts; ++i)
    {
      const auto& b = dyn_constraint_bounds_[i];
      const double diff = b.upper - b.lower;
      if (std::abs(diff) < 1e-3)
      {
        dyn_constraint_types_[i] = ConstraintType::EQ;
        num_qp_vars_ += 2;  // L1 slack pair
        num_qp_cnts_ += 2;
      }
      else
      {
        dyn_constraint_types_[i] = ConstraintType::INEQ;
        num_qp_vars_ += 1;  // hinge slack
        num_qp_cnts_ += 1;
      }
    }
  }

  constraint_constant_ = Eigen::VectorXd::Zero(num_nlp_cnts_ + n_hinge + n_abs + n_dyn_hinge + n_dyn_abs);

  // Initialize the constraint bounds
  bounds_lower_ = Eigen::VectorXd::Constant(num_qp_cnts_, -double(INFINITY));
  bounds_upper_ = Eigen::VectorXd::Constant(num_qp_cnts_, double(INFINITY));
}

void TrajOptQPProblem::Implementation::setup()
{
  // Local counts to avoid repeated virtual / composite queries
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_nlp_costs = squared_costs_.GetRows() + hinge_costs_.GetRows() + abs_costs_.GetRows();
  const Eigen::Index n_nlp_cnts = constraints_.GetRows();

  squared_costs_target_ = Eigen::VectorXd::Zero(squared_costs_.GetRows());

  box_size_ = Eigen::VectorXd::Constant(n_nlp_vars, 1e-1);

  constraint_merit_coeff_ =
      Eigen::VectorXd::Constant(n_nlp_cnts + static_cast<Eigen::Index>(dyn_constraints_.GetComponents().size()), 10.0);
  constraint_types_.resize(static_cast<std::size_t>(n_nlp_cnts));
  constraint_names_.resize(static_cast<std::size_t>(n_nlp_cnts) + dyn_constraints_.GetComponents().size());

  // Get NLP Cost and Constraint Names for Debug Print
  const auto& cnt_components = constraints_.GetComponents();
  for (std::size_t i = 0; i < cnt_components.size(); ++i)
  {
    const auto& cnt = cnt_components[i];
    for (Eigen::Index j = 0; j < cnt->GetRows(); ++j)
      constraint_names_[i] = cnt->GetName() + "_" + std::to_string(j);
  }

  const auto& dyn_cnt_components = dyn_constraints_.GetComponents();
  for (std::size_t i = 0; i < dyn_cnt_components.size(); ++i)
    constraint_names_[static_cast<std::size_t>(n_nlp_cnts) + i] = dyn_cnt_components[i]->GetName();

  // Reset and reserve name buffers (avoid accumulation across multiple setup() calls)
  cost_names_.clear();
  cost_names_.reserve(static_cast<std::size_t>(n_nlp_costs));

  // Get NLP Cost and Constraint Names for Debug Print
  for (const auto& cost : squared_costs_.GetComponents())
  {
    const std::vector<trajopt_ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); ++j)
    {
      assert(trajopt_ifopt::isBoundsEquality(cost_bounds[static_cast<std::size_t>(j)]));
      squared_costs_target_(j) = cost_bounds[static_cast<std::size_t>(j)].lower;
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  for (const auto& cost : dyn_squared_costs_.GetComponents())
    cost_names_.push_back(cost->GetName());

  for (const auto& cost : abs_costs_.GetComponents())
  {
    const std::vector<trajopt_ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); ++j)
    {
      assert(trajopt_ifopt::isBoundsEquality(cost_bounds[static_cast<std::size_t>(j)]));
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  for (const auto& cost : dyn_abs_costs_.GetComponents())
    cost_names_.push_back(cost->GetName());

  for (const auto& cost : hinge_costs_.GetComponents())
  {
    const std::vector<trajopt_ifopt::Bounds> cost_bounds = cost->GetBounds();
    for (Eigen::Index j = 0; j < cost->GetRows(); ++j)
    {
      assert(trajopt_ifopt::isBoundsInEquality(cost_bounds[static_cast<std::size_t>(j)]));
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
    }
  }

  for (const auto& cost : dyn_hinge_costs_.GetComponents())
    cost_names_.push_back(cost->GetName());

  // Get NLP bounds and detect constraint type
  constraint_bounds_ = constraints_.GetBounds();
  num_constraint_qp_vars_ = 0;
  num_constraint_qp_cnts_ = n_nlp_cnts;

  constraint_types_.resize(static_cast<std::size_t>(n_nlp_cnts));
  for (std::size_t i = 0; i < n_nlp_cnts; ++i)
  {
    const auto& b = constraint_bounds_[i];
    const double diff = b.upper - b.lower;
    if (std::abs(diff) < 1e-3)
    {
      constraint_types_[i] = ConstraintType::EQ;
      num_constraint_qp_vars_ += 2;  // L1 slack pair
      num_constraint_qp_cnts_ += 2;
    }
    else
    {
      constraint_types_[i] = ConstraintType::INEQ;
      num_constraint_qp_vars_ += 1;  // hinge slack
      num_constraint_qp_cnts_ += 1;
    }
  }

  // Cache flattened bounds for costs (used in exact cost evaluation)
  squared_cost_bounds_ = squared_costs_.GetBounds();
  abs_cost_bounds_ = abs_costs_.GetBounds();
  hinge_cost_bounds_ = hinge_costs_.GetBounds();

  // Cache variable bounds (used in updateNLPVariableBounds)
  var_bounds_ = variables_->GetBounds();

  // Update Constraints
  update();

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

  // Update if dynamic constraints are present
  update();

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
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_sq = squared_costs_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const auto n_dyn_sq = static_cast<Eigen::Index>(dyn_squared_costs_.GetComponents().size());
  const auto n_dyn_hinge = static_cast<Eigen::Index>(dyn_hinge_costs_.GetComponents().size());
  const auto n_dyn_abs = static_cast<Eigen::Index>(dyn_abs_costs_.GetComponents().size());
  const Eigen::Index total_cost = n_sq + n_hinge + n_abs + n_dyn_sq + n_dyn_hinge + n_dyn_abs;

  if (total_cost == 0)
    return {};

  Eigen::VectorXd costs = Eigen::VectorXd::Zero(total_cost);
  Eigen::VectorXd var_block = var_vals.head(n_nlp_vars);

  // Squared costs (already convexified into squared_objective_nlp_)
  if (n_sq > 0)
  {
    /** @todo How do I update this with dyn */
    costs.head(n_sq) = squared_objective_nlp_.values(var_block);
    assert(!(costs.head(n_sq).array() < -1e-8).any());
  }

  Eigen::Index cost_offset{ n_sq };
  Eigen::Index row_offset{ 0 };

  // Hinge costs
  if (n_hinge > 0)
  {
    const Eigen::VectorXd hinge_cnt_constant = constraint_constant_.topRows(n_hinge);
    const auto hinge_cnt_jac = constraint_matrix_.block(0, 0, n_hinge, n_nlp_vars);

    const Eigen::VectorXd hinge_convex_value = hinge_cnt_constant + hinge_cnt_jac * var_block;
    const Eigen::VectorXd hinge_cost = trajopt_ifopt::calcBoundsViolations(hinge_convex_value, hinge_cost_bounds_);

    costs.segment(cost_offset, n_hinge) = hinge_cost;
    assert(!(hinge_cost.array() < -1e-8).any());

    cost_offset += n_hinge;
    row_offset += n_hinge;
  }

  if (n_dyn_hinge > 0)
  {
    for (const auto& cost : dyn_hinge_costs_.GetComponents())
    {
      const Eigen::VectorXd hinge_cnt_constant = constraint_constant_.segment(row_offset, cost->GetRows());
      const auto hinge_cnt_jac = constraint_matrix_.block(row_offset, 0, cost->GetRows(), n_nlp_vars);

      const Eigen::VectorXd hinge_convex_value = hinge_cnt_constant + hinge_cnt_jac * var_block;
      const Eigen::VectorXd hinge_cost = trajopt_ifopt::calcBoundsViolations(hinge_convex_value, cost->GetBounds());

      costs(cost_offset++) = hinge_cost.sum();
      assert(!(hinge_cost.array() < -1e-8).any());
      row_offset += cost->GetRows();
    }
  }

  // Absolute costs
  if (n_abs > 0)
  {
    const Eigen::VectorXd abs_cnt_constant = constraint_constant_.segment(row_offset, n_abs);
    const auto abs_cnt_jac = constraint_matrix_.block(row_offset, 0, n_abs, n_nlp_vars);
    const Eigen::VectorXd abs_convex_value = abs_cnt_constant + abs_cnt_jac * var_block;

    // calcBoundsViolations already returns |violation|
    const Eigen::VectorXd abs_cost = trajopt_ifopt::calcBoundsViolations(abs_convex_value, abs_cost_bounds_);

    costs.segment(cost_offset, n_abs) = abs_cost;
    assert(!(abs_cost.array() < -1e-8).any());

    cost_offset += n_abs;
    row_offset += n_abs;
  }

  if (n_dyn_abs > 0)
  {
    for (const auto& cost : dyn_abs_costs_.GetComponents())
    {
      const Eigen::VectorXd abs_cnt_constant = constraint_constant_.segment(row_offset, cost->GetRows());
      const auto abs_cnt_jac = constraint_matrix_.block(row_offset, 0, cost->GetRows(), n_nlp_vars);
      const Eigen::VectorXd abs_convex_value = abs_cnt_constant + abs_cnt_jac * var_block;

      // calcBoundsViolations already returns |violation|
      const Eigen::VectorXd abs_cost = trajopt_ifopt::calcBoundsViolations(abs_convex_value, cost->GetBounds());

      costs(cost_offset++) = abs_cost.sum();
      assert(!(abs_cost.array() < -1e-8).any());

      row_offset += cost->GetRows();
    }
  }

  return costs;
}

double TrajOptQPProblem::Implementation::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  const Eigen::Index n_sq = squared_costs_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const auto n_dyn_sq = static_cast<Eigen::Index>(dyn_squared_costs_.GetComponents().size());
  const auto n_dyn_hinge = static_cast<Eigen::Index>(dyn_hinge_costs_.GetComponents().size());
  const auto n_dyn_abs = static_cast<Eigen::Index>(dyn_abs_costs_.GetComponents().size());
  const Eigen::Index n_total = n_sq + n_hinge + n_abs + n_dyn_sq + n_dyn_hinge + n_dyn_abs;

  if (n_total == 0)
    return {};

  double g{ 0 };
  setVariables(var_vals.data());

  if (n_sq > 0)
  {
    const Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(squared_costs_.GetValues(), squared_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.squaredNorm();
  }

  if (n_dyn_sq > 0)
  {
    const Eigen::VectorXd error =
        trajopt_ifopt::calcBoundsViolations(dyn_squared_costs_.GetValues(), dyn_squared_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.squaredNorm();
  }

  if (n_abs > 0)
  {
    const Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(abs_costs_.GetValues(), abs_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.sum();
  }

  if (n_dyn_abs > 0)
  {
    const Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(dyn_abs_costs_.GetValues(), dyn_abs_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.sum();
  }

  if (n_hinge > 0)
  {
    const Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(hinge_costs_.GetValues(), hinge_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.sum();
  }

  if (n_dyn_hinge > 0)
  {
    const Eigen::VectorXd error =
        trajopt_ifopt::calcBoundsViolations(dyn_hinge_costs_.GetValues(), dyn_hinge_cost_bounds_);
    assert(!(error.array() < 0.0).any());
    g += error.sum();
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::Implementation::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  const Eigen::Index n_sq = squared_costs_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const auto n_dyn_sq = static_cast<Eigen::Index>(dyn_squared_costs_.GetComponents().size());
  const auto n_dyn_hinge = static_cast<Eigen::Index>(dyn_hinge_costs_.GetComponents().size());
  const auto n_dyn_abs = static_cast<Eigen::Index>(dyn_abs_costs_.GetComponents().size());
  const Eigen::Index n_total = n_sq + n_hinge + n_abs + n_dyn_sq + n_dyn_hinge + n_dyn_abs;

  if (n_total == 0)
    return {};

  setVariables(var_vals.data());

  Eigen::VectorXd g(n_total);
  Eigen::Index idx = 0;

  if (n_sq > 0)
  {
    const Eigen::VectorXd err = trajopt_ifopt::calcBoundsViolations(squared_costs_.GetValues(), squared_cost_bounds_);
    g.segment(idx, n_sq) = err.array().square().matrix();
    idx += n_sq;
  }

  if (n_dyn_sq > 0)
  {
    const Eigen::VectorXd err =
        trajopt_ifopt::calcBoundsViolations(dyn_squared_costs_.GetValues(), dyn_squared_cost_bounds_);
    g.segment(idx, n_dyn_sq) = err.array().square().matrix();
    idx += n_dyn_sq;
  }

  if (n_abs > 0)
  {
    const Eigen::VectorXd err = trajopt_ifopt::calcBoundsViolations(abs_costs_.GetValues(), abs_cost_bounds_);
    g.segment(idx, n_abs) = err;
    idx += n_abs;
  }

  if (n_dyn_abs > 0)
  {
    const Eigen::VectorXd err = trajopt_ifopt::calcBoundsViolations(dyn_abs_costs_.GetValues(), dyn_abs_cost_bounds_);
    g.segment(idx, n_dyn_abs) = err;
    idx += n_dyn_abs;
  }

  if (n_hinge > 0)
  {
    const Eigen::VectorXd err = trajopt_ifopt::calcBoundsViolations(hinge_costs_.GetValues(), hinge_cost_bounds_);
    g.segment(idx, n_hinge) = err;
    idx += n_hinge;
  }

  if (n_dyn_hinge > 0)
  {
    const Eigen::VectorXd err =
        trajopt_ifopt::calcBoundsViolations(dyn_hinge_costs_.GetValues(), dyn_hinge_cost_bounds_);
    g.segment(idx, n_dyn_hinge) = err;
    // idx += n_dyn_hinge;
  }

  return g;
}

Eigen::VectorXd
TrajOptQPProblem::Implementation::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_cnt = constraints_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::Index n_dyn_abs = dyn_abs_costs_.GetRows();

  Eigen::Index row_index = n_hinge + n_abs + n_dyn_hinge + n_dyn_abs;
  const auto& dyn_components = dyn_constraints_.GetComponents();

  Eigen::VectorXd violations(n_cnt + static_cast<Eigen::Index>(dyn_components.size()));
  {
    // NOLINTNEXTLINE
    Eigen::VectorXd result_lin = constraint_matrix_.block(row_index, 0, n_cnt, n_nlp_vars) * var_vals.head(n_nlp_vars);
    const Eigen::VectorXd constraint_value = constraint_constant_.middleRows(row_index, n_cnt) + result_lin;

    violations.head(n_cnt) = trajopt_ifopt::calcBoundsViolations(constraint_value, constraint_bounds_);
    row_index += n_cnt;
  }

  for (std::size_t i = 0; i < dyn_components.size(); ++i)
  {
    const auto& cnt = dyn_components[i];

    // NOLINTNEXTLINE
    Eigen::VectorXd result_lin =
        constraint_matrix_.block(row_index, 0, cnt->GetRows(), n_nlp_vars) * var_vals.head(n_nlp_vars);
    const Eigen::VectorXd constraint_value = constraint_constant_.middleRows(row_index, cnt->GetRows()) + result_lin;

    violations(n_cnt + static_cast<Eigen::Index>(i)) =
        trajopt_ifopt::calcBoundsViolations(constraint_value, cnt->GetBounds()).sum();
    row_index += cnt->GetRows();
  }

  return violations;
}

Eigen::VectorXd
TrajOptQPProblem::Implementation::evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  setVariables(var_vals.data());

  const auto& dyn_components = dyn_constraints_.GetComponents();

  Eigen::VectorXd violations(constraints_.GetRows() + static_cast<Eigen::Index>(dyn_components.size()));
  {
    const Eigen::VectorXd cnt_vals = constraints_.GetValues();
    violations.head(constraints_.GetRows()) = trajopt_ifopt::calcBoundsViolations(cnt_vals, constraint_bounds_);
  }

  for (std::size_t i = 0; i < dyn_components.size(); ++i)
  {
    const auto& cnt = dyn_components[i];
    const Eigen::VectorXd cnt_vals = cnt->GetValues();
    violations(constraints_.GetRows() + static_cast<Eigen::Index>(i)) =
        trajopt_ifopt::calcBoundsViolations(cnt_vals, cnt->GetBounds()).sum();
  }

  return violations;
}

void TrajOptQPProblem::Implementation::scaleBoxSize(double& scale)
{
  box_size_ = box_size_ * scale;
  updateNLPVariableBounds();
}

void TrajOptQPProblem::Implementation::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size)
{
  assert(box_size.size() == variables_->GetRows());
  box_size_ = box_size;
  updateNLPVariableBounds();
}

void TrajOptQPProblem::Implementation::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  assert(merit_coeff.size() ==
         static_cast<std::size_t>(constraints_.GetRows()) + dyn_constraints_.GetComponents().size());
  constraint_merit_coeff_ = merit_coeff;
}

void TrajOptQPProblem::Implementation::print() const
{
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << '\n';
  std::cout << "Num NLP Vars: " << n_nlp_vars << '\n';
  std::cout << "Num QP Vars: " << num_qp_vars_ << '\n';
  std::cout << "Num NLP Constraints: " << num_qp_cnts_ << '\n';
  std::cout << "Detected Constraint Type: ";
  for (const auto& cnt_type : constraint_types_)
    std::cout << static_cast<int>(cnt_type) << ", ";

  for (const auto& cnt_type : dyn_constraint_types_)
    std::cout << static_cast<int>(cnt_type) << ", ";

  std::cout << '\n';
  std::cout << "Box Size: " << box_size_.transpose().format(format) << '\n';  // NOLINT
  std::cout << "Constraint Merit Coeff: " << constraint_merit_coeff_.transpose().format(format) << '\n';

  std::cout << "Hessian:\n" << hessian_.toDense().format(format) << '\n';
  std::cout << "Gradient: " << gradient_.transpose().format(format) << '\n';
  std::cout << "Constraint Matrix:\n" << constraint_matrix_.toDense().format(format) << '\n';
  std::cout << "Constraint Lower Bounds: "
            << bounds_lower_.head(num_nlp_cnts_ + n_hinge + n_dyn_hinge).transpose().format(format) << '\n';
  std::cout << "Constraint Upper Bounds: "
            << bounds_upper_.head(num_nlp_cnts_ + n_hinge + n_dyn_hinge).transpose().format(format) << '\n';
  std::cout << "Variable Lower Bounds: "
            << bounds_lower_.tail(bounds_lower_.rows() - num_nlp_cnts_ - n_hinge - n_dyn_hinge)
                   .transpose()
                   .format(format)
            << '\n';
  std::cout << "Variable Upper Bounds: "
            << bounds_upper_.tail(bounds_upper_.rows() - num_nlp_cnts_ - n_hinge - n_dyn_hinge)
                   .transpose()
                   .format(format)
            << '\n';
  std::cout << "All Lower Bounds: " << bounds_lower_.transpose().format(format) << '\n';
  std::cout << "All Upper Bounds: " << bounds_upper_.transpose().format(format) << '\n';
  std::cout << "NLP values: " << '\n';
  for (const auto& v_set : variables_->GetComponents())
    std::cout << v_set->GetValues().transpose().format(format) << '\n';
}

void TrajOptQPProblem::Implementation::convexifyCosts()
{
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_sq = squared_costs_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_dyn_sq = dyn_squared_costs_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::Index n_dyn_abs = dyn_abs_costs_.GetRows();
  const Eigen::Index num_nlp_costs = n_sq + n_hinge + n_abs + n_dyn_sq + n_dyn_hinge + n_dyn_abs;

  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  hessian_.resize(num_qp_vars_, num_qp_vars_);

  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  gradient_ = Eigen::VectorXd::Zero(num_qp_vars_);

  const Eigen::VectorXd x_initial = variables_->GetValues();

  // Create triplet list of nonzero gradients
  std::vector<Eigen::Triplet<double>> grad_triplet_list;
  grad_triplet_list.reserve(static_cast<std::size_t>(n_nlp_vars * num_nlp_costs) * 3);

  // Process Squared Costs
  /** @note See CostFromFunc::convex in modeling_utils.cpp. */
  if (n_sq > 0)
  {
    squared_objective_nlp_ = QuadExprs(n_sq, n_nlp_vars);
    const trajopt_ifopt::Jacobian cnt_jac = squared_costs_.GetJacobian();
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
    squared_objective_nlp_.constants.topRows(n_sq) = cost_quad_expr.constants;

    // store individual equations linear coefficients in a Triplet list to update equation linear coefficients later
    if (cost_quad_expr.linear_coeffs.nonZeros() > 0)
    {
      // Add jacobian to triplet list
      for (int k = 0; k < cost_quad_expr.linear_coeffs.outerSize(); ++k)
      {
        for (trajopt_ifopt::Jacobian::InnerIterator it(cost_quad_expr.linear_coeffs, k); it; ++it)
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
    gradient_.head(n_nlp_vars) = squared_objective_nlp_.objective_linear_coeffs;

    // Insert QP Problem Objective Quadratic Coefficients
    if (squared_objective_nlp_.objective_quadratic_coeffs.nonZeros() > 0)
    {
      hessian_.reserve(squared_objective_nlp_.objective_quadratic_coeffs.nonZeros());
      for (int k = 0; k < squared_objective_nlp_.objective_quadratic_coeffs.outerSize(); ++k)
      {
        for (trajopt_ifopt::Jacobian::InnerIterator it(squared_objective_nlp_.objective_quadratic_coeffs, k); it; ++it)
          hessian_.coeffRef(it.row(), it.col()) += it.value();
      }
    }
  }

  // Hinge and Asolute costs are handled differently than squared cost because they add constraints to the qp problem

  Eigen::Index current_var_index = n_nlp_vars;
  ////////////////////////////////////////////////////////
  // Set the gradient of the hinge cost variables
  ////////////////////////////////////////////////////////

  if (n_hinge > 0)
  {
    /** @todo This should be multiplied by the weight */
    gradient_.segment(current_var_index, n_hinge).setConstant(1.0);
    current_var_index += n_hinge;
  }

  if (n_dyn_hinge > 0)
  {
    gradient_.segment(current_var_index, n_dyn_hinge).setConstant(1.0);
    current_var_index += n_dyn_hinge;
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the absolute cost variables
  ////////////////////////////////////////////////////////
  if (n_abs > 0)
  {
    /** @todo This should be multiplied by the weight */
    gradient_.segment(current_var_index, 2L * n_abs).setConstant(1.0);
    current_var_index += (2L * n_abs);
  }

  if (n_dyn_abs > 0)
  {
    /** @todo This should be multiplied by the weight */
    gradient_.segment(current_var_index, 2L * n_dyn_abs).setConstant(1.0);
    current_var_index += (2L * n_dyn_abs);
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the constraint slack variables
  ////////////////////////////////////////////////////////

  for (Eigen::Index i = 0; i < constraint_types_.size(); i++)
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

  auto cnt_offset = static_cast<Eigen::Index>(constraint_types_.size());
  for (Eigen::Index i = 0; i < dyn_constraint_types_.size(); i++)
  {
    if (dyn_constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      gradient_[current_var_index++] = constraint_merit_coeff_[cnt_offset + i];
      gradient_[current_var_index++] = constraint_merit_coeff_[cnt_offset + i];
    }
    else
    {
      gradient_[current_var_index++] = constraint_merit_coeff_[cnt_offset + i];
    }
  }
}

void TrajOptQPProblem::Implementation::linearizeConstraints()
{
  const trajopt_ifopt::Jacobian nlp_cnt_jac = constraints_.GetJacobian();
  const trajopt_ifopt::Jacobian hinge_cnt_jac = hinge_costs_.GetJacobian();
  const trajopt_ifopt::Jacobian abs_cnt_jac = abs_costs_.GetJacobian();
  const trajopt_ifopt::Jacobian dyn_nlp_cnt_jac = dyn_constraints_.GetJacobian();
  const trajopt_ifopt::Jacobian dyn_hinge_cnt_jac = dyn_hinge_costs_.GetJacobian();
  const trajopt_ifopt::Jacobian dyn_abs_cnt_jac = dyn_abs_costs_.GetJacobian();

  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_cnt = constraints_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::Index n_dyn_abs = dyn_abs_costs_.GetRows();

  std::vector<Eigen::Triplet<double>> triplets;
  const Eigen::Index nnz_base = nlp_cnt_jac.nonZeros() + hinge_cnt_jac.nonZeros() + abs_cnt_jac.nonZeros() +
                                dyn_nlp_cnt_jac.nonZeros() + dyn_hinge_cnt_jac.nonZeros() + dyn_abs_cnt_jac.nonZeros();
  // Rough but closer than *3
  triplets.reserve(static_cast<std::size_t>(nnz_base + num_qp_vars_ +  // diag
                                            n_hinge + n_dyn_hinge + (2L * (n_abs + n_dyn_abs)) + (2L * num_nlp_cnts_)));

  Eigen::Index current_row_index = 0;

  // hinge constraints
  for (int k = 0; k < hinge_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(hinge_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // dyn hinge constraints (shifted rows)
  current_row_index += n_hinge;
  for (int k = 0; k < dyn_hinge_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(dyn_hinge_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // abs constraints (shifted again)
  current_row_index += n_dyn_hinge;
  for (int k = 0; k < abs_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(abs_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // dyn abs constraints (shifted again)
  current_row_index += n_abs;
  for (int k = 0; k < dyn_abs_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(dyn_abs_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // nlp constraints (shift again)
  current_row_index += n_dyn_abs;
  for (int k = 0; k < nlp_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(nlp_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // nlp dynamic constraints (shift again)
  current_row_index += n_cnt;
  for (int k = 0; k < dyn_nlp_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(dyn_nlp_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // hinge slack vars
  Eigen::Index current_col_index = n_nlp_vars;
  for (Eigen::Index i = 0; i < n_hinge; ++i)
    triplets.emplace_back(i, current_col_index++, -1.0);

  current_row_index = n_hinge;
  for (Eigen::Index i = 0; i < n_dyn_hinge; ++i)
    triplets.emplace_back(i, current_col_index++, -1.0);

  // abs slack vars
  current_row_index += n_dyn_hinge;
  for (Eigen::Index i = 0; i < n_abs; ++i)
  {
    triplets.emplace_back(current_row_index + i, current_col_index++, 1.0);
    triplets.emplace_back(current_row_index + i, current_col_index++, -1.0);
  }

  current_row_index += n_abs;
  for (Eigen::Index i = 0; i < n_dyn_abs; ++i)
  {
    triplets.emplace_back(current_row_index + i, current_col_index++, 1.0);
    triplets.emplace_back(current_row_index + i, current_col_index++, -1.0);
  }

  // constraint slack vars
  current_row_index += n_dyn_abs;
  for (const auto& constraint_type : constraint_types_)
  {
    if (constraint_type == ConstraintType::EQ)
    {
      triplets.emplace_back(current_row_index, current_col_index++, 1.0);
      triplets.emplace_back(current_row_index, current_col_index++, -1.0);
    }
    else
    {
      triplets.emplace_back(current_row_index, current_col_index++, -1.0);
    }
    current_row_index++;
  }

  for (const auto& constraint_type : dyn_constraint_types_)
  {
    if (constraint_type == ConstraintType::EQ)
    {
      triplets.emplace_back(current_row_index, current_col_index++, 1.0);
      triplets.emplace_back(current_row_index, current_col_index++, -1.0);
    }
    else
    {
      triplets.emplace_back(current_row_index, current_col_index++, -1.0);
    }
    current_row_index++;
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  current_row_index = num_nlp_cnts_ + hinge_cnt_jac.rows() + abs_cnt_jac.rows();
  for (Eigen::Index i = 0; i < num_qp_vars_; ++i)
    triplets.emplace_back(current_row_index + i, i, 1.0);

  // Insert the triplet list into the sparse matrix
  constraint_matrix_.resize(num_qp_cnts_, num_qp_vars_);
  constraint_matrix_.reserve(nnz_base + num_qp_vars_);
  constraint_matrix_.setFromTriplets(triplets.begin(), triplets.end());  // NOLINT
}

void TrajOptQPProblem::Implementation::updateConstraintsConstantExpression()
{
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_cnt = constraints_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_dyn_cnt = dyn_constraints_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::Index n_dyn_abs = dyn_abs_costs_.GetRows();
  const Eigen::Index total_num_cnt = num_nlp_cnts_ + n_hinge + n_abs + n_dyn_hinge + n_dyn_abs;

  if (total_num_cnt == 0)
    return;

  // Get values about which we will linearize
  const Eigen::VectorXd x_initial = variables_->GetValues().head(n_nlp_vars);

  // One mat-vec for all constraint rows (excluding slack columns)
  const Eigen::VectorXd lin = constraint_matrix_.block(0, 0, total_num_cnt, n_nlp_vars) * x_initial;

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

  Eigen::Index row = 0;
  if (n_hinge > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = hinge_costs_.GetValues();

    // The block excludes the slack variables
    constraint_constant_.segment(row, n_hinge) = cnt_initial_value - lin.segment(row, n_hinge);
    row += n_hinge;
  }

  if (n_dyn_hinge > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = dyn_hinge_costs_.GetValues();

    // The block excludes the slack variables
    constraint_constant_.segment(row, n_dyn_hinge) = cnt_initial_value - lin.segment(row, n_dyn_hinge);
    row += n_dyn_hinge;
  }

  if (n_abs > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = abs_costs_.GetValues();

    // The block excludes the slack variables
    constraint_constant_.segment(row, n_abs) = cnt_initial_value - lin.segment(row, n_abs);
    row += n_abs;
  }

  if (n_dyn_abs > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = abs_costs_.GetValues();

    // The block excludes the slack variables
    constraint_constant_.segment(row, n_dyn_abs) = cnt_initial_value - lin.segment(row, n_dyn_abs);
    row += n_dyn_abs;
  }

  if (n_cnt > 0)
  {
    const Eigen::VectorXd cnt_initial_value = constraints_.GetValues();

    // The block excludes the slack variables
    constraint_constant_.segment(row, n_cnt) = cnt_initial_value - lin.segment(row, n_cnt);
    row += n_cnt;
  }

  if (n_dyn_cnt > 0)
  {
    const Eigen::VectorXd cnt_initial_value = dyn_constraints_.GetValues();

    // The block excludes the slack variables
    constraint_constant_.segment(row, n_dyn_cnt) = cnt_initial_value - lin.segment(row, n_dyn_cnt);
    // row += n_dyn_cnt;
  }
}

void TrajOptQPProblem::Implementation::updateNLPConstraintBounds()
{
  const Eigen::Index n_cnt = constraints_.GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_dyn_cnt = dyn_constraints_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::Index n_dyn_abs = dyn_abs_costs_.GetRows();

  const Eigen::Index total_num_cnt = num_nlp_cnts_ + n_hinge + n_abs + n_dyn_hinge + n_dyn_abs;

  if (total_num_cnt == 0)
    return;

  Eigen::VectorXd cnt_bound_lower(total_num_cnt);
  Eigen::VectorXd cnt_bound_upper(total_num_cnt);
  Eigen::Index row = 0;

  // Hinge constraint bounds
  for (Eigen::Index i = 0; i < n_hinge; ++i)
  {
    const auto& b = hinge_cost_bounds_[static_cast<std::size_t>(i)];
    cnt_bound_lower[row + i] = b.lower;
    cnt_bound_upper[row + i] = b.upper;
  }
  row += n_hinge;

  for (Eigen::Index i = 0; i < n_dyn_hinge; ++i)
  {
    const auto& b = dyn_hinge_cost_bounds_[static_cast<std::size_t>(i)];
    cnt_bound_lower[row + i] = b.lower;
    cnt_bound_upper[row + i] = b.upper;
  }
  row += n_dyn_hinge;

  // Absolute constraint bounds
  for (Eigen::Index i = 0; i < n_abs; ++i)
  {
    const auto& b = abs_cost_bounds_[static_cast<std::size_t>(i)];
    cnt_bound_lower[row + i] = b.lower;
    cnt_bound_upper[row + i] = b.upper;
  }
  row += n_abs;

  for (Eigen::Index i = 0; i < n_dyn_abs; ++i)
  {
    const auto& b = dyn_abs_cost_bounds_[static_cast<std::size_t>(i)];
    cnt_bound_lower[row + i] = b.lower;
    cnt_bound_upper[row + i] = b.upper;
  }
  row += n_dyn_abs;

  // NLP constraint bounds
  for (Eigen::Index j = 0; j < n_cnt; ++j)
  {
    const auto& b = constraint_bounds_[static_cast<std::size_t>(j)];
    cnt_bound_lower[row + j] = b.lower;
    cnt_bound_upper[row + j] = b.upper;
  }
  row += n_cnt;

  for (Eigen::Index j = 0; j < n_dyn_cnt; ++j)
  {
    const auto& b = dyn_constraint_bounds_[static_cast<std::size_t>(j)];
    cnt_bound_lower[row + j] = b.lower;
    cnt_bound_upper[row + j] = b.upper;
    row += n_dyn_cnt;
  }

  const Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - constraint_constant_;
  const Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - constraint_constant_;

  bounds_lower_.topRows(total_num_cnt) = linearized_cnt_lower;
  bounds_upper_.topRows(total_num_cnt) = linearized_cnt_upper;
}

void TrajOptQPProblem::Implementation::updateNLPVariableBounds()
{
  // Equivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::Index n_dyn_abs = dyn_abs_costs_.GetRows();

  const Eigen::VectorXd x_initial = variables_->GetValues();

  // Set the variable limits once
  Eigen::VectorXd var_bounds_lower(n_nlp_vars);
  Eigen::VectorXd var_bounds_upper(n_nlp_vars);

  for (Eigen::Index i = 0; i < n_nlp_vars; ++i)
  {
    const auto& b = var_bounds_[static_cast<std::size_t>(i)];
    var_bounds_lower[i] = b.lower;
    var_bounds_upper[i] = b.upper;
  }

  // Calculate box constraints, while limiting to variable bounds and maintaining the trust region size
  const Eigen::VectorXd var_bounds_lower_final =
      (x_initial.cwiseMin(var_bounds_upper - box_size_) - box_size_).cwiseMax(var_bounds_lower);
  const Eigen::VectorXd var_bounds_upper_final =
      (x_initial.cwiseMax(var_bounds_lower + box_size_) + box_size_).cwiseMin(var_bounds_upper);

  const Eigen::Index var_row_index = num_nlp_cnts_ + n_hinge + n_abs + n_dyn_hinge + n_dyn_abs;

  bounds_lower_.segment(var_row_index, n_nlp_vars) = var_bounds_lower_final;
  bounds_upper_.segment(var_row_index, n_nlp_vars) = var_bounds_upper_final;
}

void TrajOptQPProblem::Implementation::updateSlackVariableBounds()
{
  const Eigen::Index n_nlp_vars = variables_->GetRows();
  const Eigen::Index n_hinge = hinge_costs_.GetRows();
  const Eigen::Index n_abs = abs_costs_.GetRows();
  const Eigen::Index n_dyn_hinge = dyn_hinge_costs_.GetRows();
  const Eigen::Index n_dyn_abs = dyn_abs_costs_.GetRows();

  Eigen::Index current_cnt_index = n_nlp_vars + num_nlp_cnts_ + n_hinge + n_abs + n_dyn_hinge + n_dyn_abs;

  for (Eigen::Index i = 0; i < n_hinge; i++)
  {
    bounds_lower_[current_cnt_index] = 0;
    bounds_upper_[current_cnt_index++] = double(INFINITY);
  }

  for (Eigen::Index i = 0; i < n_dyn_hinge; i++)
  {
    bounds_lower_[current_cnt_index] = 0;
    bounds_upper_[current_cnt_index++] = double(INFINITY);
  }

  for (Eigen::Index i = 0; i < n_abs; i++)
  {
    bounds_lower_[current_cnt_index] = 0;
    bounds_upper_[current_cnt_index++] = double(INFINITY);
    bounds_lower_[current_cnt_index] = 0;
    bounds_upper_[current_cnt_index++] = double(INFINITY);
  }

  for (const auto& constraint_type : constraint_types_)
  {
    if (constraint_type == ConstraintType::EQ)
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

  for (const auto& constraint_type : dyn_constraint_types_)
  {
    if (constraint_type == ConstraintType::EQ)
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

void TrajOptQPProblem::addVariableSet(std::shared_ptr<trajopt_ifopt::VariableSet> variable_set)
{
  impl_->addVariableSet(std::move(variable_set));
}

void TrajOptQPProblem::addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set)
{
  impl_->addConstraintSet(std::move(constraint_set));
}

void TrajOptQPProblem::addDynamicConstraintSet(std::shared_ptr<trajopt_ifopt::DynamicConstraintSet> dyn_constraint_set)
{
  impl_->addDynamicConstraintSet(dyn_constraint_set);
}

void TrajOptQPProblem::addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                                  CostPenaltyType penalty_type)
{
  impl_->addCostSet(std::move(constraint_set), penalty_type);
}

void TrajOptQPProblem::addDynamicCostSet(std::shared_ptr<trajopt_ifopt::DynamicConstraintSet> dyn_constraint_set,
                                         CostPenaltyType penalty_type)
{
  impl_->addDynamicCostSet(std::move(dyn_constraint_set), penalty_type);
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

Eigen::Index TrajOptQPProblem::getNumNLPVars() const
{
  return std::as_const<Implementation>(*impl_).variables_->GetRows();
}

Eigen::Index TrajOptQPProblem::getNumNLPConstraints() const
{
  const auto& base = std::as_const<Implementation>(*impl_);
  return (base.constraints_.GetRows() + static_cast<Eigen::Index>(base.dyn_constraints_.GetComponents().size()));
}

Eigen::Index TrajOptQPProblem::getNumNLPCosts() const
{
  const auto& base = std::as_const<Implementation>(*impl_);
  Eigen::Index cnt =
      (base.squared_costs_.GetRows() + static_cast<Eigen::Index>(base.dyn_squared_costs_.GetComponents().size()));
  cnt += (base.hinge_costs_.GetRows() + static_cast<Eigen::Index>(base.dyn_hinge_costs_.GetComponents().size()));
  cnt += (base.abs_costs_.GetRows() + static_cast<Eigen::Index>(base.dyn_abs_costs_.GetComponents().size()));
  return cnt;
}

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

Eigen::Ref<const trajopt_ifopt::Jacobian> TrajOptQPProblem::getHessian() { return impl_->hessian_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getGradient() { return impl_->gradient_; }

Eigen::Ref<const trajopt_ifopt::Jacobian> TrajOptQPProblem::getConstraintMatrix() { return impl_->constraint_matrix_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsLower() { return impl_->bounds_lower_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsUpper() { return impl_->bounds_upper_; }

}  // namespace trajopt_sqp
