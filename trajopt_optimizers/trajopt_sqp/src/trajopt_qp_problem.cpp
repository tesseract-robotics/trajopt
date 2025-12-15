
#include <trajopt_ifopt/core/constraint_set.h>

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
struct ComponentInfo
{
  Eigen::Index rows{ 0 };
  Eigen::VectorXd coeffs;
  std::vector<trajopt_ifopt::Bounds> bounds;
  std::vector<ConstraintType> constraint_types;
};

struct ConvexProblem
{
  // Static quantities
  Eigen::Index n_nlp_vars{ 0 };

  Eigen::Index n_cnt{ 0 };
  Eigen::Index n_squared{ 0 };
  Eigen::Index n_hinge{ 0 };
  Eigen::Index n_abs{ 0 };

  std::vector<ComponentInfo> cnt_infos;
  std::vector<ComponentInfo> sq_infos;
  std::vector<ComponentInfo> hinge_infos;
  std::vector<ComponentInfo> abs_infos;

  // These quantities are computed in the update() method
  Eigen::Index num_qp_vars{ 0 };
  Eigen::Index num_qp_cnts{ 0 };

  Eigen::Index n_dyn_squared{ 0 };
  Eigen::Index n_dyn_cnt{ 0 };
  Eigen::Index n_dyn_hinge{ 0 };
  Eigen::Index n_dyn_abs{ 0 };

  std::vector<ComponentInfo> dyn_cnt_infos;
  std::vector<ComponentInfo> dyn_sq_infos;
  std::vector<ComponentInfo> dyn_hinge_infos;
  std::vector<ComponentInfo> dyn_abs_infos;

  // These objects are computed in the convexifyCosts() method
  trajopt_ifopt::Jacobian hessian;
  Eigen::VectorXd gradient;
  QuadExprs squared_objective_nlp;
  Eigen::VectorXd squared_costs_target;

  // This object is computed in the linearizeConstraints() method
  trajopt_ifopt::Jacobian constraint_matrix;

  // This object is computed in the updateConstraintsConstantExpression() method
  Eigen::VectorXd constraint_constant;  // This should be the center of the bounds

  // These objects are computed in the updateNLPConstraintBounds(), updateNLPVariableBounds() and
  // updateSlackVariableBounds() methods
  Eigen::VectorXd bounds_lower;
  Eigen::VectorXd bounds_upper;

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals);
};

Eigen::VectorXd ConvexProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  /** @note The legacy trajop appears to use all variables include slack for convex cost evaluation */
  const auto total_cost =
      static_cast<Eigen::Index>(sq_infos.size() + hinge_infos.size() + abs_infos.size() + dyn_sq_infos.size() +
                                dyn_hinge_infos.size() + dyn_abs_infos.size());

  if (total_cost == 0)
    return {};

  Eigen::VectorXd costs = Eigen::VectorXd::Zero(total_cost);
  Eigen::Ref<const Eigen::VectorXd> var_block = var_vals.head(n_nlp_vars);

  Eigen::Index cost_idx{ 0 };
  Eigen::Index row_offset{ 0 };

  // Squared costs (already convexified into squared_objective_nlp_)
  if (!sq_infos.empty())
  {
    Eigen::VectorXd sq_costs = squared_objective_nlp.values(var_block);
    assert(!(sq_costs.array() < -1e-8).any());
    for (const auto& c_info : sq_infos)
    {
      costs(cost_idx++) = sq_costs.segment(row_offset, c_info.rows).sum();
      row_offset += c_info.rows;
    }
  }

  /** @todo How do I update this with dyn */

  // Reset row offset for constraint matrix
  row_offset = 0;

  // Hinge costs
  if (!hinge_infos.empty())
  {
    for (const auto& c_info : hinge_infos)
    {
      const Eigen::VectorXd hinge_cnt_constant = constraint_constant.segment(row_offset, c_info.rows);
      const auto hinge_cnt_jac = constraint_matrix.middleRows(row_offset, c_info.rows);

      const Eigen::VectorXd hinge_convex_value = hinge_cnt_constant + hinge_cnt_jac * var_vals;
      const Eigen::VectorXd cost = trajopt_ifopt::calcBoundsViolations(hinge_convex_value, c_info.bounds);

      costs(cost_idx++) = cost.sum();
      assert(!(cost.array() < -1e-8).any());
      row_offset += c_info.rows;
    }
  }

  if (!dyn_hinge_infos.empty())
  {
    for (const auto& c_info : dyn_hinge_infos)
    {
      if (c_info.rows == 0)
      {
        costs(cost_idx++) = 0;
        continue;
      }

      const Eigen::VectorXd hinge_cnt_constant = constraint_constant.segment(row_offset, c_info.rows);
      const auto hinge_cnt_jac = constraint_matrix.middleRows(row_offset, c_info.rows);

      const Eigen::VectorXd hinge_convex_value = hinge_cnt_constant + hinge_cnt_jac * var_vals;
      const Eigen::VectorXd cost = trajopt_ifopt::calcBoundsViolations(hinge_convex_value, c_info.bounds);

      costs(cost_idx++) = cost.sum();
      assert(!(cost.array() < -1e-8).any());
      row_offset += c_info.rows;
    }
  }

  // Absolute costs
  if (!abs_infos.empty())
  {
    for (const auto& c_info : abs_infos)
    {
      const Eigen::VectorXd abs_cnt_constant = constraint_constant.segment(row_offset, c_info.rows);
      const auto abs_cnt_jac = constraint_matrix.middleRows(row_offset, c_info.rows);
      const Eigen::VectorXd abs_convex_value = abs_cnt_constant + abs_cnt_jac * var_vals;

      // calcBoundsViolations already returns |violation|
      const Eigen::VectorXd cost = trajopt_ifopt::calcBoundsViolations(abs_convex_value, c_info.bounds);

      costs(cost_idx++) = cost.sum();
      assert(!(cost.array() < -1e-8).any());
      row_offset += c_info.rows;
    }
  }

  if (!dyn_abs_infos.empty())
  {
    for (const auto& c_info : dyn_abs_infos)
    {
      if (c_info.rows == 0)
      {
        costs(cost_idx++) = 0;
        continue;
      }

      const Eigen::VectorXd abs_cnt_constant = constraint_constant.segment(row_offset, c_info.rows);
      const auto abs_cnt_jac = constraint_matrix.middleRows(row_offset, c_info.rows);
      const Eigen::VectorXd abs_convex_value = abs_cnt_constant + abs_cnt_jac * var_vals;

      // calcBoundsViolations already returns |violation|
      const Eigen::VectorXd cost = trajopt_ifopt::calcBoundsViolations(abs_convex_value, c_info.bounds);

      costs(cost_idx++) = cost.sum();
      assert(!(cost.array() < -1e-8).any());
      row_offset += c_info.rows;
    }
  }

  return costs;
}

Eigen::VectorXd ConvexProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  /** @note The legacy trajop does not use slack variables for convex evaluation */
  Eigen::Index cnt_idx{ 0 };
  Eigen::Index row_index = n_hinge + n_abs + n_dyn_hinge + n_dyn_abs;
  Eigen::VectorXd violations(cnt_infos.size() + dyn_cnt_infos.size());

  Eigen::Ref<const Eigen::VectorXd> var_block = var_vals.head(n_nlp_vars);
  for (const auto& c_info : cnt_infos)
  {
    // NOLINTNEXTLINE
    Eigen::VectorXd result_lin = constraint_matrix.block(row_index, 0, c_info.rows, n_nlp_vars) * var_block;
    const Eigen::VectorXd constraint_value = constraint_constant.middleRows(row_index, c_info.rows) + result_lin;

    violations(cnt_idx++) = trajopt_ifopt::calcBoundsViolations(constraint_value, c_info.bounds).sum();
    row_index += c_info.rows;
  }

  for (const auto& c_info : dyn_cnt_infos)
  {
    if (c_info.rows == 0)
    {
      violations(cnt_idx++) = 0;
      continue;
    }

    // NOLINTNEXTLINE
    Eigen::VectorXd result_lin = constraint_matrix.block(row_index, 0, c_info.rows, n_nlp_vars) * var_block;
    const Eigen::VectorXd constraint_value = constraint_constant.middleRows(row_index, c_info.rows) + result_lin;

    violations(cnt_idx++) = trajopt_ifopt::calcBoundsViolations(constraint_value, c_info.bounds).sum();
    row_index += c_info.rows;
  }

  return violations;
}

struct TrajOptQPProblem::Implementation
{
  Implementation()
    : variables_(std::make_shared<trajopt_ifopt::CompositeVariables>("variable-sets"))
    , constraints_("constraint-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, false)
    , squared_costs_("squared-cost-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, false)
    , hinge_costs_("hinge-cost-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, false)
    , abs_costs_("abs-cost-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, false)
    , dyn_constraints_("dyn-constraint-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, true)
    , dyn_squared_costs_("dyn-squared-cost-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, true)
    , dyn_hinge_costs_("dyn-hinge-cost-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, true)
    , dyn_abs_costs_("dyn-abs-cost-terms", trajopt_ifopt::Differentiable::Mode::kStackRows, true)
  {
  }

  bool initialized_{ false };

  ///////////////////////////////
  // These will never change size
  ///////////////////////////////
  Eigen::Index num_cost_components{ 0 };
  Eigen::Index num_cnt_components{ 0 };
  Eigen::Index num_constraint_qp_vars_{ 0 };
  Eigen::Index num_constraint_qp_cnts_{ 0 };

  trajopt_ifopt::CompositeVariables::Ptr variables_;
  trajopt_ifopt::CompositeDifferentiable constraints_;
  trajopt_ifopt::CompositeDifferentiable squared_costs_;
  trajopt_ifopt::CompositeDifferentiable hinge_costs_;
  trajopt_ifopt::CompositeDifferentiable abs_costs_;

  Eigen::VectorXd constraint_merit_coeff_;
  std::vector<double> squared_costs_target_;
  std::vector<std::string> constraint_names_;
  std::vector<std::string> cost_names_;

  std::vector<ComponentInfo> cnt_infos;
  std::vector<ComponentInfo> sq_infos;
  std::vector<ComponentInfo> hinge_infos;
  std::vector<ComponentInfo> abs_infos;

  std::vector<trajopt_ifopt::Bounds> var_bounds_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;

  ////////////////////////////////////////////////////////////////////
  // These objects can changes size if dynamic constraints are present
  ////////////////////////////////////////////////////////////////////

  trajopt_ifopt::CompositeDifferentiable dyn_constraints_;
  trajopt_ifopt::CompositeDifferentiable dyn_squared_costs_;
  trajopt_ifopt::CompositeDifferentiable dyn_hinge_costs_;
  trajopt_ifopt::CompositeDifferentiable dyn_abs_costs_;

  std::vector<ComponentInfo> dyn_cnt_infos;
  std::vector<ComponentInfo> dyn_sq_infos;
  std::vector<ComponentInfo> dyn_hinge_infos;
  std::vector<ComponentInfo> dyn_abs_infos;

  ConvexProblem cvp;

  void addVariableSet(std::shared_ptr<trajopt_ifopt::Variables> variable_set);

  void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set);

  void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set, CostPenaltyType penalty_type);

  void setup();

  void update();

  void setVariables(const double* x);

  Eigen::VectorXd getVariableValues() const;

  void convexify();

  double evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

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

void TrajOptQPProblem::Implementation::addVariableSet(std::shared_ptr<trajopt_ifopt::Variables> variable_set)
{
  variables_->addComponent(std::move(variable_set));
  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set)
{
  constraint_set->linkWithVariables(variables_);

  if (constraint_set->isDynamic())
    dyn_constraints_.addComponent(std::move(constraint_set));
  else
    constraints_.addComponent(std::move(constraint_set));

  initialized_ = false;
}

void TrajOptQPProblem::Implementation::addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                                                  CostPenaltyType penalty_type)
{
  constraint_set->linkWithVariables(variables_);
  const std::vector<trajopt_ifopt::Bounds> cost_bounds = constraint_set->getBounds();
  switch (penalty_type)
  {
    case CostPenaltyType::SQUARED:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt squared cost must have equality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_squared_costs_.addComponent(std::move(constraint_set));
      else
        squared_costs_.addComponent(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::ABSOLUTE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt absolute cost must have equality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_abs_costs_.addComponent(std::move(constraint_set));
      else
        abs_costs_.addComponent(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::HINGE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (!trajopt_ifopt::isBoundsInEquality(bound))
          throw std::runtime_error("TrajOpt Ifopt hinge cost must have inequality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_hinge_costs_.addComponent(std::move(constraint_set));
      else
        hinge_costs_.addComponent(std::move(constraint_set));

      break;
    }
    default:
    {
      throw std::runtime_error("Unsupport CostPenaltyType!");
    }
  }

  initialized_ = false;
}

void TrajOptQPProblem::Implementation::update()
{
  if (initialized_ && dyn_constraints_.empty() && dyn_squared_costs_.empty() && dyn_abs_costs_.empty() &&
      dyn_hinge_costs_.empty())
    return;

  // Local counts to avoid repeated virtual / composite queries
  cvp.n_dyn_cnt = dyn_constraints_.getRows();
  cvp.n_dyn_squared = dyn_squared_costs_.getRows();
  cvp.n_dyn_hinge = dyn_hinge_costs_.getRows();
  cvp.n_dyn_abs = dyn_abs_costs_.getRows();

  cvp.dyn_sq_infos = dyn_sq_infos;
  cvp.dyn_abs_infos = dyn_abs_infos;
  cvp.dyn_hinge_infos = dyn_hinge_infos;
  cvp.dyn_cnt_infos = dyn_cnt_infos;

  // Hinge cost adds a variable and an inequality constraint (→ 2 constraints)
  // Absolute cost adds two variables and an equality constraint (→ 3 constraints)
  cvp.num_qp_vars = num_constraint_qp_vars_ + cvp.n_nlp_vars + cvp.n_hinge + (2L * cvp.n_abs);
  cvp.num_qp_cnts = num_constraint_qp_cnts_ + cvp.n_dyn_cnt + cvp.n_nlp_vars + (2L * cvp.n_hinge) + (3L * cvp.n_abs);

  cvp.squared_costs_target = Eigen::VectorXd::Zero(cvp.n_squared + cvp.n_dyn_squared);
  if (cvp.n_squared > 0)
    cvp.squared_costs_target.head(cvp.n_squared) = Eigen::Map<Eigen::VectorXd>(
        squared_costs_target_.data(), static_cast<Eigen::Index>(squared_costs_target_.size()));

  if (!dyn_squared_costs_.empty())
  {
    Eigen::Index sq_idx{ cvp.n_squared };
    for (const auto& info : dyn_sq_infos)
    {
      for (const auto& b : info.bounds)
        cvp.squared_costs_target(sq_idx++) = b.lower;
    }
  }

  if (!dyn_abs_costs_.empty())
  {
    cvp.num_qp_vars += (2L * cvp.n_dyn_abs);
    cvp.num_qp_cnts += (3L * cvp.n_dyn_abs);
  }

  if (!dyn_hinge_costs_.empty())
  {
    cvp.num_qp_vars += cvp.n_dyn_hinge;
    cvp.num_qp_cnts += (2L * cvp.n_dyn_hinge);
  }

  if (!dyn_constraints_.empty())
  {
    for (auto& info : dyn_cnt_infos)
    {
      for (const auto& constraint_type : info.constraint_types)
      {
        if (constraint_type == ConstraintType::EQ)
        {
          cvp.num_qp_vars += 2;  // L1 slack pair
          cvp.num_qp_cnts += 2;
        }
        else
        {
          cvp.num_qp_vars += 1;  // hinge slack
          cvp.num_qp_cnts += 1;
        }
      }
    }
  }

  cvp.constraint_constant =
      Eigen::VectorXd::Zero(cvp.n_cnt + cvp.n_hinge + cvp.n_abs + cvp.n_dyn_cnt + cvp.n_dyn_hinge + cvp.n_dyn_abs);

  // Initialize the constraint bounds
  cvp.bounds_lower = Eigen::VectorXd::Constant(cvp.num_qp_cnts, -double(INFINITY));
  cvp.bounds_upper = Eigen::VectorXd::Constant(cvp.num_qp_cnts, double(INFINITY));
}

void TrajOptQPProblem::Implementation::setup()
{
  // Call update
  squared_costs_.update();
  hinge_costs_.update();
  abs_costs_.update();
  dyn_squared_costs_.update();
  dyn_hinge_costs_.update();
  dyn_abs_costs_.update();
  constraints_.update();
  dyn_constraints_.update();

  // Local counts to avoid repeated virtual / composite queries
  const Eigen::Index n_nlp_vars = variables_->getRows();

  box_size_ = Eigen::VectorXd::Constant(n_nlp_vars, 1e-1);

  // Reset and reserve name buffers (avoid accumulation across multiple setup() calls)
  num_cost_components =
      static_cast<Eigen::Index>(squared_costs_.getComponents().size() + hinge_costs_.getComponents().size() +
                                abs_costs_.getComponents().size() + dyn_squared_costs_.getComponents().size() +
                                dyn_hinge_costs_.getComponents().size() + dyn_abs_costs_.getComponents().size());
  cost_names_.clear();
  cost_names_.reserve(static_cast<std::size_t>(num_cost_components));

  // Get NLP Cost and Constraint Names for Debug Print
  sq_infos.clear();
  sq_infos.reserve(squared_costs_.getComponents().size());
  squared_costs_target_.reserve(static_cast<std::size_t>(squared_costs_.getRows()));
  for (const auto& cost : squared_costs_.getComponents())
  {
    cost_names_.push_back(cost->getName());

    ComponentInfo info;
    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

    for (const auto& cost_bound : info.bounds)
    {
      assert(trajopt_ifopt::isBoundsEquality(cost_bound));
      squared_costs_target_.push_back(cost_bound.lower);
    }

    sq_infos.push_back(info);
  }

  dyn_sq_infos.clear();
  dyn_sq_infos.reserve(dyn_squared_costs_.getComponents().size());
  for (const auto& cost : dyn_squared_costs_.getComponents())
  {
    cost_names_.push_back(cost->getName());

    ComponentInfo info;
    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();
#ifndef NDEBUG
    for (const auto& cost_bound : info.bounds)
      assert(trajopt_ifopt::isBoundsEquality(cost_bound));
#endif

    dyn_sq_infos.push_back(info);
  }

  abs_infos.clear();
  abs_infos.reserve(abs_costs_.getComponents().size());
  for (const auto& cost : abs_costs_.getComponents())
  {
    cost_names_.push_back(cost->getName());

    ComponentInfo info;
    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

#ifndef NDEBUG
    for (const auto& cost_bound : info.bounds)
      assert(trajopt_ifopt::isBoundsEquality(cost_bound));
#endif

    abs_infos.push_back(info);
  }

  dyn_abs_infos.clear();
  dyn_abs_infos.reserve(dyn_abs_costs_.getComponents().size());
  for (const auto& cost : dyn_abs_costs_.getComponents())
  {
    cost_names_.push_back(cost->getName());

    ComponentInfo info;
    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

#ifndef NDEBUG
    for (const auto& cost_bound : info.bounds)
      assert(trajopt_ifopt::isBoundsEquality(cost_bound));
#endif

    dyn_abs_infos.push_back(info);
  }

  hinge_infos.clear();
  hinge_infos.reserve(hinge_costs_.getComponents().size());
  for (const auto& cost : hinge_costs_.getComponents())
  {
    cost_names_.push_back(cost->getName());

    ComponentInfo info;
    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

#ifndef NDEBUG
    for (const auto& cost_bound : info.bounds)
      assert(trajopt_ifopt::isBoundsInEquality(cost_bound));
#endif

    hinge_infos.push_back(info);
  }

  dyn_hinge_infos.clear();
  dyn_hinge_infos.reserve(dyn_hinge_costs_.getComponents().size());
  for (const auto& cost : dyn_hinge_costs_.getComponents())
  {
    cost_names_.push_back(cost->getName());

    ComponentInfo info;
    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

#ifndef NDEBUG
    for (const auto& cost_bound : info.bounds)
      assert(trajopt_ifopt::isBoundsInEquality(cost_bound));
#endif

    dyn_hinge_infos.push_back(info);
  }

  // Get NLP bounds and detect constraint type
  const Eigen::Index n_nlp_cnts = constraints_.getRows();
  num_cnt_components =
      static_cast<Eigen::Index>(constraints_.getComponents().size() + dyn_constraints_.getComponents().size());
  constraint_merit_coeff_ = Eigen::VectorXd::Constant(num_cnt_components, 10.0);
  constraint_names_.clear();
  constraint_names_.reserve(static_cast<std::size_t>(num_cnt_components));

  num_constraint_qp_vars_ = 0;
  num_constraint_qp_cnts_ = n_nlp_cnts;

  cnt_infos.clear();
  cnt_infos.reserve(constraints_.getComponents().size());
  for (const auto& cnt : constraints_.getComponents())
  {
    constraint_names_.push_back(cnt->getName());

    ComponentInfo info;
    info.rows = cnt->getRows();
    info.coeffs = cnt->getCoefficients();
    info.bounds = cnt->getBounds();
    info.constraint_types.reserve(info.bounds.size());

    for (const auto& cnt_bound : info.bounds)
    {
      const double diff = cnt_bound.upper - cnt_bound.lower;
      if (std::abs(diff) < 1e-3)
      {
        info.constraint_types.push_back(ConstraintType::EQ);
        num_constraint_qp_vars_ += 2;  // L1 slack pair
        num_constraint_qp_cnts_ += 2;
      }
      else
      {
        info.constraint_types.push_back(ConstraintType::INEQ);
        num_constraint_qp_vars_ += 1;  // hinge slack
        num_constraint_qp_cnts_ += 1;
      }
    }

    cnt_infos.push_back(info);
  }

  dyn_cnt_infos.clear();
  dyn_cnt_infos.reserve(dyn_constraints_.getComponents().size());
  for (const auto& cnt : dyn_constraints_.getComponents())
  {
    constraint_names_.push_back(cnt->getName());

    ComponentInfo info;
    info.rows = cnt->getRows();
    info.coeffs = cnt->getCoefficients();
    info.bounds = cnt->getBounds();
    info.constraint_types.reserve(info.bounds.size());

    for (const auto& cnt_bound : info.bounds)
    {
      const double diff = cnt_bound.upper - cnt_bound.lower;
      if (std::abs(diff) < 1e-3)
        info.constraint_types.push_back(ConstraintType::EQ);
      else
        info.constraint_types.push_back(ConstraintType::INEQ);
    }

    dyn_cnt_infos.push_back(info);
  }

  // Cache variable bounds (used in updateNLPVariableBounds)
  var_bounds_ = variables_->getBounds();

  // Copy static components to the convex problem
  cvp.n_nlp_vars = variables_->getRows();
  cvp.n_squared = squared_costs_.getRows();
  cvp.n_abs = abs_costs_.getRows();
  cvp.n_hinge = hinge_costs_.getRows();
  cvp.n_cnt = constraints_.getRows();

  cvp.sq_infos = sq_infos;
  cvp.abs_infos = abs_infos;
  cvp.hinge_infos = hinge_infos;
  cvp.cnt_infos = cnt_infos;

  // Update
  update();

  initialized_ = true;
}

void TrajOptQPProblem::Implementation::setVariables(const double* x)
{
  variables_->setVariables(Eigen::Map<const Eigen::VectorXd>(x, variables_->getRows()));

  squared_costs_.update();
  hinge_costs_.update();
  abs_costs_.update();
  constraints_.update();
  dyn_squared_costs_.update();
  dyn_hinge_costs_.update();
  dyn_abs_costs_.update();
  dyn_constraints_.update();

  if (initialized_)
  {
    if (!dyn_constraints_.empty())
    {
      dyn_cnt_infos.clear();
      for (const auto& component : dyn_constraints_.getComponents())
      {
        ComponentInfo info;
        info.rows = component->getRows();
        info.coeffs = component->getCoefficients();
        info.bounds = component->getBounds();
        info.constraint_types.reserve(info.bounds.size());

        for (const auto& cnt_bound : info.bounds)
        {
          const double diff = cnt_bound.upper - cnt_bound.lower;
          if (std::abs(diff) < 1e-3)
            info.constraint_types.push_back(ConstraintType::EQ);
          else
            info.constraint_types.push_back(ConstraintType::INEQ);
        }

        dyn_cnt_infos.push_back(info);
      }
    }

    if (!dyn_squared_costs_.empty())
    {
      dyn_sq_infos.clear();
      for (const auto& component : dyn_squared_costs_.getComponents())
      {
        ComponentInfo info;
        info.rows = component->getRows();
        info.coeffs = component->getCoefficients();
        info.bounds = component->getBounds();
        dyn_sq_infos.push_back(info);
      }
    }

    if (!dyn_hinge_costs_.empty())
    {
      dyn_hinge_infos.clear();
      for (const auto& component : dyn_hinge_costs_.getComponents())
      {
        ComponentInfo info;
        info.rows = component->getRows();
        info.coeffs = component->getCoefficients();
        info.bounds = component->getBounds();
        dyn_hinge_infos.push_back(info);
      }
    }

    if (!dyn_abs_costs_.empty())
    {
      dyn_abs_infos.clear();
      for (const auto& component : dyn_abs_costs_.getComponents())
      {
        ComponentInfo info;
        info.rows = component->getRows();
        info.coeffs = component->getCoefficients();
        info.bounds = component->getBounds();
        dyn_abs_infos.push_back(info);
      }
    }
  }
}

Eigen::VectorXd TrajOptQPProblem::Implementation::getVariableValues() const { return variables_->getValues(); }

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

double TrajOptQPProblem::Implementation::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return evaluateExactCosts(var_vals).sum();
}

Eigen::VectorXd
TrajOptQPProblem::Implementation::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& /*var_vals*/)
{
  assert(initialized_);

  if (num_cost_components == 0)
    return {};

  Eigen::VectorXd g(num_cost_components);
  Eigen::Index cost_idx{ 0 };

  if (!sq_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = squared_costs_.getValues();
    for (const auto& info : sq_infos)
    {
      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      g(cost_idx++) = (err.array().square() * info.coeffs.array()).sum();
      row_offset += info.rows;
    }
  }

  if (!dyn_sq_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = dyn_squared_costs_.getValues();
    for (const auto& info : sq_infos)
    {
      if (info.rows == 0)
      {
        g(cost_idx++) = 0;
        continue;
      }

      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      g(cost_idx++) = (err.array().square() * info.coeffs.array()).sum();
      row_offset += info.rows;
    }
  }

  if (!abs_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = abs_costs_.getValues();
    for (const auto& info : abs_infos)
    {
      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      g(cost_idx++) = err.sum();
      row_offset += info.rows;
    }
  }

  if (!dyn_abs_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = dyn_abs_costs_.getValues();
    for (const auto& info : dyn_abs_infos)
    {
      if (info.rows == 0)
      {
        g(cost_idx++) = 0;
        continue;
      }

      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      g(cost_idx++) = err.sum();
      row_offset += info.rows;
    }
  }

  if (!hinge_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = hinge_costs_.getValues();
    for (const auto& info : hinge_infos)
    {
      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      g(cost_idx++) = err.sum();
      row_offset += info.rows;
    }
  }

  if (!dyn_hinge_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = dyn_hinge_costs_.getValues();
    for (const auto& info : dyn_hinge_infos)
    {
      if (info.rows == 0)
      {
        g(cost_idx++) = 0;
        continue;
      }

      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      g(cost_idx++) = err.sum();
      row_offset += info.rows;
    }
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::Implementation::evaluateExactConstraintViolations(
    const Eigen::Ref<const Eigen::VectorXd>& /*var_vals*/)
{
  Eigen::VectorXd violations(num_cnt_components);
  Eigen::Index cnt_idx{ 0 };

  if (!cnt_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = constraints_.getValues();
    for (const auto& info : cnt_infos)
    {
      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      violations(cnt_idx++) = err.sum();
      row_offset += info.rows;
    }
  }

  if (!dyn_cnt_infos.empty())
  {
    Eigen::Index row_offset{ 0 };
    Eigen::VectorXd values = dyn_constraints_.getValues();
    for (const auto& info : dyn_cnt_infos)
    {
      if (info.rows == 0)
      {
        violations(cnt_idx++) = 0;
        continue;
      }

      const Eigen::VectorXd err =
          trajopt_ifopt::calcBoundsViolations(values.segment(row_offset, info.rows), info.bounds);
      violations(cnt_idx++) = err.sum();
      row_offset += info.rows;
    }
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
  assert(box_size.size() == variables_->getRows());
  box_size_ = box_size;
  updateNLPVariableBounds();
}

void TrajOptQPProblem::Implementation::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  assert(merit_coeff.size() == constraints_.getComponents().size() + dyn_constraints_.getComponents().size());
  constraint_merit_coeff_ = merit_coeff;
}

void TrajOptQPProblem::Implementation::print() const
{
  Eigen::Index total_cnt = cvp.n_cnt + cvp.n_dyn_cnt + cvp.n_hinge + cvp.n_dyn_hinge;
  const Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << '\n';
  std::cout << "Num NLP Vars: " << cvp.n_nlp_vars << '\n';
  std::cout << "Num QP Vars: " << cvp.num_qp_vars << '\n';
  std::cout << "Num NLP Constraints: " << cvp.num_qp_cnts << '\n';
  std::cout << "Detected Constraint Type: ";
  for (const auto& info : cvp.cnt_infos)
    for (const auto& cnt_type : info.constraint_types)
      std::cout << static_cast<int>(cnt_type) << ", ";

  for (const auto& info : cvp.dyn_cnt_infos)
    for (const auto& cnt_type : info.constraint_types)
      std::cout << static_cast<int>(cnt_type) << ", ";

  std::cout << '\n';
  std::cout << "Box Size: " << box_size_.transpose().format(format) << '\n';  // NOLINT
  std::cout << "Constraint Merit Coeff: " << constraint_merit_coeff_.transpose().format(format) << '\n';

  std::cout << "Hessian:\n" << cvp.hessian.toDense().format(format) << '\n';
  std::cout << "Gradient: " << cvp.gradient.transpose().format(format) << '\n';
  std::cout << "Constraint Matrix:\n" << cvp.constraint_matrix.toDense().format(format) << '\n';
  std::cout << "Constraint Lower Bounds: " << cvp.bounds_lower.head(total_cnt).transpose().format(format) << '\n';
  std::cout << "Constraint Upper Bounds: " << cvp.bounds_upper.head(total_cnt).transpose().format(format) << '\n';
  std::cout << "Variable Lower Bounds: "
            << cvp.bounds_lower.tail(cvp.bounds_lower.rows() - total_cnt).transpose().format(format) << '\n';
  std::cout << "Variable Upper Bounds: "
            << cvp.bounds_upper.tail(cvp.bounds_upper.rows() - total_cnt).transpose().format(format) << '\n';
  std::cout << "All Lower Bounds: " << cvp.bounds_lower.transpose().format(format) << '\n';
  std::cout << "All Upper Bounds: " << cvp.bounds_upper.transpose().format(format) << '\n';
  std::cout << "NLP values: " << '\n';
  for (const auto& v_set : variables_->getComponents())
    std::cout << v_set->getValues().transpose().format(format) << '\n';
}

void TrajOptQPProblem::Implementation::convexifyCosts()
{
  const Eigen::Index num_nlp_costs =
      cvp.n_squared + cvp.n_hinge + cvp.n_abs + cvp.n_dyn_squared + cvp.n_dyn_hinge + cvp.n_dyn_abs;

  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  cvp.hessian.resize(cvp.num_qp_vars, cvp.num_qp_vars);

  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  cvp.gradient = Eigen::VectorXd::Zero(cvp.num_qp_vars);

  const Eigen::VectorXd x_initial = variables_->getValues();

  // Create triplet list of nonzero gradients
  std::vector<Eigen::Triplet<double>> grad_triplet_list;
  grad_triplet_list.reserve(static_cast<std::size_t>(cvp.n_nlp_vars * num_nlp_costs) * 3);

  // Process Squared Costs
  /** @note See CostFromFunc::convex in modeling_utils.cpp. */
  if (cvp.n_squared > 0)
  {
    cvp.squared_objective_nlp = QuadExprs(cvp.n_squared, cvp.n_nlp_vars);
    const trajopt_ifopt::Jacobian cnt_jac = squared_costs_.getJacobian();
    const Eigen::VectorXd cnt_vals = squared_costs_.getValues();
    const Eigen::VectorXd cnt_weights = squared_costs_.getCoefficients();

    // This is not correct should pass the value to createAffExprs then use bound to which could change the sign of the
    // affine expression
    //    Eigen::VectorXd cnt_error = trajopt_ifopt::calcBoundsErrors(cnt_vals, squared_costs_.getBounds());

    // This should be correct now
    AffExprs cnt_aff_expr = createAffExprs(cnt_vals, cnt_jac, x_initial);
    cnt_aff_expr.constants = (cvp.squared_costs_target - cnt_aff_expr.constants);
    cnt_aff_expr.linear_coeffs *= -1;
    QuadExprs cost_quad_expr = squareAffExprs(cnt_aff_expr, cnt_weights);

    // Sum objective function linear coefficients
    cvp.squared_objective_nlp.objective_linear_coeffs += cost_quad_expr.objective_linear_coeffs;

    // Sum objective function quadratic coefficients
    cvp.squared_objective_nlp.objective_quadratic_coeffs += cost_quad_expr.objective_quadratic_coeffs;

    // store individual equations constant
    cvp.squared_objective_nlp.constants.topRows(cvp.n_squared) = cost_quad_expr.constants;

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
    cvp.squared_objective_nlp.quadratic_coeffs.reserve(cvp.squared_objective_nlp.quadratic_coeffs.size() +
                                                       cost_quad_expr.quadratic_coeffs.size());
    cvp.squared_objective_nlp.quadratic_coeffs.insert(cvp.squared_objective_nlp.quadratic_coeffs.end(),
                                                      cost_quad_expr.quadratic_coeffs.begin(),
                                                      cost_quad_expr.quadratic_coeffs.end());

    // Store individual equations linear coefficients
    cvp.squared_objective_nlp.linear_coeffs.setFromTriplets(grad_triplet_list.begin(),
                                                            grad_triplet_list.end());  // NOLINT

    // Insert QP Problem Objective Linear Coefficients
    cvp.gradient.head(cvp.n_nlp_vars) = cvp.squared_objective_nlp.objective_linear_coeffs;

    // Insert QP Problem Objective Quadratic Coefficients
    if (cvp.squared_objective_nlp.objective_quadratic_coeffs.nonZeros() > 0)
    {
      cvp.hessian.reserve(cvp.squared_objective_nlp.objective_quadratic_coeffs.nonZeros());
      for (int k = 0; k < cvp.squared_objective_nlp.objective_quadratic_coeffs.outerSize(); ++k)
      {
        for (trajopt_ifopt::Jacobian::InnerIterator it(cvp.squared_objective_nlp.objective_quadratic_coeffs, k); it;
             ++it)
          cvp.hessian.coeffRef(it.row(), it.col()) += it.value();
      }
    }
  }

  // Hinge and Asolute costs are handled differently than squared cost because they add constraints to the qp problem

  Eigen::Index current_var_index = cvp.n_nlp_vars;
  ////////////////////////////////////////////////////////
  // Set the gradient of the hinge cost variables
  ////////////////////////////////////////////////////////

  if (cvp.n_hinge > 0)
  {
    for (const auto& info : cvp.hinge_infos)
    {
      cvp.gradient.segment(current_var_index, info.rows) = info.coeffs;
      current_var_index += info.rows;
    }
  }

  if (cvp.n_dyn_hinge > 0)
  {
    for (const auto& info : cvp.dyn_hinge_infos)
    {
      cvp.gradient.segment(current_var_index, info.rows) = info.coeffs;
      current_var_index += info.rows;
    }
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the absolute cost variables
  ////////////////////////////////////////////////////////
  if (cvp.n_abs > 0)
  {
    for (const auto& info : cvp.abs_infos)
    {
      cvp.gradient.segment(current_var_index, info.rows) = info.coeffs;
      current_var_index += info.rows;
      cvp.gradient.segment(current_var_index, info.rows) = info.coeffs;
      current_var_index += info.rows;
    }
  }

  if (cvp.n_dyn_abs > 0)
  {
    for (const auto& info : cvp.dyn_abs_infos)
    {
      cvp.gradient.segment(current_var_index, info.rows) = info.coeffs;
      current_var_index += info.rows;
      cvp.gradient.segment(current_var_index, info.rows) = info.coeffs;
      current_var_index += info.rows;
    }
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the constraint slack variables
  ////////////////////////////////////////////////////////

  for (Eigen::Index i = 0; i < cvp.cnt_infos.size(); i++)
  {
    const auto& cnt_info = cvp.cnt_infos[static_cast<std::size_t>(i)];
    const double merit_coeff = constraint_merit_coeff_[i];
    for (Eigen::Index j = 0; j < cnt_info.constraint_types.size(); j++)
    {
      const auto constraint_type = cnt_info.constraint_types[static_cast<std::size_t>(j)];
      const double coeff = cnt_info.coeffs(j);
      if (constraint_type == ConstraintType::EQ)
      {
        cvp.gradient[current_var_index++] = coeff * merit_coeff;
        cvp.gradient[current_var_index++] = coeff * merit_coeff;
      }
      else
      {
        cvp.gradient[current_var_index++] = coeff * merit_coeff;
      }
    }
  }

  auto cnt_offset = static_cast<Eigen::Index>(cvp.cnt_infos.size());
  for (Eigen::Index i = 0; i < cvp.dyn_cnt_infos.size(); i++)
  {
    const auto& cnt_info = cvp.dyn_cnt_infos[static_cast<std::size_t>(i)];
    const double merit_coeff = constraint_merit_coeff_[cnt_offset + i];
    for (Eigen::Index j = 0; j < cnt_info.constraint_types.size(); j++)
    {
      const auto constraint_type = cnt_info.constraint_types[static_cast<std::size_t>(j)];
      const double coeff = cnt_info.coeffs(j);
      if (constraint_type == ConstraintType::EQ)
      {
        cvp.gradient[current_var_index++] = coeff * merit_coeff;
        cvp.gradient[current_var_index++] = coeff * merit_coeff;
      }
      else
      {
        cvp.gradient[current_var_index++] = coeff * merit_coeff;
      }
    }
  }
}

void TrajOptQPProblem::Implementation::linearizeConstraints()
{
  const trajopt_ifopt::Jacobian nlp_cnt_jac = constraints_.getJacobian();
  const trajopt_ifopt::Jacobian hinge_cnt_jac = hinge_costs_.getJacobian();
  const trajopt_ifopt::Jacobian abs_cnt_jac = abs_costs_.getJacobian();
  const trajopt_ifopt::Jacobian dyn_nlp_cnt_jac = dyn_constraints_.getJacobian();
  const trajopt_ifopt::Jacobian dyn_hinge_cnt_jac = dyn_hinge_costs_.getJacobian();
  const trajopt_ifopt::Jacobian dyn_abs_cnt_jac = dyn_abs_costs_.getJacobian();

  std::vector<Eigen::Triplet<double>> triplets;
  const Eigen::Index nnz_base = nlp_cnt_jac.nonZeros() + hinge_cnt_jac.nonZeros() + abs_cnt_jac.nonZeros() +
                                dyn_nlp_cnt_jac.nonZeros() + dyn_hinge_cnt_jac.nonZeros() + dyn_abs_cnt_jac.nonZeros();
  // Rough but closer than *3
  triplets.reserve(static_cast<std::size_t>(nnz_base + cvp.num_qp_vars + cvp.n_hinge + cvp.n_dyn_hinge +
                                            (2L * (cvp.n_abs + cvp.n_dyn_abs)) + (2L * (cvp.n_cnt + cvp.n_dyn_cnt))));

  Eigen::Index current_row_index = 0;

  // hinge constraints
  for (int k = 0; k < hinge_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(hinge_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // dyn hinge constraints (shifted rows)
  current_row_index += cvp.n_hinge;
  for (int k = 0; k < dyn_hinge_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(dyn_hinge_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // abs constraints (shifted again)
  current_row_index += cvp.n_dyn_hinge;
  for (int k = 0; k < abs_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(abs_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // dyn abs constraints (shifted again)
  current_row_index += cvp.n_abs;
  for (int k = 0; k < dyn_abs_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(dyn_abs_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // nlp constraints (shift again)
  current_row_index += cvp.n_dyn_abs;
  for (int k = 0; k < nlp_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(nlp_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // nlp dynamic constraints (shift again)
  current_row_index += cvp.n_cnt;
  for (int k = 0; k < dyn_nlp_cnt_jac.outerSize(); ++k)
    for (trajopt_ifopt::Jacobian::InnerIterator it(dyn_nlp_cnt_jac, k); it; ++it)
      triplets.emplace_back(current_row_index + it.row(), it.col(), it.value());

  // hinge slack vars
  Eigen::Index current_col_index = cvp.n_nlp_vars;
  for (Eigen::Index i = 0; i < cvp.n_hinge; ++i)
    triplets.emplace_back(i, current_col_index++, -1.0);

  current_row_index = cvp.n_hinge;
  for (Eigen::Index i = 0; i < cvp.n_dyn_hinge; ++i)
    triplets.emplace_back(i, current_col_index++, -1.0);

  // abs slack vars
  current_row_index += cvp.n_dyn_hinge;
  for (Eigen::Index i = 0; i < cvp.n_abs; ++i)
  {
    triplets.emplace_back(current_row_index + i, current_col_index++, 1.0);
    triplets.emplace_back(current_row_index + i, current_col_index++, -1.0);
  }

  current_row_index += cvp.n_abs;
  for (Eigen::Index i = 0; i < cvp.n_dyn_abs; ++i)
  {
    triplets.emplace_back(current_row_index + i, current_col_index++, 1.0);
    triplets.emplace_back(current_row_index + i, current_col_index++, -1.0);
  }

  // constraint slack vars
  current_row_index += cvp.n_dyn_abs;
  for (const auto& info : cvp.cnt_infos)
  {
    for (const auto& constraint_type : info.constraint_types)
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
  }

  for (const auto& info : cvp.dyn_cnt_infos)
  {
    for (const auto& constraint_type : info.constraint_types)
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
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  current_row_index = cvp.n_cnt + cvp.n_dyn_cnt + cvp.n_hinge + cvp.n_dyn_hinge + cvp.n_abs + cvp.n_dyn_abs;
  for (Eigen::Index i = 0; i < cvp.num_qp_vars; ++i)
    triplets.emplace_back(current_row_index + i, i, 1.0);

  // Insert the triplet list into the sparse matrix
  cvp.constraint_matrix.resize(cvp.num_qp_cnts, cvp.num_qp_vars);
  cvp.constraint_matrix.reserve(nnz_base + cvp.num_qp_vars);
  cvp.constraint_matrix.setFromTriplets(triplets.begin(), triplets.end());  // NOLINT
}

void TrajOptQPProblem::Implementation::updateConstraintsConstantExpression()
{
  const Eigen::Index total_num_cnt =
      cvp.n_cnt + cvp.n_hinge + cvp.n_abs + cvp.n_dyn_cnt + cvp.n_dyn_hinge + cvp.n_dyn_abs;

  if (total_num_cnt == 0)
    return;

  // Get values about which we will linearize
  const Eigen::VectorXd x_initial = variables_->getValues().head(cvp.n_nlp_vars);

  // One mat-vec for all constraint rows (excluding slack columns)
  const Eigen::VectorXd lin = cvp.constraint_matrix.block(0, 0, total_num_cnt, cvp.n_nlp_vars) * x_initial;

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
  if (cvp.n_hinge > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = hinge_costs_.getValues();

    // The block excludes the slack variables
    cvp.constraint_constant.segment(row, cvp.n_hinge) = cnt_initial_value - lin.segment(row, cvp.n_hinge);
    row += cvp.n_hinge;
  }

  if (cvp.n_dyn_hinge > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = dyn_hinge_costs_.getValues();

    // The block excludes the slack variables
    cvp.constraint_constant.segment(row, cvp.n_dyn_hinge) = cnt_initial_value - lin.segment(row, cvp.n_dyn_hinge);
    row += cvp.n_dyn_hinge;
  }

  if (cvp.n_abs > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = abs_costs_.getValues();

    // The block excludes the slack variables
    cvp.constraint_constant.segment(row, cvp.n_abs) = cnt_initial_value - lin.segment(row, cvp.n_abs);
    row += cvp.n_abs;
  }

  if (cvp.n_dyn_abs > 0)
  {  // Get values about which we will linearize
    const Eigen::VectorXd cnt_initial_value = dyn_abs_costs_.getValues();

    // The block excludes the slack variables
    cvp.constraint_constant.segment(row, cvp.n_dyn_abs) = cnt_initial_value - lin.segment(row, cvp.n_dyn_abs);
    row += cvp.n_dyn_abs;
  }

  if (cvp.n_cnt > 0)
  {
    const Eigen::VectorXd cnt_initial_value = constraints_.getValues();

    // The block excludes the slack variables
    cvp.constraint_constant.segment(row, cvp.n_cnt) = cnt_initial_value - lin.segment(row, cvp.n_cnt);
    row += cvp.n_cnt;
  }

  if (cvp.n_dyn_cnt > 0)
  {
    const Eigen::VectorXd cnt_initial_value = dyn_constraints_.getValues();

    // The block excludes the slack variables
    cvp.constraint_constant.segment(row, cvp.n_dyn_cnt) = cnt_initial_value - lin.segment(row, cvp.n_dyn_cnt);
    // row += n_dyn_cnt;
  }
}

void TrajOptQPProblem::Implementation::updateNLPConstraintBounds()
{
  const Eigen::Index total_num_cnt =
      cvp.n_cnt + cvp.n_hinge + cvp.n_abs + cvp.n_dyn_cnt + cvp.n_dyn_hinge + cvp.n_dyn_abs;

  if (total_num_cnt == 0)
    return;

  Eigen::VectorXd cnt_bound_lower(total_num_cnt);
  Eigen::VectorXd cnt_bound_upper(total_num_cnt);
  Eigen::Index row = 0;

  // Hinge constraint bounds
  for (const auto& info : hinge_infos)
  {
    for (const auto& b : info.bounds)
    {
      cnt_bound_lower[row] = b.lower;
      cnt_bound_upper[row++] = b.upper;
    }
  }

  for (const auto& info : dyn_hinge_infos)
  {
    for (const auto& b : info.bounds)
    {
      cnt_bound_lower[row] = b.lower;
      cnt_bound_upper[row++] = b.upper;
    }
  }

  // Absolute constraint bounds
  for (const auto& info : abs_infos)
  {
    for (const auto& b : info.bounds)
    {
      cnt_bound_lower[row] = b.lower;
      cnt_bound_upper[row++] = b.upper;
    }
  }

  for (const auto& info : dyn_abs_infos)
  {
    for (const auto& b : info.bounds)
    {
      cnt_bound_lower[row] = b.lower;
      cnt_bound_upper[row++] = b.upper;
    }
  }

  // NLP constraint bounds
  for (const auto& info : cnt_infos)
  {
    for (const auto& b : info.bounds)
    {
      cnt_bound_lower[row] = b.lower;
      cnt_bound_upper[row++] = b.upper;
    }
  }

  for (const auto& info : dyn_cnt_infos)
  {
    for (const auto& b : info.bounds)
    {
      cnt_bound_lower[row] = b.lower;
      cnt_bound_upper[row++] = b.upper;
    }
  }

  const Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - cvp.constraint_constant;
  const Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - cvp.constraint_constant;

  cvp.bounds_lower.topRows(total_num_cnt) = linearized_cnt_lower;
  cvp.bounds_upper.topRows(total_num_cnt) = linearized_cnt_upper;
}

void TrajOptQPProblem::Implementation::updateNLPVariableBounds()
{
  // Equivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  const Eigen::VectorXd x_initial = variables_->getValues();

  // Set the variable limits once
  Eigen::VectorXd var_bounds_lower(cvp.n_nlp_vars);
  Eigen::VectorXd var_bounds_upper(cvp.n_nlp_vars);

  for (Eigen::Index i = 0; i < cvp.n_nlp_vars; ++i)
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

  const Eigen::Index var_row_index =
      cvp.n_cnt + cvp.n_hinge + cvp.n_abs + cvp.n_dyn_cnt + cvp.n_dyn_hinge + cvp.n_dyn_abs;

  cvp.bounds_lower.segment(var_row_index, cvp.n_nlp_vars) = var_bounds_lower_final;
  cvp.bounds_upper.segment(var_row_index, cvp.n_nlp_vars) = var_bounds_upper_final;
}

void TrajOptQPProblem::Implementation::updateSlackVariableBounds()
{
  Eigen::Index current_cnt_index =
      cvp.n_nlp_vars + cvp.n_cnt + cvp.n_hinge + cvp.n_abs + cvp.n_dyn_cnt + cvp.n_dyn_hinge + cvp.n_dyn_abs;

  for (Eigen::Index i = 0; i < cvp.n_hinge; i++)
  {
    cvp.bounds_lower[current_cnt_index] = 0;
    cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
  }

  for (Eigen::Index i = 0; i < cvp.n_dyn_hinge; i++)
  {
    cvp.bounds_lower[current_cnt_index] = 0;
    cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
  }

  for (Eigen::Index i = 0; i < cvp.n_abs; i++)
  {
    cvp.bounds_lower[current_cnt_index] = 0;
    cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
    cvp.bounds_lower[current_cnt_index] = 0;
    cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
  }

  for (Eigen::Index i = 0; i < cvp.n_dyn_abs; i++)
  {
    cvp.bounds_lower[current_cnt_index] = 0;
    cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
    cvp.bounds_lower[current_cnt_index] = 0;
    cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
  }

  for (const auto& info : cnt_infos)
  {
    for (const auto& constraint_type : info.constraint_types)
    {
      if (constraint_type == ConstraintType::EQ)
      {
        cvp.bounds_lower[current_cnt_index] = 0;
        cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
        cvp.bounds_lower[current_cnt_index] = 0;
        cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
      }
      else
      {
        cvp.bounds_lower[current_cnt_index] = 0;
        cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
      }
    }
  }

  for (const auto& info : dyn_cnt_infos)
  {
    for (const auto& constraint_type : info.constraint_types)
    {
      if (constraint_type == ConstraintType::EQ)
      {
        cvp.bounds_lower[current_cnt_index] = 0;
        cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
        cvp.bounds_lower[current_cnt_index] = 0;
        cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
      }
      else
      {
        cvp.bounds_lower[current_cnt_index] = 0;
        cvp.bounds_upper[current_cnt_index++] = double(INFINITY);
      }
    }
  }
}

TrajOptQPProblem::TrajOptQPProblem() : impl_(std::make_unique<Implementation>()) {}

TrajOptQPProblem::~TrajOptQPProblem() = default;

void TrajOptQPProblem::addVariableSet(std::shared_ptr<trajopt_ifopt::Variables> variable_set)
{
  impl_->addVariableSet(std::move(variable_set));
}

void TrajOptQPProblem::addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set)
{
  impl_->addConstraintSet(std::move(constraint_set));
}

void TrajOptQPProblem::addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                                  CostPenaltyType penalty_type)
{
  impl_->addCostSet(std::move(constraint_set), penalty_type);
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
  return impl_->cvp.evaluateConvexCosts(var_vals);
}

double TrajOptQPProblem::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateTotalExactCost(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateExactCosts(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::getExactCosts() { return evaluateExactCosts(impl_->variables_->getValues()); }

Eigen::VectorXd TrajOptQPProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->cvp.evaluateConvexConstraintViolations(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return impl_->evaluateExactConstraintViolations(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::getExactConstraintViolations()
{
  return evaluateExactConstraintViolations(impl_->variables_->getValues());  // NOLINT
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
  return std::as_const<Implementation>(*impl_).variables_->getRows();
}

Eigen::Index TrajOptQPProblem::getNumNLPConstraints() const
{
  return std::as_const<Implementation>(*impl_).num_cnt_components;
}

Eigen::Index TrajOptQPProblem::getNumNLPCosts() const
{
  return std::as_const<Implementation>(*impl_).num_cost_components;
}

Eigen::Index TrajOptQPProblem::getNumQPVars() const
{
  assert(impl_->initialized_);
  return std::as_const<Implementation>(*impl_).cvp.num_qp_vars;
}

Eigen::Index TrajOptQPProblem::getNumQPConstraints() const
{
  assert(impl_->initialized_);
  return std::as_const<Implementation>(*impl_).cvp.num_qp_cnts;
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

Eigen::Ref<const trajopt_ifopt::Jacobian> TrajOptQPProblem::getHessian() { return impl_->cvp.hessian; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getGradient() { return impl_->cvp.gradient; }

Eigen::Ref<const trajopt_ifopt::Jacobian> TrajOptQPProblem::getConstraintMatrix()
{
  return impl_->cvp.constraint_matrix;
}
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsLower() { return impl_->cvp.bounds_lower; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsUpper() { return impl_->cvp.bounds_upper; }

}  // namespace trajopt_sqp
