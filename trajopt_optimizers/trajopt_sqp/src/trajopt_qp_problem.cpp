
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
 *
 *
 * A slack variable is referred to as an additional variable that has been introduced
 * to the optimization problem to turn a inequality constraint into an equality constraint.
 * Information taken from: https://en.wikipedia.org/wiki/Slack_variable
 *  * As with the other variables in the augmented constraints, the slack variable cannot take on
 * negative values, as the simplex algorithm requires them to be positive or zero.
 *  *   - If a slack variable associated with a constraint is zero at a particular candidate solution,
 *     the constraint is binding there, as the constraint restricts the possible changes from that point.
 *   - If a slack variable is positive at a particular candidate solution, the constraint is non-binding
 *     there, as the constraint does not restrict the possible changes from that point.
 *   - If a slack variable is negative at some point, the point is infeasible (not allowed), as it does
 *     not satisfy the constraint.
 *  * Terminology
 *  *   - If an inequality constraint holds with equality at the optimal point, the constraint is said to
 *     be binding, as the point cannot be varied in the direction of the constraint even though doing
 *     so would improve the value of the objective function.
 *   - If an inequality constraint holds as a strict inequality at the optimal point (that is, does
 *     not hold with equality), the constraint is said to be non-binding, as the point could be varied
 *     in the direction of the constraint, although it would not be optimal to do so. Under certain
 *     conditions, as for example in convex optimization, if a constraint is non-binding, the optimization
 *     problem would have the same solution even in the absence of that constraint.
 *   - If a constraint is not satisfied at a given point, the point is said to be infeasible.
 *  * @example By introducing the slack variable y >= 0, the inequality Ax <= b can be converted
 * to the equation Ax + y = b.
 */

namespace trajopt_sqp
{
struct ComponentInfo
{
  Eigen::Index rows{ 0 };
  Eigen::VectorXd coeffs;
  std::vector<trajopt_ifopt::Bounds> bounds;
};

struct ConstraintInfo
{
  Eigen::Index rows{ 0 };
  Eigen::VectorXd coeffs;
  std::vector<trajopt_ifopt::Bounds> bounds;
  std::vector<Eigen::Triplet<double>> slack_vars;
  std::vector<double> slack_gradients;
};

struct ConvexProblem
{
  // Static quantities
  Eigen::Index n_nlp_vars{ 0 };
  Eigen::Index n_slack_vars{ 0 };

  // These quantities are computed in the update() method
  Eigen::Index n_costs{ 0 };
  Eigen::Index n_cost_constraints{ 0 };
  Eigen::Index n_constraints{ 0 };

  Eigen::Index num_qp_vars{ 0 };
  Eigen::Index num_qp_cnts{ 0 };

  std::vector<ComponentInfo> cost_infos;
  std::vector<ConstraintInfo> cost_constraint_infos;
  std::vector<ConstraintInfo> constraint_infos;

  // These objects are computed in the convexify() method
  trajopt_ifopt::Jacobian hessian;
  Eigen::VectorXd gradient;
  QuadExprs squared_objective_nlp;
  Eigen::VectorXd squared_costs_target;

  trajopt_ifopt::Jacobian constraint_matrix;
  Eigen::VectorXd constraint_constant;  // This should be the center of the bounds

  // These objects are computed in the and convexify() updateNLPVariableBounds() methods
  Eigen::VectorXd bounds_lower;
  Eigen::VectorXd bounds_upper;

  // Scratch buffers reused across evaluations
  Eigen::VectorXd scratch_val;  // holds convex_value or constraint_value
  Eigen::VectorXd scratch_err;  // holds bounds violations

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals);
};

Eigen::VectorXd ConvexProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  /** @note The legacy trajop appears to use all variables include slack for convex cost evaluation */
  const auto total_cost = static_cast<Eigen::Index>(cost_infos.size() + cost_constraint_infos.size());

  if (total_cost == 0)
    return {};

  Eigen::VectorXd costs(total_cost);
  costs.setZero();

  Eigen::Ref<const Eigen::VectorXd> var_block = var_vals.head(n_nlp_vars);

  Eigen::Index cost_idx{ 0 };
  Eigen::Index row_offset{ 0 };

  // Squared costs (already convexified into squared_objective_nlp_)
  if (!cost_infos.empty())
  {
    Eigen::VectorXd sq_costs = squared_objective_nlp.values(var_block);
    assert(!(sq_costs.array() < -1e-8).any());
    for (const auto& c_info : cost_infos)
    {
      costs(cost_idx++) = sq_costs.segment(row_offset, c_info.rows).sum();
      row_offset += c_info.rows;
    }
  }

  // Reset row offset for constraint matrix
  row_offset = 0;

  if (!cost_constraint_infos.empty())
  {
    for (const auto& c_info : cost_constraint_infos)
    {
      if (c_info.rows == 0)
      {
        costs(cost_idx++) = 0;
        continue;
      }

      auto jac = constraint_matrix.middleRows(row_offset, c_info.rows);
      auto constant = constraint_constant.segment(row_offset, c_info.rows);

      // Ensure scratch buffers big enough (no allocation after first growth)
      scratch_val.resize(c_info.rows);
      scratch_err.resize(c_info.rows);

      // scratch_val = constant + jac * var_vals
      scratch_val = constant;                   // copy into scratch (but no alloc)
      scratch_val.noalias() += jac * var_vals;  // mat-vec into existing memory

      // compute violations in-place
      trajopt_ifopt::calcBoundsViolations(scratch_err, scratch_val, c_info.bounds);

      costs(cost_idx++) = scratch_err.sum();
      assert(!(cost.array() < -1e-8).any());
      row_offset += c_info.rows;
    }
  }

  return costs;
}

Eigen::VectorXd ConvexProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  /** @note The legacy trajop does not use slack variables for convex evaluation */
  Eigen::VectorXd violations(constraint_infos.size());
  Eigen::Index cnt_idx{ 0 };
  Eigen::Index row_index{ n_cost_constraints };

  Eigen::Ref<const Eigen::VectorXd> var_block = var_vals.head(n_nlp_vars);
  for (const auto& c_info : constraint_infos)
  {
    if (c_info.rows == 0)
    {
      violations(cnt_idx++) = 0;
      continue;
    }

    // NOLINTNEXTLINE
    auto jac = constraint_matrix.block(row_index, 0, c_info.rows, n_nlp_vars);
    auto constant = constraint_constant.middleRows(row_index, c_info.rows);

    // Ensure scratch buffers big enough (no allocation after first growth)
    scratch_val.resize(c_info.rows);
    scratch_err.resize(c_info.rows);

    // scratch_val = constant + jac * var_vals
    scratch_val = constant;                    // copy into scratch (but no alloc)
    scratch_val.noalias() += jac * var_block;  // mat-vec into existing memory

    // compute violations in-place
    trajopt_ifopt::calcBoundsViolations(scratch_err, scratch_val, c_info.bounds);

    violations(cnt_idx++) = scratch_err.sum();
    row_index += c_info.rows;
  }

  return violations;
}

struct TrajOptQPProblem::Implementation
{
  Implementation() : variables(std::make_shared<trajopt_ifopt::CompositeVariables>("variable-sets")) {}

  bool initialized{ false };

  ///////////////////////////////
  // These will never change size
  ///////////////////////////////
  Eigen::Index num_cost_components{ 0 };
  Eigen::Index num_cnt_components{ 0 };
  bool has_dyn_component{ false };

  std::vector<trajopt_ifopt::Differentiable::Ptr> all_components;
  std::vector<trajopt_ifopt::Differentiable::Ptr> all_costs;
  std::vector<trajopt_ifopt::Differentiable::Ptr> all_cost_constraints;
  std::vector<trajopt_ifopt::Differentiable::Ptr> all_constraints;

  trajopt_ifopt::CompositeVariables::Ptr variables;
  std::vector<trajopt_ifopt::Differentiable::Ptr> constraints;
  std::vector<trajopt_ifopt::Differentiable::Ptr> squared_costs;
  std::vector<trajopt_ifopt::Differentiable::Ptr> hinge_costs;
  std::vector<trajopt_ifopt::Differentiable::Ptr> abs_costs;

  std::vector<trajopt_ifopt::Differentiable::Ptr> dyn_constraint;
  std::vector<trajopt_ifopt::Differentiable::Ptr> dyn_squared_costs;
  std::vector<trajopt_ifopt::Differentiable::Ptr> dyn_hinge_costs;
  std::vector<trajopt_ifopt::Differentiable::Ptr> dyn_abs_costs;

  Eigen::VectorXd constraint_merit_coeff;
  std::vector<double> squared_costs_target;
  std::vector<std::string> constraint_names;
  std::vector<std::string> cost_names;

  Eigen::VectorXd var_bounds_lower;
  Eigen::VectorXd var_bounds_upper;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size;

  std::vector<Eigen::Triplet<double>> cache_triplets_1;
  std::vector<Eigen::Triplet<double>> cache_triplets_2;

  // Scratch buffers reused across evaluations
  Eigen::VectorXd scratch_err;  // holds bounds violations

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

  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& values);

  void setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff);

  void print() const;

  /**
   * @brief Helper that updates the NLP variable bounds (middle section)
   * @details Called by convexify()
   */
  void updateNLPVariableBounds(const Eigen::Ref<const Eigen::VectorXd>& nlp_values);
};

void TrajOptQPProblem::Implementation::addVariableSet(std::shared_ptr<trajopt_ifopt::Variables> variable_set)
{
  variables->addComponent(std::move(variable_set));
  initialized = false;
}

void TrajOptQPProblem::Implementation::addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set)
{
  constraint_set->linkWithVariables(variables);

  if (constraint_set->isDynamic())
    dyn_constraint.emplace_back(std::move(constraint_set));
  else
    constraints.emplace_back(std::move(constraint_set));

  initialized = false;
}

void TrajOptQPProblem::Implementation::addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                                                  CostPenaltyType penalty_type)
{
  constraint_set->linkWithVariables(variables);
  const std::vector<trajopt_ifopt::Bounds> cost_bounds = constraint_set->getBounds();
  switch (penalty_type)
  {
    case CostPenaltyType::SQUARED:
    {
      for (const auto& bound : cost_bounds)
      {
        if (bound.getType() != trajopt_ifopt::BoundsType::EQUALITY)
          throw std::runtime_error("TrajOpt Ifopt squared cost must have equality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_squared_costs.emplace_back(std::move(constraint_set));
      else
        squared_costs.emplace_back(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::ABSOLUTE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (bound.getType() != trajopt_ifopt::BoundsType::EQUALITY)
          throw std::runtime_error("TrajOpt Ifopt absolute cost must have equality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_abs_costs.emplace_back(std::move(constraint_set));
      else
        abs_costs.emplace_back(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::HINGE:
    {
      for (const auto& bound : cost_bounds)
      {
        if (bound.getType() != trajopt_ifopt::BoundsType::LOWER_BOUND &&
            bound.getType() != trajopt_ifopt::BoundsType::UPPER_BOUND)
          throw std::runtime_error("TrajOpt Ifopt hinge cost must have inequality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_hinge_costs.emplace_back(std::move(constraint_set));
      else
        hinge_costs.emplace_back(std::move(constraint_set));

      break;
    }
    default:
    {
      throw std::runtime_error("Unsupport CostPenaltyType!");
    }
  }

  initialized = false;
}

void TrajOptQPProblem::Implementation::update()
{
  if (initialized && !has_dyn_component)
    return;

  cvp.n_slack_vars = 0;
  cvp.n_costs = 0;
  cvp.n_cost_constraints = 0;
  cvp.n_constraints = 0;

  for (std::size_t i = 0; i < all_costs.size(); ++i)
  {
    const auto& cost = all_costs[i];
    if (initialized && !cost->isDynamic())
    {
      cvp.n_costs += cost->getRows();
      continue;
    }

    auto& info = cvp.cost_infos[i];
    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

    cvp.n_costs += info.rows;
  }

  // Hinge cost adds a variable and an inequality constraint (→ 2 constraints)
  // Absolute cost adds two variables and an equality constraint (→ 3 constraints)
  /** @todo update to handle absolute cost correctly */
  for (std::size_t i = 0; i < all_cost_constraints.size(); ++i)
  {
    const auto& cost = all_cost_constraints[i];
    auto& info = cvp.cost_constraint_infos[i];
    if (initialized && !cost->isDynamic())
    {
      cvp.n_cost_constraints += cost->getRows();
      cvp.n_slack_vars += static_cast<Eigen::Index>(info.slack_vars.size());
      continue;
    }

    info.rows = cost->getRows();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();
    info.slack_vars.clear();
    info.slack_gradients.clear();
    info.slack_vars.reserve(2 * info.bounds.size());
    info.slack_gradients.reserve(2 * info.bounds.size());

    for (std::size_t i = 0; i < info.bounds.size(); ++i)
    {
      const auto cnt_bound_type = info.bounds[i].getType();
      const double coeff = info.coeffs(static_cast<Eigen::Index>(i));
      if (cnt_bound_type == trajopt_ifopt::BoundsType::EQUALITY)
      {
        info.slack_vars.emplace_back(i, info.slack_vars.size(), 1.0);
        info.slack_vars.emplace_back(i, info.slack_vars.size(), -1.0);
        info.slack_gradients.emplace_back(coeff);
        info.slack_gradients.emplace_back(coeff);
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::LOWER_BOUND)
      {
        info.slack_vars.emplace_back(i, info.slack_vars.size(), -1.0);
        info.slack_gradients.emplace_back(coeff);
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::UPPER_BOUND)
      {
        info.slack_vars.emplace_back(i, info.slack_vars.size(), 1.0);
        info.slack_gradients.emplace_back(coeff);
      }
      else
      {
        throw std::runtime_error("Unsupported bounds type!");
      }
    }

    cvp.n_cost_constraints += info.rows;
    cvp.n_slack_vars += static_cast<Eigen::Index>(info.slack_vars.size());
  }

  for (std::size_t i = 0; i < all_constraints.size(); ++i)
  {
    const auto& cnt = all_constraints[i];
    auto& info = cvp.constraint_infos[i];
    if (initialized && !cnt->isDynamic())
    {
      cvp.n_constraints += cnt->getRows();
      cvp.n_slack_vars += static_cast<Eigen::Index>(info.slack_vars.size());
      continue;
    }

    info.rows = cnt->getRows();
    info.coeffs = cnt->getCoefficients();
    info.bounds = cnt->getBounds();
    info.slack_vars.clear();
    info.slack_gradients.clear();
    info.slack_vars.reserve(2 * info.bounds.size());
    info.slack_gradients.reserve(2 * info.bounds.size());

    for (std::size_t i = 0; i < info.bounds.size(); ++i)
    {
      const auto cnt_bound_type = info.bounds[i].getType();
      const double coeff = info.coeffs(static_cast<Eigen::Index>(i));
      if (cnt_bound_type == trajopt_ifopt::BoundsType::EQUALITY)
      {
        info.slack_vars.emplace_back(i, info.slack_vars.size(), 1.0);
        info.slack_vars.emplace_back(i, info.slack_vars.size(), -1.0);
        info.slack_gradients.emplace_back(coeff);
        info.slack_gradients.emplace_back(coeff);
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::LOWER_BOUND)
      {
        info.slack_vars.emplace_back(i, info.slack_vars.size(), -1.0);
        info.slack_gradients.emplace_back(coeff);
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::UPPER_BOUND)
      {
        info.slack_vars.emplace_back(i, info.slack_vars.size(), 1.0);
        info.slack_gradients.emplace_back(coeff);
      }
      else
      {
        throw std::runtime_error("Unsupported bounds type!");
      }
    }

    cvp.n_constraints += info.rows;
    cvp.n_slack_vars += static_cast<Eigen::Index>(info.slack_vars.size());
  }

  cvp.num_qp_vars = cvp.n_nlp_vars + cvp.n_slack_vars;
  cvp.num_qp_cnts = cvp.n_cost_constraints + cvp.n_constraints + cvp.num_qp_vars;

  cvp.squared_costs_target = Eigen::VectorXd::Zero(cvp.n_costs);
  if (cvp.n_costs > 0)
  {
    Eigen::Index row{ 0 };
    for (const auto& cost : cvp.cost_infos)
    {
      for (const auto& b : cost.bounds)
        cvp.squared_costs_target(row++) = b.getLower();
    }
  }

  cvp.constraint_constant = Eigen::VectorXd::Zero(cvp.n_cost_constraints + cvp.n_constraints);

  // Initialize the constraint bounds
  // We default to slack variable bounds to avoid having to set those seperatly
  cvp.bounds_lower = Eigen::VectorXd::Constant(cvp.num_qp_cnts, 0.0);
  cvp.bounds_upper = Eigen::VectorXd::Constant(cvp.num_qp_cnts, double(INFINITY));
}

void TrajOptQPProblem::Implementation::setup()
{
  all_costs.clear();
  all_costs.insert(all_costs.end(), squared_costs.begin(), squared_costs.end());
  all_costs.insert(all_costs.end(), dyn_squared_costs.begin(), dyn_squared_costs.end());

  all_cost_constraints.clear();
  all_cost_constraints.insert(all_cost_constraints.end(), hinge_costs.begin(), hinge_costs.end());
  all_cost_constraints.insert(all_cost_constraints.end(), dyn_hinge_costs.begin(), dyn_hinge_costs.end());
  all_cost_constraints.insert(all_cost_constraints.end(), abs_costs.begin(), abs_costs.end());
  all_cost_constraints.insert(all_cost_constraints.end(), dyn_abs_costs.begin(), dyn_abs_costs.end());

  all_constraints.clear();
  all_constraints.insert(all_constraints.end(), constraints.begin(), constraints.end());
  all_constraints.insert(all_constraints.end(), dyn_constraint.begin(), dyn_constraint.end());

  all_components.clear();
  all_components.insert(all_components.end(), all_costs.begin(), all_costs.end());
  all_components.insert(all_components.end(), all_cost_constraints.begin(), all_cost_constraints.end());
  all_components.insert(all_components.end(), all_constraints.begin(), all_constraints.end());

  // Call update
  for (auto& c : all_components)
    c->update();

  // Get count
  cvp.n_costs = 0;
  for (const auto& c : all_costs)
  {
    cvp.n_costs += c->getRows();
    if (c->isDynamic())
      has_dyn_component = true;
  }

  cvp.n_cost_constraints = 0;
  for (const auto& c : all_cost_constraints)
  {
    cvp.n_cost_constraints += c->getRows();
    if (c->isDynamic())
      has_dyn_component = true;
  }

  cvp.n_constraints = 0;
  for (const auto& c : all_constraints)
  {
    cvp.n_constraints += c->getRows();
    if (c->isDynamic())
      has_dyn_component = true;
  }

  // Local counts to avoid repeated virtual / composite queries
  cvp.n_nlp_vars = variables->getRows();

  box_size = Eigen::VectorXd::Constant(cvp.n_nlp_vars, 1e-1);

  // Reset and reserve name buffers (avoid accumulation across multiple setup() calls)
  num_cost_components = static_cast<Eigen::Index>(all_costs.size() + all_cost_constraints.size());

  // Get NLP Cost and Constraint Names for Debug Print
  cvp.cost_infos.clear();
  cvp.cost_infos.resize(all_costs.size());

  cvp.cost_constraint_infos.clear();
  cvp.cost_constraint_infos.resize(all_cost_constraints.size());

  cvp.constraint_infos.clear();
  cvp.constraint_infos.resize(all_constraints.size());

  // Define cost names
  cost_names.clear();
  cost_names.reserve(static_cast<std::size_t>(num_cost_components));
  for (const auto& cost : all_costs)
    cost_names.push_back(cost->getName());

  for (const auto& cost : all_cost_constraints)
    cost_names.push_back(cost->getName());

  // Get NLP bounds and detect constraint type
  num_cnt_components = static_cast<Eigen::Index>(all_constraints.size());
  constraint_merit_coeff = Eigen::VectorXd::Constant(num_cnt_components, 10.0);

  // Define constraint names
  constraint_names.clear();
  constraint_names.reserve(static_cast<std::size_t>(num_cnt_components));
  for (const auto& cnt : all_constraints)
    constraint_names.push_back(cnt->getName());

  // Cache variable bounds (used in updateNLPVariableBounds)
  var_bounds_lower.resize(cvp.n_nlp_vars);
  var_bounds_upper.resize(cvp.n_nlp_vars);
  std::vector<trajopt_ifopt::Bounds> var_bounds = variables->getBounds();
  for (Eigen::Index i = 0; i < cvp.n_nlp_vars; ++i)
  {
    const auto& b = var_bounds[static_cast<std::size_t>(i)];
    var_bounds_lower[i] = b.getLower();
    var_bounds_upper[i] = b.getUpper();
  }

  // Update
  update();

  initialized = true;
}

void TrajOptQPProblem::Implementation::setVariables(const double* x)
{
  variables->setVariables(Eigen::Map<const Eigen::VectorXd>(x, variables->getRows()));

  for (auto& c : all_components)
    c->update();
}

Eigen::VectorXd TrajOptQPProblem::Implementation::getVariableValues() const { return variables->getValues(); }

void TrajOptQPProblem::Implementation::convexify()
{
  assert(initialized);  // NOLINT

  // Update if dynamic constraints are present
  update();

  // Convexify
  const Eigen::Index num_nlp_costs = cvp.n_costs + cvp.n_cost_constraints;

  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  cvp.hessian.resize(cvp.num_qp_vars, cvp.num_qp_vars);

  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  cvp.gradient = Eigen::VectorXd::Zero(cvp.num_qp_vars);

  const Eigen::VectorXd x_initial = variables->getValues();

  // Create triplet list of nonzero gradients
  cache_triplets_1.clear();
  cache_triplets_1.reserve(static_cast<std::size_t>(cvp.n_nlp_vars * num_nlp_costs) * 3);

  // Process Squared Costs
  /** @note See CostFromFunc::convex in modeling_utils.cpp. */
  if (cvp.n_costs > 0)
  {
    cvp.squared_objective_nlp = QuadExprs(cvp.n_costs, cvp.n_nlp_vars);
    Eigen::Index row = 0;
    for (const auto& c : all_costs)
    {
      // This is not correct should pass the value to createAffExprs then use bound to which could change the sign of
      // the affine expression
      //    Eigen::VectorXd cnt_error = trajopt_ifopt::calcBoundsErrors(cnt_vals, squared_costs_.getBounds());

      // This should be correct now
      AffExprs cnt_aff_expr = createAffExprs(c->getValues(), c->getJacobian(), x_initial);
      cnt_aff_expr.constants = (cvp.squared_costs_target.segment(row, c->getRows()) - cnt_aff_expr.constants);
      cnt_aff_expr.linear_coeffs *= -1;
      QuadExprs cost_quad_expr = squareAffExprs(cnt_aff_expr, c->getCoefficients());

      // store individual equations constant
      cvp.squared_objective_nlp.constants.segment(row, c->getRows()) = cost_quad_expr.constants;

      // Sum objective function linear coefficients
      cvp.squared_objective_nlp.objective_linear_coeffs += cost_quad_expr.objective_linear_coeffs;

      // Sum objective function quadratic coefficients
      cvp.squared_objective_nlp.objective_quadratic_coeffs += cost_quad_expr.objective_quadratic_coeffs;

      // store individual equations linear coefficients in a Triplet list to update equation linear coefficients later
      if (cost_quad_expr.linear_coeffs.nonZeros() > 0)
      {
        // Add jacobian to triplet list
        for (int k = 0; k < cost_quad_expr.linear_coeffs.outerSize(); ++k)
        {
          for (trajopt_ifopt::Jacobian::InnerIterator it(cost_quad_expr.linear_coeffs, k); it; ++it)
            cache_triplets_1.emplace_back(row + it.row(), it.col(), it.value());
        }
      }

      // Store individual equations quadratic coefficients
      cvp.squared_objective_nlp.quadratic_coeffs.insert(cvp.squared_objective_nlp.quadratic_coeffs.end(),
                                                        cost_quad_expr.quadratic_coeffs.begin(),
                                                        cost_quad_expr.quadratic_coeffs.end());

      row += c->getRows();
    }

    // Store individual equations linear coefficients
    cvp.squared_objective_nlp.linear_coeffs.setFromTriplets(cache_triplets_1.begin(),
                                                            cache_triplets_1.end());  // NOLINT

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

  /** Use cache triplet and clear */
  cache_triplets_2.clear();
  cache_triplets_2.reserve(static_cast<std::size_t>(cvp.num_qp_vars + cvp.num_qp_vars));

  Eigen::Index constraint_matrix_row{ 0 };
  Eigen::Index constraint_matrix_non_zeros{ 0 };
  Eigen::Index current_var_index{ cvp.n_nlp_vars };
  ////////////////////////////////////////////////////////
  // Set the gradient of the cost constraint variables
  ////////////////////////////////////////////////////////
  for (std::size_t i = 0; i < all_cost_constraints.size(); ++i)
  {
    const auto& info = cvp.cost_constraint_infos[i];
    const auto& cost = all_cost_constraints[i];
    if (info.rows == 0)
      continue;

    // Slack Variable Gradient
    cvp.gradient.segment(current_var_index, info.slack_gradients.size()) = Eigen::Map<const Eigen::VectorXd>(
        info.slack_gradients.data(), static_cast<Eigen::Index>(info.slack_gradients.size()));
    current_var_index += static_cast<Eigen::Index>(info.slack_gradients.size());

    // Linearize Constraints
    const trajopt_ifopt::Jacobian jac = cost->getJacobian();
    for (int k = 0; k < jac.outerSize(); ++k)
      for (trajopt_ifopt::Jacobian::InnerIterator it(jac, k); it; ++it)
        cache_triplets_2.emplace_back(constraint_matrix_row + it.row(), it.col(), it.value());

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
    auto cc = cvp.constraint_constant.segment(constraint_matrix_row, info.rows);
    cc = cost->getValues();
    cc.noalias() -= jac * x_initial;

    // Update NLP Constraint Bounds
    for (std::size_t j = 0; j < info.bounds.size(); ++j)
    {
      const auto& b = info.bounds[j];
      const Eigen::Index row = constraint_matrix_row + static_cast<Eigen::Index>(j);
      const double constant = cvp.constraint_constant[row];
      cvp.bounds_lower[row] = b.getLower() - constant;
      cvp.bounds_upper[row] = b.getUpper() - constant;
    }

    constraint_matrix_non_zeros += jac.nonZeros();
    constraint_matrix_row += info.rows;
  }

  ////////////////////////////////////////////////////////
  // Set the gradient of the constraint slack variables
  ////////////////////////////////////////////////////////

  for (std::size_t i = 0; i < all_constraints.size(); ++i)
  {
    const auto& info = cvp.constraint_infos[i];
    const auto& cnt = all_constraints[i];

    // Slack Variable Gradient
    cvp.gradient.segment(current_var_index, info.slack_gradients.size()) =
        constraint_merit_coeff[static_cast<Eigen::Index>(i)] *
        Eigen::Map<const Eigen::VectorXd>(info.slack_gradients.data(),
                                          static_cast<Eigen::Index>(info.slack_gradients.size()));
    current_var_index += static_cast<Eigen::Index>(info.slack_gradients.size());

    // Linearize Constraints
    const trajopt_ifopt::Jacobian jac = cnt->getJacobian();
    for (int k = 0; k < jac.outerSize(); ++k)
      for (trajopt_ifopt::Jacobian::InnerIterator it(jac, k); it; ++it)
        cache_triplets_2.emplace_back(constraint_matrix_row + it.row(), it.col(), it.value());

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
    auto cc = cvp.constraint_constant.segment(constraint_matrix_row, info.rows);
    cc = cnt->getValues();
    cc.noalias() -= jac * x_initial;

    // Update NLP Constraint Bounds
    for (std::size_t j = 0; j < info.bounds.size(); ++j)
    {
      const auto& b = info.bounds[j];
      const Eigen::Index row = constraint_matrix_row + static_cast<Eigen::Index>(j);
      const double constant = cvp.constraint_constant[row];
      cvp.bounds_lower[row] = b.getLower() - constant;
      cvp.bounds_upper[row] = b.getUpper() - constant;
    }

    constraint_matrix_non_zeros += jac.nonZeros();
    constraint_matrix_row += info.rows;
  }

  ////////////////////////////////////////////////////////
  // Set the slack variables constraint matrix
  ////////////////////////////////////////////////////////

  // Reset row index
  constraint_matrix_row = 0;

  Eigen::Index current_col_index = cvp.n_nlp_vars;
  for (const auto& info : cvp.cost_constraint_infos)
  {
    for (const auto& var : info.slack_vars)
      cache_triplets_2.emplace_back(constraint_matrix_row + var.row(), current_col_index + var.col(), var.value());

    constraint_matrix_row += info.rows;
    current_col_index += static_cast<Eigen::Index>(info.slack_vars.size());
  }

  for (const auto& info : cvp.constraint_infos)
  {
    for (const auto& var : info.slack_vars)
      cache_triplets_2.emplace_back(constraint_matrix_row + var.row(), current_col_index + var.col(), var.value());

    constraint_matrix_row += info.rows;
    current_col_index += static_cast<Eigen::Index>(info.slack_vars.size());
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  constraint_matrix_row = cvp.n_constraints + cvp.n_cost_constraints;
  for (Eigen::Index i = 0; i < cvp.num_qp_vars; ++i)
    cache_triplets_2.emplace_back(constraint_matrix_row + i, i, 1.0);

  // Insert the triplet list into the sparse matrix
  cvp.constraint_matrix.resize(cvp.num_qp_cnts, cvp.num_qp_vars);
  cvp.constraint_matrix.reserve(constraint_matrix_non_zeros + cvp.num_qp_vars);
  cvp.constraint_matrix.setFromTriplets(cache_triplets_2.begin(), cache_triplets_2.end());

  // Update NLP Bounds
  updateNLPVariableBounds(x_initial);
}

double TrajOptQPProblem::Implementation::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return evaluateExactCosts(var_vals).sum();
}

Eigen::VectorXd
TrajOptQPProblem::Implementation::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& /*var_vals*/)
{
  assert(initialized);

  if (num_cost_components == 0)
    return {};

  Eigen::VectorXd g(num_cost_components);
  Eigen::Index cost_idx{ 0 };

  for (const auto& c : all_costs)
  {
    if (c->getRows() == 0)
    {
      g(cost_idx++) = 0;
      continue;
    }

    scratch_err.resize(c->getRows());
    trajopt_ifopt::calcBoundsViolations(scratch_err, c->getValues(), c->getBounds());
    g(cost_idx++) = (scratch_err.array().square() * c->getCoefficients().array()).sum();
  }

  for (const auto& c : all_cost_constraints)
  {
    if (c->getRows() == 0)
    {
      g(cost_idx++) = 0;
      continue;
    }

    scratch_err.resize(c->getRows());
    trajopt_ifopt::calcBoundsViolations(scratch_err, c->getValues(), c->getBounds());
    g(cost_idx++) = scratch_err.sum();
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::Implementation::evaluateExactConstraintViolations(
    const Eigen::Ref<const Eigen::VectorXd>& /*var_vals*/)
{
  Eigen::VectorXd violations(all_constraints.size());
  Eigen::Index cnt_idx{ 0 };

  for (const auto& c : all_constraints)
  {
    if (c->getRows() == 0)
    {
      violations(cnt_idx++) = 0;
      continue;
    }

    scratch_err.resize(c->getRows());
    trajopt_ifopt::calcBoundsViolations(scratch_err, c->getValues(), c->getBounds());
    violations(cnt_idx++) = scratch_err.sum();
  }

  return violations;
}

void TrajOptQPProblem::Implementation::scaleBoxSize(double& scale)
{
  box_size = box_size * scale;
  updateNLPVariableBounds(variables->getValues());
}

void TrajOptQPProblem::Implementation::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& values)
{
  assert(values.size() == variables->getRows());
  box_size = values;
  updateNLPVariableBounds(variables->getValues());
}

void TrajOptQPProblem::Implementation::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  assert(merit_coeff.size() == constraints.size() + dyn_constraint.size());
  constraint_merit_coeff = merit_coeff;
}

void TrajOptQPProblem::Implementation::print() const
{
  Eigen::Index total_cnt = cvp.n_cost_constraints + cvp.n_constraints;
  const Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << '\n';
  std::cout << "Num NLP Vars: " << cvp.n_nlp_vars << '\n';
  std::cout << "Num QP Vars: " << cvp.num_qp_vars << '\n';
  std::cout << "Num NLP Constraints: " << cvp.num_qp_cnts << '\n';
  std::cout << "Box Size: " << box_size.transpose().format(format) << '\n';  // NOLINT
  std::cout << "Constraint Merit Coeff: " << constraint_merit_coeff.transpose().format(format) << '\n';

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
  for (const auto& v_set : variables->getComponents())
    std::cout << v_set->getValues().transpose().format(format) << '\n';
}

void TrajOptQPProblem::Implementation::updateNLPVariableBounds(const Eigen::Ref<const Eigen::VectorXd>& nlp_values)
{
  // Equivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  const Eigen::Index idx = cvp.n_constraints + cvp.n_cost_constraints;

  auto lower = cvp.bounds_lower.segment(idx, cvp.n_nlp_vars);
  auto upper = cvp.bounds_upper.segment(idx, cvp.n_nlp_vars);

  // Assumes x, var_bounds_lower/upper are size n (or you use matching segments)
  for (Eigen::Index i = 0; i < cvp.n_nlp_vars; ++i)
  {
    const double xi = nlp_values[i];
    const double bi = box_size[i];
    const double lb = var_bounds_lower[i];
    const double ub = var_bounds_upper[i];

    lower[i] = std::max(lb, std::min(xi, ub - bi) - bi);
    upper[i] = std::min(ub, std::max(xi, lb + bi) + bi);
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

Eigen::VectorXd TrajOptQPProblem::getExactCosts() { return evaluateExactCosts(impl_->variables->getValues()); }

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
  return evaluateExactConstraintViolations(impl_->variables->getValues());  // NOLINT
}

void TrajOptQPProblem::scaleBoxSize(double& scale) { impl_->scaleBoxSize(scale); }

void TrajOptQPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) { impl_->setBoxSize(box_size); }

void TrajOptQPProblem::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  impl_->setConstraintMeritCoeff(merit_coeff);
}

Eigen::VectorXd TrajOptQPProblem::getBoxSize() const { return std::as_const<Implementation>(*impl_).box_size; }

void TrajOptQPProblem::print() const { std::as_const<Implementation>(*impl_).print(); }

Eigen::Index TrajOptQPProblem::getNumNLPVars() const
{
  return std::as_const<Implementation>(*impl_).variables->getRows();
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
  assert(impl_->initialized);
  return std::as_const<Implementation>(*impl_).cvp.num_qp_vars;
}

Eigen::Index TrajOptQPProblem::getNumQPConstraints() const
{
  assert(impl_->initialized);
  return std::as_const<Implementation>(*impl_).cvp.num_qp_cnts;
}

const std::vector<std::string>& TrajOptQPProblem::getNLPConstraintNames() const
{
  return std::as_const<Implementation>(*impl_).constraint_names;
}
const std::vector<std::string>& TrajOptQPProblem::getNLPCostNames() const
{
  return std::as_const<Implementation>(*impl_).cost_names;
}

Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoxSize() { return impl_->box_size; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getConstraintMeritCoeff() { return impl_->constraint_merit_coeff; }

Eigen::Ref<const trajopt_ifopt::Jacobian> TrajOptQPProblem::getHessian() { return impl_->cvp.hessian; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getGradient() { return impl_->cvp.gradient; }

Eigen::Ref<const trajopt_ifopt::Jacobian> TrajOptQPProblem::getConstraintMatrix()
{
  return impl_->cvp.constraint_matrix;
}
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsLower() { return impl_->cvp.bounds_lower; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsUpper() { return impl_->cvp.bounds_upper; }

}  // namespace trajopt_sqp
