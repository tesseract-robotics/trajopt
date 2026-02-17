
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/expressions.h>
#include <trajopt_sqp/types.h>

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
enum class ComponentInfoType : std::uint8_t
{
  kObjectiveSquared = 0,
  kPenaltyHinge,
  kPenaltyAbsolute,
  kMeritConstraint,
  kUnknown
};

struct ComponentInfo
{
  ComponentInfoType type{ ComponentInfoType::kUnknown };
  Eigen::Index rows{ 0 };
  Eigen::Index non_zeros{ 0 };
  Eigen::VectorXd coeffs;
  std::vector<trajopt_ifopt::Bounds> bounds;
};

struct ConvexProblem
{
  // Static quantities
  Eigen::Index n_nlp_vars{ 0 };
  Eigen::Index n_slack_vars{ 0 };

  // These quantities are computed in the update() method
  Eigen::Index n_objective_terms{ 0 };
  Eigen::Index n_constraint_terms{ 0 };  // (n_penalty_constraints + n_merit_constraints)
  Eigen::Index n_penalty_constraints{ 0 };
  Eigen::Index n_merit_constraints{ 0 };

  Eigen::Index n_objective_term_non_zeros{ 0 };
  Eigen::Index n_constraint_term_non_zeros{ 0 };
  Eigen::Index n_penalty_constraint_non_zeros{ 0 };
  Eigen::Index n_merit_constraint_non_zeros{ 0 };

  Eigen::Index num_qp_vars{ 0 };
  Eigen::Index num_qp_cnts{ 0 };

  std::vector<ComponentInfo> objective_term_infos;
  std::vector<std::reference_wrapper<ComponentInfo>> constraint_term_infos;  // (penalty_constraint_infos +
                                                                             // merit_constraint_infos)
  std::vector<ComponentInfo> penalty_constraint_infos;
  std::vector<ComponentInfo> merit_constraint_infos;

  // These objects are computed in the convexify() method
  trajopt_ifopt::Jacobian hessian;
  Eigen::VectorXd gradient;
  QuadExprs squared_objective_nlp;
  Eigen::VectorXd squared_objective_target;

  trajopt_ifopt::Jacobian constraint_matrix;
  Eigen::VectorXd constraint_constant;  // This should be the center of the bounds

  // These objects are computed in the and convexify() updateNLPVariableBounds() methods
  Eigen::VectorXd bounds_lower;
  Eigen::VectorXd bounds_upper;

  // Scratch buffers reused across evaluations
  mutable Eigen::VectorXd scratch_val;  // holds convex_value or constraint_value
  mutable Eigen::VectorXd scratch_err;  // holds bounds violations

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const;

  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const;
};

Eigen::VectorXd ConvexProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  /** @note The legacy trajop appears to use all variables include slack for convex cost evaluation */
  const auto total_cost = static_cast<Eigen::Index>(objective_term_infos.size() + penalty_constraint_infos.size());

  if (total_cost == 0)
    return {};

  Eigen::VectorXd costs(total_cost);
  costs.setZero();

  Eigen::Ref<const Eigen::VectorXd> var_block = var_vals.head(n_nlp_vars);

  Eigen::Index cost_idx{ 0 };
  Eigen::Index row_offset{ 0 };

  // Squared costs (already convexified into squared_objective_nlp_)
  if (!objective_term_infos.empty())
  {
    // reuse scratch_val (ensure big enough once)
    if (scratch_val.size() < n_objective_terms)
      scratch_val.resize(n_objective_terms);

    auto sq_costs = scratch_val.head(n_objective_terms);
    squared_objective_nlp.values(sq_costs, var_block);

    assert(!(sq_costs.array() < -1e-8).any());
    for (const auto& c_info : objective_term_infos)
    {
      costs(cost_idx++) = sq_costs.segment(row_offset, c_info.rows).sum();
      row_offset += c_info.rows;
    }
  }

  // Reset row offset for constraint matrix
  row_offset = 0;
  for (const auto& c_info : penalty_constraint_infos)
  {
    if (c_info.rows == 0)
    {
      costs(cost_idx++) = 0;
      continue;
    }

    auto jac = constraint_matrix.middleRows(row_offset, c_info.rows);
    auto constant = constraint_constant.segment(row_offset, c_info.rows);

    // Ensure scratch buffers big enough (no allocation after first growth)
    if (scratch_val.size() < c_info.rows)
      scratch_val.resize(c_info.rows);
    if (scratch_err.size() < c_info.rows)
      scratch_err.resize(c_info.rows);

    auto val = scratch_val.head(c_info.rows);
    auto err = scratch_err.head(c_info.rows);

    // scratch_val = constant + jac * var_vals
    val = constant;                   // copy into scratch (but no alloc)
    val.noalias() += jac * var_vals;  // mat-vec into existing memory

    // compute violations in-place
    trajopt_ifopt::calcBoundsViolations(err, val, c_info.bounds);

    costs(cost_idx++) = err.sum();
    assert(!(err.array() < -1e-8).any());
    row_offset += c_info.rows;
  }

  return costs;
}

Eigen::VectorXd
ConvexProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  /** @note The legacy trajop does not use slack variables for convex evaluation */
  Eigen::VectorXd violations(merit_constraint_infos.size());
  Eigen::Index cnt_idx{ 0 };
  Eigen::Index row_index{ n_penalty_constraints };

  Eigen::Ref<const Eigen::VectorXd> var_block = var_vals.head(n_nlp_vars);
  for (const auto& c_info : merit_constraint_infos)
  {
    if (c_info.rows == 0)
    {
      violations(cnt_idx++) = 0;
      continue;
    }

    // NOLINTNEXTLINE
    auto jac = constraint_matrix.middleRows(row_index, c_info.rows).leftCols(n_nlp_vars);
    auto constant = constraint_constant.middleRows(row_index, c_info.rows);

    // Ensure scratch buffers big enough (no allocation after first growth)
    if (scratch_val.size() < c_info.rows)
      scratch_val.resize(c_info.rows);
    if (scratch_err.size() < c_info.rows)
      scratch_err.resize(c_info.rows);

    auto val = scratch_val.head(c_info.rows);
    auto err = scratch_err.head(c_info.rows);

    // scratch_val = constant + jac * var_vals
    val = constant;                    // copy into scratch (but no alloc)
    val.noalias() += jac * var_block;  // mat-vec into existing memory

    // compute violations in-place
    trajopt_ifopt::calcBoundsViolations(err, val, c_info.bounds);

    violations(cnt_idx++) = err.sum();
    row_index += c_info.rows;
  }

  return violations;
}

struct TrajOptQPProblem::Implementation
{
  Implementation(std::shared_ptr<trajopt_ifopt::Variables> variables) : variables(std::move(variables)) {}

  bool initialized{ false };

  ///////////////////////////////
  // These will never change size
  ///////////////////////////////
  Eigen::Index num_cost_components{ 0 };
  Eigen::Index num_cnt_components{ 0 };
  bool has_dyn_component{ false };

  /**
   * @brief Convex objective-only terms (no additional QP constraint rows).
   * @details
   * These are the standard NLP cost terms that are convexified into the QP objective
   * (i.e., contribute to the Hessian/gradient directly). In the current implementation,
   * this bucket contains the squared-cost terms.
   *
   * @note These terms do not introduce slack variables or constraint rows by themselves.
   */
  std::vector<trajopt_ifopt::Differentiable::Ptr> objective_terms;

  /**
   * @brief Constraint-like terms that are modeled as penalties in the objective.
   * @details
   * These terms originate as constraints (e.g., hinge and absolute penalties), but are
   * treated as "costified constraints" by introducing slack variables and adding the
   * corresponding constraint rows to the QP. Their influence on the optimization is
   * primarily through the objective via the slack variable gradient contribution.
   *
   * In other words:
   *  - They still appear in the QP constraint matrix (to relate the slack variables to
   *    the linearized constraint expression),
   *  - But they conceptually belong to the cost/penalty side of the merit function
   *    (i.e., they are not weighted by @ref constraint_merit_coeff).
   *
   */
  std::vector<trajopt_ifopt::Differentiable::Ptr> penalty_constraints;

  /**
   * @brief Primary NLP constraints that participate in the merit function as constraints.
   * @details
   * These are the constraints that the high-level SQP/trust-region layer treats as
   * "true constraints" for convergence/merit calculations. They are linearized into the
   * QP constraint matrix and are typically weighted in the merit function using
   * @ref constraint_merit_coeff (e.g., by scaling the slack variable gradients).
   *
   * @note Although slack variables may be introduced for these constraints (to convert
   * inequalities to equalities), the defining characteristic of this bucket is that
   * their violation contributes to the merit function as a constraint term, not merely
   * as a penalty.
   *
   */
  std::vector<trajopt_ifopt::Differentiable::Ptr> merit_constraints;

  /**
   * @brief Convenience container for all QP constraint-row producing terms.
   * @details
   * This is the union of:
   *  - @ref penalty_constraints (hinge/abs style costified constraints)
   *  - @ref merit_constraints   (primary NLP constraints)
   *
   * These terms are the ones that populate:
   *  - the linearized constraint Jacobian block(s),
   *  - the slack-variable coupling structure,
   *  - and the constraint bounds / constants used for merit evaluation.
   */
  std::vector<trajopt_ifopt::Differentiable::Ptr> constraint_terms;

  /**
   * @brief Convenience container for all differentiable components in the QP problem.
   * @details
   * This is the union of:
   *  - @ref objective_terms
   *  - @ref constraint_terms
   *
   * It is typically used for bulk operations such as:
   *  - calling @c update() on every component,
   *  - iterating over all terms for setup / bookkeeping,
   *  - centralized handling of dynamic components.
   */
  std::vector<trajopt_ifopt::Differentiable::Ptr> all_components;

  trajopt_ifopt::Variables::Ptr variables;
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

  // These two help reduce number of calls to update method
  int n_dyn_components{ 0 };
  int n_dyn_components_prev{ -1 };

  std::vector<Eigen::Triplet<double>> cache_triplets_1;
  std::vector<Eigen::Triplet<double>> cache_triplets_2;
  std::vector<double> cache_slack_gradient;
  AffExprs cache_aff_expr;
  QuadExprs cache_quad_expr;

  // Scratch buffers reused across evaluations
  mutable Eigen::VectorXd scratch_err;  // holds bounds violations

  ConvexProblem cvp;

  void addVariableSet(std::shared_ptr<trajopt_ifopt::Variables> variable_set);

  void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set);

  void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set, CostPenaltyType penalty_type);

  void setup();

  void update();

  void setVariables(const double* x);

  Eigen::VectorXd getVariableValues() const;

  void convexify();

  double getTotalExactCost() const;

  Eigen::VectorXd getExactCosts() const;

  Eigen::VectorXd getExactConstraintViolations() const;

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
    case CostPenaltyType::kSquared:
    {
      for (const auto& bound : cost_bounds)
      {
        if (bound.getType() != trajopt_ifopt::BoundsType::kEquality)
          throw std::runtime_error("TrajOpt Ifopt squared cost must have equality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_squared_costs.emplace_back(std::move(constraint_set));
      else
        squared_costs.emplace_back(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::kAbsolute:
    {
      for (const auto& bound : cost_bounds)
      {
        if (bound.getType() != trajopt_ifopt::BoundsType::kEquality)
          throw std::runtime_error("TrajOpt Ifopt absolute cost must have equality bounds!");
      }

      if (constraint_set->isDynamic())
        dyn_abs_costs.emplace_back(std::move(constraint_set));
      else
        abs_costs.emplace_back(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::kHinge:
    {
      for (const auto& bound : cost_bounds)
      {
        if (bound.getType() != trajopt_ifopt::BoundsType::kLowerBound &&
            bound.getType() != trajopt_ifopt::BoundsType::kUpperBound)
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
  if (initialized && (!has_dyn_component || (n_dyn_components_prev == 0 && n_dyn_components == 0)))
    return;

  n_dyn_components_prev = n_dyn_components;

  cvp.n_objective_terms = 0;
  cvp.n_penalty_constraints = 0;
  cvp.n_merit_constraints = 0;

  cvp.n_objective_term_non_zeros = 0;
  cvp.n_constraint_term_non_zeros = 0;
  cvp.n_penalty_constraint_non_zeros = 0;
  cvp.n_merit_constraint_non_zeros = 0;

  for (std::size_t i = 0; i < objective_terms.size(); ++i)
  {
    const auto& cost = objective_terms[i];
    if (initialized && !cost->isDynamic())
    {
      cvp.n_objective_terms += cost->getRows();
      cvp.n_objective_term_non_zeros += cost->getNonZeros();
      continue;
    }

    auto& info = cvp.objective_term_infos[i];
    info.rows = cost->getRows();
    info.non_zeros = cost->getNonZeros();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

    cvp.n_objective_terms += info.rows;
    cvp.n_objective_term_non_zeros += info.non_zeros;
  }

  // Hinge cost adds a variable and an inequality constraint (→ 2 constraints)
  // Absolute cost adds two variables and an equality constraint (→ 3 constraints)
  /** @todo update to handle absolute cost correctly */
  for (std::size_t i = 0; i < penalty_constraints.size(); ++i)
  {
    const auto& cost = penalty_constraints[i];
    auto& info = cvp.penalty_constraint_infos[i];
    if (initialized && !cost->isDynamic())
    {
      cvp.n_penalty_constraints += cost->getRows();
      cvp.n_penalty_constraint_non_zeros += cost->getNonZeros();
      continue;
    }

    info.rows = cost->getRows();
    info.non_zeros = cost->getNonZeros();
    info.coeffs = cost->getCoefficients();
    info.bounds = cost->getBounds();

    cvp.n_penalty_constraints += info.rows;
    cvp.n_penalty_constraint_non_zeros += info.non_zeros;
  }

  for (std::size_t i = 0; i < merit_constraints.size(); ++i)
  {
    const auto& cnt = merit_constraints[i];
    auto& info = cvp.merit_constraint_infos[i];
    if (initialized && !cnt->isDynamic())
    {
      cvp.n_merit_constraints += cnt->getRows();
      cvp.n_merit_constraint_non_zeros += cnt->getNonZeros();
      continue;
    }

    info.rows = cnt->getRows();
    info.non_zeros = cnt->getNonZeros();
    info.coeffs = cnt->getCoefficients();
    info.bounds = cnt->getBounds();

    cvp.n_merit_constraints += info.rows;
    cvp.n_merit_constraint_non_zeros += info.non_zeros;
  }

  cvp.n_constraint_terms = cvp.n_penalty_constraints + cvp.n_merit_constraints;
  cvp.n_constraint_term_non_zeros = cvp.n_penalty_constraint_non_zeros + cvp.n_merit_constraint_non_zeros;

  cvp.squared_objective_target.setZero(cvp.n_objective_terms);
  cvp.constraint_constant.setZero(cvp.n_constraint_terms);

  cvp.bounds_lower.resize(cvp.n_nlp_vars + cvp.n_penalty_constraints + cvp.n_merit_constraints);
  cvp.bounds_upper.resize(cvp.n_nlp_vars + cvp.n_penalty_constraints + cvp.n_merit_constraints);
}

void TrajOptQPProblem::Implementation::setup()
{
  objective_terms.clear();
  objective_terms.insert(objective_terms.end(), squared_costs.begin(), squared_costs.end());
  objective_terms.insert(objective_terms.end(), dyn_squared_costs.begin(), dyn_squared_costs.end());

  penalty_constraints.clear();
  penalty_constraints.insert(penalty_constraints.end(), hinge_costs.begin(), hinge_costs.end());
  penalty_constraints.insert(penalty_constraints.end(), dyn_hinge_costs.begin(), dyn_hinge_costs.end());
  penalty_constraints.insert(penalty_constraints.end(), abs_costs.begin(), abs_costs.end());
  penalty_constraints.insert(penalty_constraints.end(), dyn_abs_costs.begin(), dyn_abs_costs.end());

  merit_constraints.clear();
  merit_constraints.insert(merit_constraints.end(), constraints.begin(), constraints.end());
  merit_constraints.insert(merit_constraints.end(), dyn_constraint.begin(), dyn_constraint.end());

  constraint_terms.clear();
  constraint_terms.insert(constraint_terms.end(), penalty_constraints.begin(), penalty_constraints.end());
  constraint_terms.insert(constraint_terms.end(), merit_constraints.begin(), merit_constraints.end());

  all_components.clear();
  all_components.insert(all_components.end(), objective_terms.begin(), objective_terms.end());
  all_components.insert(all_components.end(), penalty_constraints.begin(), penalty_constraints.end());
  all_components.insert(all_components.end(), merit_constraints.begin(), merit_constraints.end());

  // Call update
  n_dyn_components = 0;
  for (auto& c : all_components)
  {
    int cnt = c->update();
    if (c->isDynamic())
      n_dyn_components += cnt;
  }

  // Get count
  cvp.n_objective_terms = 0;
  has_dyn_component = false;
  for (const auto& c : objective_terms)
  {
    cvp.n_objective_terms += c->getRows();
    if (c->isDynamic())
      has_dyn_component = true;
  }

  cvp.n_penalty_constraints = 0;
  for (const auto& c : penalty_constraints)
  {
    cvp.n_penalty_constraints += c->getRows();
    if (c->isDynamic())
      has_dyn_component = true;
  }

  cvp.n_merit_constraints = 0;
  for (const auto& c : merit_constraints)
  {
    cvp.n_merit_constraints += c->getRows();
    if (c->isDynamic())
      has_dyn_component = true;
  }

  // Local counts to avoid repeated virtual / composite queries
  cvp.n_nlp_vars = variables->getRows();

  box_size = Eigen::VectorXd::Constant(cvp.n_nlp_vars, 1e-1);

  // Reset and reserve name buffers (avoid accumulation across multiple setup() calls)
  num_cost_components = static_cast<Eigen::Index>(objective_terms.size() + penalty_constraints.size());

  // Get NLP Cost and Constraint Names for Debug Print
  cvp.objective_term_infos.clear();
  cvp.objective_term_infos.resize(objective_terms.size());

  cvp.penalty_constraint_infos.clear();
  cvp.penalty_constraint_infos.resize(penalty_constraints.size());

  cvp.merit_constraint_infos.clear();
  cvp.merit_constraint_infos.resize(merit_constraints.size());

  cvp.constraint_term_infos.clear();

  // Define cost names
  cost_names.clear();
  cost_names.reserve(static_cast<std::size_t>(num_cost_components));
  for (std::size_t i = 0; i < objective_terms.size(); ++i)
  {
    cost_names.push_back(objective_terms[i]->getName());
    cvp.objective_term_infos[i].type = ComponentInfoType::kObjectiveSquared;
  }

  for (std::size_t i = 0; i < penalty_constraints.size(); ++i)
  {
    cost_names.push_back(penalty_constraints[i]->getName());

    auto& info = cvp.penalty_constraint_infos[i];
    info.type = (i < (hinge_costs.size() + dyn_hinge_costs.size())) ? ComponentInfoType::kPenaltyHinge :
                                                                      ComponentInfoType::kPenaltyAbsolute;
    cvp.constraint_term_infos.emplace_back(info);
  }

  // Get NLP bounds and detect constraint type
  num_cnt_components = static_cast<Eigen::Index>(merit_constraints.size());
  constraint_merit_coeff = Eigen::VectorXd::Constant(num_cnt_components, 10.0);

  // Define constraint names
  constraint_names.clear();
  constraint_names.reserve(static_cast<std::size_t>(num_cnt_components));
  for (std::size_t i = 0; i < merit_constraints.size(); ++i)
  {
    constraint_names.push_back(merit_constraints[i]->getName());

    auto& info = cvp.merit_constraint_infos[i];
    info.type = ComponentInfoType::kMeritConstraint;
    cvp.constraint_term_infos.emplace_back(info);
  }

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
  const std::size_t hash = variables->getHash();
  variables->setVariables(Eigen::Map<const Eigen::VectorXd>(x, variables->getRows()));

  // Skip update if variables did not change
  if (hash == variables->getHash())
    return;

  n_dyn_components = 0;
  for (auto& c : all_components)
  {
    int cnt = c->update();
    if (c->isDynamic())
      n_dyn_components += cnt;
  }
}

Eigen::VectorXd TrajOptQPProblem::Implementation::getVariableValues() const { return variables->getValues(); }

void TrajOptQPProblem::Implementation::convexify()
{
  assert(initialized);  // NOLINT

  // Update if dynamic constraints are present
  update();

  // Get current variable values
  const Eigen::VectorXd x_initial = variables->getValues();

  // Convexify
  // Hinge and Asolute costs are handled differently than squared cost because they add constraints to the qp problem

  /** Use cache triplet and clear */
  cache_triplets_2.clear();
  cache_triplets_2.reserve(
      static_cast<std::size_t>(cvp.n_constraint_term_non_zeros + (cvp.n_constraint_terms * 3) + cvp.n_nlp_vars));
  cache_slack_gradient.clear();
  cache_slack_gradient.reserve(static_cast<std::size_t>(cvp.n_constraint_terms * 3));

  cvp.n_slack_vars = 0;
  Eigen::Index constraint_matrix_row{ 0 };
  Eigen::Index current_var_index{ cvp.n_nlp_vars };
  Eigen::Index merit_constraint_index{ 0 };
  for (std::size_t i = 0; i < constraint_terms.size(); ++i)
  {
    const auto& info = cvp.constraint_term_infos[i].get();
    const auto& cnt = constraint_terms[i];
    if (info.rows == 0)
      continue;

    // Linearize Constraints
    const trajopt_ifopt::Jacobian jac = cnt->getJacobian();

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

    const double merit_coeff =
        (info.type == ComponentInfoType::kMeritConstraint) ? constraint_merit_coeff(merit_constraint_index++) : 1;
    for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
    {
      for (trajopt_ifopt::Jacobian::InnerIterator it(jac, k); it; ++it)
      {
        // Originally it pruned these but it changes sparsity so we now set to zero
        if (std::abs(it.value()) < 1e-7)
          cache_triplets_2.emplace_back(constraint_matrix_row + it.row(), it.col(), 0.0);
        else
          cache_triplets_2.emplace_back(constraint_matrix_row + it.row(), it.col(), it.value());
      }

      ///////////////////////////////
      // Update NLP Constraint Bounds
      ///////////////////////////////
      const auto& cnt_bound = info.bounds[static_cast<std::size_t>(k)];
      const auto cnt_bound_type = cnt_bound.getType();
      const Eigen::Index row = constraint_matrix_row + k;
      const double constant = cvp.constraint_constant[row];
      cvp.bounds_lower[row] = cnt_bound.getLower() - constant;
      cvp.bounds_upper[row] = cnt_bound.getUpper() - constant;

      //////////////////////////////////////////////////////////
      // Set the slack variables constraint matrix and gradient
      //////////////////////////////////////////////////////////

      assert(info.type != ComponentInfoType::kObjectiveSquared);
      const double coeff = merit_coeff * info.coeffs(k);
      if (cnt_bound_type == trajopt_ifopt::BoundsType::kEquality)
      {
        assert(info.type != ComponentInfoType::kPenaltyHinge);
        cache_slack_gradient.emplace_back(coeff);
        cache_slack_gradient.emplace_back(coeff);
        cache_triplets_2.emplace_back(row, current_var_index++, 1.0);
        cache_triplets_2.emplace_back(row, current_var_index++, -1.0);
        cvp.n_slack_vars += 2;
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::kLowerBound)
      {
        assert(info.type != ComponentInfoType::kPenaltyAbsolute);
        cache_slack_gradient.emplace_back(coeff);
        cache_triplets_2.emplace_back(row, current_var_index++, 1.0);
        ++cvp.n_slack_vars;
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::kUpperBound)
      {
        assert(info.type != ComponentInfoType::kPenaltyAbsolute);
        cache_slack_gradient.emplace_back(coeff);
        cache_triplets_2.emplace_back(row, current_var_index++, -1.0);
        ++cvp.n_slack_vars;
      }
      else
      {
        throw std::runtime_error("Unsupported bounds type!");
      }
    }

    constraint_matrix_row += info.rows;
  }

  cvp.num_qp_vars = cvp.n_nlp_vars + cvp.n_slack_vars;
  cvp.num_qp_cnts = cvp.n_penalty_constraints + cvp.n_merit_constraints + cvp.num_qp_vars;

  // Initialize the constraint bounds
  // We default to slack variable bounds to avoid having to set those seperatly
  cvp.bounds_lower.conservativeResize(cvp.num_qp_cnts);
  cvp.bounds_upper.conservativeResize(cvp.num_qp_cnts);
  cvp.bounds_lower.tail(cvp.n_slack_vars).setConstant(0.0);
  cvp.bounds_upper.tail(cvp.n_slack_vars).setConstant(double(INFINITY));

  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  cvp.hessian.resize(cvp.num_qp_vars, cvp.num_qp_vars);
  cvp.hessian.setZero();

  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  cvp.gradient.resize(cvp.num_qp_vars);
  cvp.gradient.setZero();
  cvp.gradient.tail(cvp.n_slack_vars) = Eigen::Map<Eigen::VectorXd>(cache_slack_gradient.data(), cvp.n_slack_vars);

  // Create triplet list of nonzero gradients
  cache_triplets_1.clear();
  cache_triplets_1.reserve(static_cast<std::size_t>(cvp.n_objective_term_non_zeros));

  // Process Squared Costs
  /** @note See CostFromFunc::convex in modeling_utils.cpp. */
  if (cvp.n_objective_terms > 0)
  {
    cvp.squared_objective_nlp.constants.setZero(cvp.n_objective_terms);
    cvp.squared_objective_nlp.linear_coeffs.resize(cvp.n_objective_terms, cvp.n_nlp_vars);
    cvp.squared_objective_nlp.linear_coeffs.setZero();
    cvp.squared_objective_nlp.objective_linear_coeffs.setZero(cvp.n_nlp_vars);
    cvp.squared_objective_nlp.objective_quadratic_coeffs.resize(cvp.n_nlp_vars, cvp.n_nlp_vars);
    cvp.squared_objective_nlp.objective_quadratic_coeffs.setZero();
    cvp.squared_objective_nlp.quadratic_coeffs.resize(static_cast<std::size_t>(cvp.n_objective_terms));

    Eigen::Index row = 0;
    bool has_obj_quad = false;
    for (std::size_t i = 0; i < objective_terms.size(); ++i)
    {
      const auto& info = cvp.objective_term_infos[i];
      const auto& obj = objective_terms[i];

      Eigen::Index idx{ row };
      for (const auto& b : info.bounds)
        cvp.squared_objective_target(idx++) = b.getLower();

      // This is not correct should pass the value to createAffExprs then use bound to which could change the sign of
      // the affine expression
      //    Eigen::VectorXd cnt_error = trajopt_ifopt::calcBoundsErrors(cnt_vals, squared_costs_.getBounds());

      // This should be correct now
      cache_aff_expr.create(obj->getValues(), obj->getJacobian(), x_initial);
      cache_aff_expr.constants = (cvp.squared_objective_target.segment(row, obj->getRows()) - cache_aff_expr.constants);
      cache_aff_expr.linear_coeffs *= -1;
      cache_aff_expr.square(cache_quad_expr, obj->getCoefficients());

      // Update has objective quad
      has_obj_quad |= (cache_quad_expr.objective_quadratic_coeffs.nonZeros() > 0);

      // store individual equations constant
      cvp.squared_objective_nlp.constants.segment(row, obj->getRows()) = cache_quad_expr.constants;

      // Sum objective function linear coefficients
      cvp.squared_objective_nlp.objective_linear_coeffs += cache_quad_expr.objective_linear_coeffs;

      // Sum objective function quadratic coefficients
      cvp.squared_objective_nlp.objective_quadratic_coeffs += cache_quad_expr.objective_quadratic_coeffs;

      // store individual equations linear coefficients in a Triplet list to update equation linear coefficients later
      if (cache_quad_expr.linear_coeffs.nonZeros() > 0)
      {
        // Add jacobian to triplet list
        for (int k = 0; k < cache_quad_expr.linear_coeffs.outerSize(); ++k)
        {
          for (trajopt_ifopt::Jacobian::InnerIterator it(cache_quad_expr.linear_coeffs, k); it; ++it)
            cache_triplets_1.emplace_back(row + it.row(), it.col(), it.value());
        }
      }

      // Store individual equations quadratic coefficients
      assert(cache_quad_expr.quadratic_coeffs.size() == obj->getRows());
      for (std::size_t j = 0; j < cache_quad_expr.quadratic_coeffs.size(); ++j)
        cvp.squared_objective_nlp.quadratic_coeffs[static_cast<std::size_t>(row) + j] =
            cache_quad_expr.quadratic_coeffs[j];

      row += obj->getRows();
    }

    // Store individual equations linear coefficients
    cvp.squared_objective_nlp.linear_coeffs.setFromTriplets(cache_triplets_1.begin(),
                                                            cache_triplets_1.end());  // NOLINT

    // Insert QP Problem Objective Linear Coefficients
    cvp.gradient.head(cvp.n_nlp_vars) = cvp.squared_objective_nlp.objective_linear_coeffs;

    // Insert QP Problem Objective Quadratic Coefficients
    if (has_obj_quad)
    {
      cvp.hessian.reserve(cvp.squared_objective_nlp.objective_quadratic_coeffs.nonZeros());
      for (Eigen::Index r = 0; r < cvp.squared_objective_nlp.objective_quadratic_coeffs.outerSize(); ++r)
      {
        cvp.hessian.startVec(r);  // start row k (RowMajor)

        // RowMajor Q: r == it.row(), col is sorted, perfect for insertBack
        for (trajopt_ifopt::Jacobian::InnerIterator it(cvp.squared_objective_nlp.objective_quadratic_coeffs, r); it;
             ++it)
        {
          // Originally it pruned these but it changes sparsity so we now set to zero
          if (std::abs(it.value()) < 1e-7)
            cvp.hessian.insertBack(r, it.col()) = 0.0;
          else
            cvp.hessian.insertBack(r, it.col()) = it.value();
        }
      }
      cvp.hessian.finalize();
      cvp.hessian.makeCompressed();
    }
  }

  ////////////////////////////////////////////////////////
  // Set the slack variables constraint matrix
  ////////////////////////////////////////////////////////

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  constraint_matrix_row = cvp.n_merit_constraints + cvp.n_penalty_constraints;
  for (Eigen::Index i = 0; i < cvp.num_qp_vars; ++i)
    cache_triplets_2.emplace_back(constraint_matrix_row + i, i, 1.0);

  // Insert the triplet list into the sparse matrix
  cvp.constraint_matrix.resize(cvp.num_qp_cnts, cvp.num_qp_vars);
  cvp.constraint_matrix.reserve(cvp.n_constraint_term_non_zeros + cvp.n_slack_vars + cvp.num_qp_vars);
  cvp.constraint_matrix.setFromTriplets(cache_triplets_2.begin(), cache_triplets_2.end());
  cvp.constraint_matrix.makeCompressed();

  // Update NLP Bounds
  updateNLPVariableBounds(x_initial);
}

double TrajOptQPProblem::Implementation::getTotalExactCost() const { return getExactCosts().sum(); }

Eigen::VectorXd TrajOptQPProblem::Implementation::getExactCosts() const
{
  assert(initialized);

  if (num_cost_components == 0)
    return {};

  Eigen::VectorXd g(num_cost_components);
  Eigen::Index cost_idx{ 0 };

  for (const auto& c : objective_terms)
  {
    if (c->getRows() == 0)
    {
      g(cost_idx++) = 0;
      continue;
    }

    if (scratch_err.size() < c->getRows())
      scratch_err.resize(c->getRows());
    auto err = scratch_err.head(c->getRows());

    trajopt_ifopt::calcBoundsViolations(err, c->getValues(), c->getBounds());
    g(cost_idx++) = (err.array().square() * c->getCoefficients().array()).sum();
  }

  for (const auto& c : penalty_constraints)
  {
    if (c->getRows() == 0)
    {
      g(cost_idx++) = 0;
      continue;
    }

    if (scratch_err.size() < c->getRows())
      scratch_err.resize(c->getRows());
    auto err = scratch_err.head(c->getRows());

    trajopt_ifopt::calcBoundsViolations(err, c->getValues(), c->getBounds());
    g(cost_idx++) = err.sum();
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::Implementation::getExactConstraintViolations() const
{
  Eigen::VectorXd violations(merit_constraints.size());
  Eigen::Index cnt_idx{ 0 };

  for (const auto& c : merit_constraints)
  {
    if (c->getRows() == 0)
    {
      violations(cnt_idx++) = 0;
      continue;
    }

    if (scratch_err.size() < c->getRows())
      scratch_err.resize(c->getRows());
    auto err = scratch_err.head(c->getRows());

    trajopt_ifopt::calcBoundsViolations(err, c->getValues(), c->getBounds());
    violations(cnt_idx++) = err.sum();
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
  std::cout << "Constraint Lower Bounds: " << cvp.bounds_lower.head(cvp.n_constraint_terms).transpose().format(format)
            << '\n';
  std::cout << "Constraint Upper Bounds: " << cvp.bounds_upper.head(cvp.n_constraint_terms).transpose().format(format)
            << '\n';
  std::cout << "Variable Lower Bounds: "
            << cvp.bounds_lower.tail(cvp.bounds_lower.rows() - cvp.n_constraint_terms).transpose().format(format)
            << '\n';
  std::cout << "Variable Upper Bounds: "
            << cvp.bounds_upper.tail(cvp.bounds_upper.rows() - cvp.n_constraint_terms).transpose().format(format)
            << '\n';
  std::cout << "All Lower Bounds: " << cvp.bounds_lower.transpose().format(format) << '\n';
  std::cout << "All Upper Bounds: " << cvp.bounds_upper.transpose().format(format) << '\n';
  std::cout << "NLP values: " << '\n' << variables->getValues().transpose().format(format) << '\n';
}

void TrajOptQPProblem::Implementation::updateNLPVariableBounds(const Eigen::Ref<const Eigen::VectorXd>& nlp_values)
{
  // Equivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  const Eigen::Index idx = cvp.n_merit_constraints + cvp.n_penalty_constraints;

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

TrajOptQPProblem::TrajOptQPProblem(std::shared_ptr<trajopt_ifopt::Variables> variables)
  : impl_(std::make_unique<Implementation>(std::move(variables)))
{
}

TrajOptQPProblem::~TrajOptQPProblem() = default;

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

double TrajOptQPProblem::evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  return evaluateConvexCosts(var_vals).sum();
}

Eigen::VectorXd TrajOptQPProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  return std::as_const<Implementation>(*impl_).cvp.evaluateConvexCosts(var_vals);
}

double TrajOptQPProblem::getTotalExactCost() const { return std::as_const<Implementation>(*impl_).getTotalExactCost(); }

Eigen::VectorXd TrajOptQPProblem::getExactCosts() const
{
  return std::as_const<Implementation>(*impl_).getExactCosts();
}

Eigen::VectorXd
TrajOptQPProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  return std::as_const<Implementation>(*impl_).cvp.evaluateConvexConstraintViolations(var_vals);
}

Eigen::VectorXd TrajOptQPProblem::getExactConstraintViolations() const
{
  return std::as_const<Implementation>(*impl_).getExactConstraintViolations();  // NOLINT
}

void TrajOptQPProblem::scaleBoxSize(double& scale) { impl_->scaleBoxSize(scale); }

void TrajOptQPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) { impl_->setBoxSize(box_size); }

void TrajOptQPProblem::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  impl_->setConstraintMeritCoeff(merit_coeff);
}

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

const Eigen::VectorXd& TrajOptQPProblem::getBoxSize() const { return impl_->box_size; }
const Eigen::VectorXd& TrajOptQPProblem::getConstraintMeritCoeff() const { return impl_->constraint_merit_coeff; }

const trajopt_ifopt::Jacobian& TrajOptQPProblem::getHessian() const { return impl_->cvp.hessian; }
const Eigen::VectorXd& TrajOptQPProblem::getGradient() const { return impl_->cvp.gradient; }

const trajopt_ifopt::Jacobian& TrajOptQPProblem::getConstraintMatrix() const { return impl_->cvp.constraint_matrix; }
const Eigen::VectorXd& TrajOptQPProblem::getBoundsLower() const { return impl_->cvp.bounds_lower; }
const Eigen::VectorXd& TrajOptQPProblem::getBoundsUpper() const { return impl_->cvp.bounds_upper; }

}  // namespace trajopt_sqp
