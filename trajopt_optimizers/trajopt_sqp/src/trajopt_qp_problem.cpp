
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/expressions.h>
#include <trajopt_sqp/types.h>

#include <algorithm>
#include <tuple>
#include <utility>
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

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
 * QP Variables: |NLP Vars, Hinge Cnt Cost Slack Variable, Absolute Cnt Cost Slack Variable, NLP Constraint Slack Vars,
 *                Squared Cost One-Sided Row Slack Vars |
 *
 * Constraint Matrix
 * | Hinge Cost Cnt Jacobian           , Hinge cost cnt slack variable jacobian            |
 * | Absolute Cost Cnt Jacobian        , Absolute cost cnt slack variable jacobian         |
 * | NLP constraint jacobian           , NLP constraint slack variable jacobian            |
 * | Squared cost one-sided row jacobian, Squared cost one-sided row slack variable jacobian |
 * |        QP Variable Jacobian (Diag Matrix of ones)                                     |
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
namespace
{
/**
 * @brief Sum @p values times their row weights @p w, leaving out rows of weight exactly 0.
 * @details A row of weight 0 is disabled, so it adds nothing even when its value is infinite.
 */
template <typename Values>
double weightedSum(const Eigen::ArrayBase<Values>& values, const Eigen::Ref<const Eigen::VectorXd>& w)
{
  return (w.array() != 0.0).select(values * w.array(), 0.0).sum();
}

/**
 * @brief Write one constraint set's violation into slot @p idx of @p out.
 * @details `raw` is the sum of the row violations if @p sum, else the largest of them, and `weighted` sums them
 * times their weights, both leaving out rows of weight exactly 0. @p err must not be empty.
 */
void reduceSetViolation(ConstraintViolations& out,
                        Eigen::Index idx,
                        const Eigen::Ref<const Eigen::VectorXd>& err,
                        const Eigen::Ref<const Eigen::VectorXd>& w,
                        bool sum)
{
  assert(err.size() > 0);
  const auto enabled = (w.array() != 0.0).select(err.array(), 0.0);
  out.raw(idx) = sum ? enabled.sum() : enabled.maxCoeff();
  out.weighted(idx) = weightedSum(err.array(), w);
}

/** @brief Entries of a linearized row up to this magnitude are treated as zero, as trajopt_sco's cleanupAff does. */
constexpr double SMALL_ENTRY_THRESHOLD = 1e-7;

/**
 * @brief Set the entries of @p jac up to the small-entry threshold to zero.
 * @details The entries stay stored, so the sparsity pattern of this problem's matrices does not depend on their values.
 */
void zeroSmallEntries(trajopt_ifopt::Jacobian& jac)
{
  for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
  {
    for (trajopt_ifopt::Jacobian::InnerIterator it(jac, k); it; ++it)
    {
      if (std::abs(it.value()) <= SMALL_ENTRY_THRESHOLD)
        it.valueRef() = 0.0;
    }
  }
}

/** @brief Whether a slack can model the row: it is an equality or bounded on one side only. */
bool isEqualityOrOneSided(const trajopt_ifopt::Bounds& bound)
{
  const trajopt_ifopt::BoundsType type = bound.getType();
  return type == trajopt_ifopt::BoundsType::kEquality || type == trajopt_ifopt::BoundsType::kLowerBound ||
         type == trajopt_ifopt::BoundsType::kUpperBound;
}

/** @brief Throw unless a slack can model every row of the set. */
void checkRowBounds(const std::vector<trajopt_ifopt::Bounds>& bounds, const std::string& name)
{
  if (!std::all_of(bounds.begin(), bounds.end(), isEqualityOrOneSided))
    throw std::runtime_error("TrajOpt Ifopt set '" + name + "' rows must have equality or one-sided bounds!");
}

}  // namespace

enum class ComponentInfoType : std::uint8_t
{
  kObjectiveSquared = 0,
  kPenaltyLinear,
  kMeritConstraint,
  kUnknown
};

struct ComponentInfo
{
  ComponentInfoType type{ ComponentInfoType::kUnknown };
  Eigen::Index rows{ 0 };
  Eigen::Index non_zeros{ 0 };
  /** @brief Per-row weights read at the most recent convexify(); not filled in objective_term_infos */
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
  Eigen::Index n_constraint_terms{ 0 };  // (n_penalty_constraints + n_merit_constraints + n_objective_slack_rows)
  Eigen::Index n_penalty_constraints{ 0 };
  Eigen::Index n_merit_constraints{ 0 };
  /** @brief One-sided rows of the objective terms; each owns a QP row below the merit rows, and a slack */
  Eigen::Index n_objective_slack_rows{ 0 };

  Eigen::Index n_objective_term_non_zeros{ 0 };
  Eigen::Index n_constraint_term_non_zeros{ 0 };
  Eigen::Index n_penalty_constraint_non_zeros{ 0 };
  Eigen::Index n_merit_constraint_non_zeros{ 0 };

  Eigen::Index num_qp_vars{ 0 };
  Eigen::Index num_qp_cnts{ 0 };

  std::vector<ComponentInfo> objective_term_infos;
  /** @brief One entry per objective term: the count, bounds and weights of its one-sided rows, in row order */
  std::vector<ComponentInfo> objective_slack_infos;
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

  /**
   * @brief Evaluate the bounds violations of the slack-free linear model's rows [row_offset, row_offset + c_info.rows).
   * @return A view into scratch_err, valid until the next call
   */
  Eigen::Ref<const Eigen::VectorXd> linearizedViolations(const Eigen::Ref<const Eigen::VectorXd>& var_block,
                                                         Eigen::Index row_offset,
                                                         const ComponentInfo& c_info) const
  {
    auto jac = constraint_matrix.middleRows(row_offset, c_info.rows).leftCols(n_nlp_vars);
    auto constant = constraint_constant.segment(row_offset, c_info.rows);

    // Ensure scratch buffers big enough (no allocation after first growth)
    if (scratch_val.size() < c_info.rows)
      scratch_val.resize(c_info.rows);
    if (scratch_err.size() < c_info.rows)
      scratch_err.resize(c_info.rows);

    auto val = scratch_val.head(c_info.rows);
    auto err = scratch_err.head(c_info.rows);

    // scratch_val = constant + jac * var_block
    val = constant;                    // copy into scratch (but no alloc)
    val.noalias() += jac * var_block;  // mat-vec into existing memory

    // compute violations in-place
    trajopt_ifopt::calcBoundsViolations(err, val, c_info.bounds);

    assert(!(err.array() < -1e-8).any());
    return err;
  }

  ConstraintViolations evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals,
                                                          bool sum) const;
};

Eigen::VectorXd ConvexProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  /** @note Penalty rows are evaluated on the slack-free linear model: at a QP solution each row's slack
   *  absorbs its violation, so including the slack columns would report zero. The result depends only on
   *  the NLP-variable block of var_vals. */
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

  // One-sided rows of the squared costs are charged their weight times the squared violation of the linearized row
  row_offset = n_penalty_constraints + n_merit_constraints;
  for (std::size_t i = 0; i < objective_slack_infos.size(); ++i)
  {
    const auto& s_info = objective_slack_infos[i];
    if (s_info.rows == 0)
      continue;

    const auto err = linearizedViolations(var_block, row_offset, s_info);
    costs(static_cast<Eigen::Index>(i)) += weightedSum(err.array().square(), s_info.coeffs);
    row_offset += s_info.rows;
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

    const auto err = linearizedViolations(var_block, row_offset, c_info);
    costs(cost_idx++) = weightedSum(err.array(), c_info.coeffs);
    row_offset += c_info.rows;
  }

  return costs;
}

ConstraintViolations
ConvexProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals, bool sum) const
{
  /** @note Merit rows are evaluated on the slack-free linear model, as penalty rows are in evaluateConvexCosts(). */
  const auto n = static_cast<Eigen::Index>(merit_constraint_infos.size());
  ConstraintViolations result{ Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n) };
  Eigen::Index cnt_idx{ 0 };
  Eigen::Index row_index{ n_penalty_constraints };

  Eigen::Ref<const Eigen::VectorXd> var_block = var_vals.head(n_nlp_vars);
  for (const auto& c_info : merit_constraint_infos)
  {
    if (c_info.rows == 0)
    {
      ++cnt_idx;
      continue;
    }

    reduceSetViolation(result, cnt_idx++, linearizedViolations(var_block, row_index, c_info), c_info.coeffs, sum);
    row_index += c_info.rows;
  }

  return result;
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
   * @brief The squared-cost terms.
   * @details
   * An equality row is convexified into the QP objective (it contributes to the Hessian/gradient directly) and
   * adds no QP constraint row. A one-sided row adds one QP constraint row and one slack variable, and its weight
   * goes on that slack's Hessian diagonal.
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
   * @brief Convenience container for the terms whose every row is a QP constraint row.
   * @details
   * The one-sided rows of @ref objective_terms add QP constraint rows too, below the rows of these terms; they
   * are not part of this container.
   *
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

  /** @brief Whether a constraint set's raw violation is the sum of its rows, or else the largest of them */
  bool sum_constraint_set_violations{ true };

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

  /** @brief Whether convexify() reduces the QP handed to the solver, see buildReducedQP() */
  bool qp_row_reduction{ false };

  /**
   * @brief The QP handed to the solver, without the one-sided linear-slack rows that cannot change sign in the box
   * @details Valid while active; the full problem in cvp stays the model the merit is evaluated on. Its variables
   * are the NLP variables followed by one slack per kept row slack. Its dimensions change only in convexify(),
   * because the solver loads a QP once per convexification and afterwards only updates its bounds.
   */
  struct ReducedQP
  {
    bool active{ false };
    Eigen::Index n_rows{ 0 };
    Eigen::Index num_vars{ 0 };
    Eigen::Index num_cnts{ 0 };
    trajopt_ifopt::Jacobian hessian;
    trajopt_ifopt::Jacobian constraint_matrix;
    Eigen::VectorXd gradient;
    Eigen::VectorXd bounds_lower;
    Eigen::VectorXd bounds_upper;
    /** @brief The trust box the QP was built for; its NLP variable rows never exceed it */
    Eigen::VectorXd box_lower;
    Eigen::VectorXd box_upper;
  } reduced;

  /** @brief One row of cvp's constraint matrix as buildReducedQP() classifies it */
  struct ReductionRow
  {
    /** @brief The row's entries in the constraint matrix arrays, NLP columns first */
    Eigen::Index begin{ 0 };
    Eigen::Index n_nlp{ 0 };
    /** @brief +1 for a row a·x - s <= ub, -1 for a row a·x + s >= lb, which is -a·x - s <= -lb */
    double sign{ 1 };
    /** @brief The bound of the row in the a·x - s <= ub form */
    double ub{ 0 };
    double w{ 0 };
    double norm{ 0 };
    /** @brief The leader this row is merged into, or the row itself */
    Eigen::Index leader{ -1 };
    /** @brief The slack weight of a leader: its own plus that of every row merged into it, scaled to its row */
    double group_w{ 0 };
    enum class Fate : std::uint8_t
    {
      kCopy,
      kKeep,
      kDrop,
      kFold,
    } fate{ Fate::kCopy };
  };
  std::vector<ReductionRow> reduction_rows;
  /** @brief A row that can merge, ordered so that the rows it can merge with are its neighbours */
  struct MergeCandidate
  {
    /** @brief A hash of the row's NLP columns: equal for rows with equal columns */
    std::uint64_t columns{ 0 };
    /** @brief The bound of the normalized row, ub / norm */
    double t{ 0 };
    Eigen::Index row{ 0 };
    bool operator<(const MergeCandidate& other) const
    {
      return std::tie(columns, t, row) < std::tie(other.columns, other.t, other.row);
    }
  };
  std::vector<MergeCandidate> reduction_order;
  /**
   * @brief Per row of cvp's constraint matrix, set by convexify(): +1 for a·x - s <= ub and -1 for a·x + s >= lb
   * with one linear slack s, 0 for any other row
   */
  std::vector<std::int8_t> linear_slack_row_sign;

  /**
   * @brief Build the reduced QP from cvp, if qp_row_reduction
   * @details A row is eligible if it is one-sided, a·x - s <= ub or a·x + s >= lb, with one linear slack (a hinge cost
   * or NLP constraint row, not a row of a squared cost) of weight w > 0; every other row is kept as it is. Write an
   * eligible row as a·x - s <= ub, negating a lower-bounded one. With x0 the current values and the trust box [l, u]
   * the QP's NLP variable rows (centred on x0 clamped to its limits), let r0 = a·x0 - ub and
   * d+ / d- = the maximum / minimum of a·(x - x0) over the box. A row with r0 + d+ <= 0 cannot become violated and is
   * dropped. A row with r0 + d- >= 0 cannot become satisfied; its slack equals a·x - ub everywhere in the box, so it
   * is folded into the gradient as w·a. Both tests keep a relative margin for rounding. The other rows are kept,
   * and among those, rows with the same sparsity whose rows and bounds are positive multiples of one another
   * (to 1e-12 after normalizing) become one of them, row L, with the weight Sum_m w_m |a_m| / |a_L|.
   * Each eligible slack appears in one row only, so minimizing the full QP over its slacks leaves
   * Sum w max(0, a·x - ub), which on the box equals the reduced QP's objective plus a constant: the two have the
   * same minimizers in x and the predicted improvement is the full QP's. The QP stays the full one if nothing is
   * reduced.
   * @param x0 The variable values cvp was convexified at
   */
  void buildReducedQP(const Eigen::VectorXd& x0);

  /**
   * @brief Copy the trust-box rows of cvp into the reduced QP, if it is active
   * @details The NLP variable rows are clipped to the box the reduced QP was built for, so a box grown without a
   * convexify() shrinks the trust region instead of reaching rows the reduction left out. The clipped box is empty
   * only if the variables were moved outside that box without a convexify().
   */
  void syncReducedBounds();

  /** @brief The QP handed to the solver: the reduced QP while it is active, the full one in cvp otherwise */
  struct ServedQP
  {
    /** @brief The linearized rows, which sit above one bound row per QP variable */
    Eigen::Index n_rows;
    Eigen::Index num_vars;
    Eigen::Index num_cnts;
    const trajopt_ifopt::Jacobian& hessian;
    const trajopt_ifopt::Jacobian& constraint_matrix;
    const Eigen::VectorXd& gradient;
    const Eigen::VectorXd& bounds_lower;
    const Eigen::VectorXd& bounds_upper;
  };
  ServedQP servedQP() const
  {
    if (reduced.active)
      return { reduced.n_rows,   reduced.num_vars,     reduced.num_cnts,    reduced.hessian, reduced.constraint_matrix,
               reduced.gradient, reduced.bounds_lower, reduced.bounds_upper };
    return { cvp.n_constraint_terms, cvp.num_qp_vars, cvp.num_qp_cnts,  cvp.hessian,
             cvp.constraint_matrix,  cvp.gradient,    cvp.bounds_lower, cvp.bounds_upper };
  }

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

  ConstraintViolations getExactConstraintViolations() const;

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
  checkRowBounds(constraint_set->getBounds(), constraint_set->getName());

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
  checkRowBounds(constraint_set->getBounds(), constraint_set->getName());

  switch (penalty_type)
  {
    case CostPenaltyType::kSquared:
    {
      if (constraint_set->isDynamic())
        dyn_squared_costs.emplace_back(std::move(constraint_set));
      else
        squared_costs.emplace_back(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::kAbsolute:
    {
      if (constraint_set->isDynamic())
        dyn_abs_costs.emplace_back(std::move(constraint_set));
      else
        abs_costs.emplace_back(std::move(constraint_set));

      break;
    }
    case CostPenaltyType::kHinge:
    {
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
  cvp.n_objective_slack_rows = 0;

  cvp.n_objective_term_non_zeros = 0;
  cvp.n_constraint_term_non_zeros = 0;
  cvp.n_penalty_constraint_non_zeros = 0;
  cvp.n_merit_constraint_non_zeros = 0;

  Eigen::Index n_objective_slack_non_zeros{ 0 };
  for (std::size_t i = 0; i < objective_terms.size(); ++i)
  {
    const auto& cost = objective_terms[i];
    auto& slack_info = cvp.objective_slack_infos[i];
    if (initialized && !cost->isDynamic())
    {
      cvp.n_objective_terms += cost->getRows();
      cvp.n_objective_term_non_zeros += cost->getNonZeros();
      cvp.n_objective_slack_rows += slack_info.rows;
      n_objective_slack_non_zeros += slack_info.non_zeros;
      continue;
    }

    auto& info = cvp.objective_term_infos[i];
    info.rows = cost->getRows();
    info.non_zeros = cost->getNonZeros();
    info.bounds = cost->getBounds();
    checkRowBounds(info.bounds, cost->getName());

    slack_info.bounds.clear();
    for (const auto& b : info.bounds)
    {
      if (b.getType() != trajopt_ifopt::BoundsType::kEquality)
        slack_info.bounds.push_back(b);
    }
    slack_info.rows = static_cast<Eigen::Index>(slack_info.bounds.size());
    // An upper bound: the non-zeros of the whole set, its equality rows included
    slack_info.non_zeros = (slack_info.rows > 0) ? info.non_zeros : 0;

    cvp.n_objective_terms += info.rows;
    cvp.n_objective_term_non_zeros += info.non_zeros;
    cvp.n_objective_slack_rows += slack_info.rows;
    n_objective_slack_non_zeros += slack_info.non_zeros;
  }

  // Each row of a linear-penalty cost adds a QP row, with one slack if it is one-sided and two if it is an equality
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
    info.bounds = cost->getBounds();
    checkRowBounds(info.bounds, cost->getName());

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
    info.bounds = cnt->getBounds();
    checkRowBounds(info.bounds, cnt->getName());

    cvp.n_merit_constraints += info.rows;
    cvp.n_merit_constraint_non_zeros += info.non_zeros;
  }

  cvp.n_constraint_terms = cvp.n_penalty_constraints + cvp.n_merit_constraints + cvp.n_objective_slack_rows;
  cvp.n_constraint_term_non_zeros =
      cvp.n_penalty_constraint_non_zeros + cvp.n_merit_constraint_non_zeros + n_objective_slack_non_zeros;

  cvp.squared_objective_target.setZero(cvp.n_objective_terms);
  cvp.constraint_constant.setZero(cvp.n_constraint_terms);

  cvp.bounds_lower.resize(cvp.n_nlp_vars + cvp.n_constraint_terms);
  cvp.bounds_upper.resize(cvp.n_nlp_vars + cvp.n_constraint_terms);
}

void TrajOptQPProblem::Implementation::setup()
{
  reduced.active = false;
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

  cvp.objective_slack_infos.clear();
  cvp.objective_slack_infos.resize(objective_terms.size());

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
    info.type = ComponentInfoType::kPenaltyLinear;
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

  // The infos above were cleared, so update() has to fill them for the fixed-size sets too
  initialized = false;
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
  reduced.active = false;

  // Update if dynamic constraints are present
  update();

  // Get current variable values
  const Eigen::VectorXd x_initial = variables->getValues();

  // Convexify

  /** Use cache triplet and clear */
  cache_triplets_2.clear();
  cache_triplets_2.reserve(
      static_cast<std::size_t>(cvp.n_constraint_term_non_zeros + (cvp.n_constraint_terms * 3) + cvp.n_nlp_vars));
  cache_slack_gradient.clear();
  cache_slack_gradient.reserve(static_cast<std::size_t>(cvp.n_constraint_terms * 3));

  linear_slack_row_sign.assign(static_cast<std::size_t>(cvp.n_constraint_terms), 0);
  cvp.n_slack_vars = 0;
  Eigen::Index constraint_matrix_row{ 0 };
  Eigen::Index current_var_index{ cvp.n_nlp_vars };
  Eigen::Index merit_constraint_index{ 0 };
  for (std::size_t i = 0; i < constraint_terms.size(); ++i)
  {
    auto& info = cvp.constraint_term_infos[i].get();
    const auto& cnt = constraint_terms[i];

    // Every merit set owns one merit-coefficient slot, including a set that currently has no rows.
    const double merit_coeff =
        (info.type == ComponentInfoType::kMeritConstraint) ? constraint_merit_coeff(merit_constraint_index++) : 1;
    if (info.rows == 0)
      continue;

    // Weights are read at the linearization point, so the model agrees with the exact merit there.
    info.coeffs = cnt->getCoefficients();

    // Linearize Constraints. Small entries are zeroed before the constant below is taken from the rows, so the
    // linear model reproduces the constraint values at x_initial.
    trajopt_ifopt::Jacobian jac = cnt->getJacobian();
    zeroSmallEntries(jac);

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

    for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
    {
      for (trajopt_ifopt::Jacobian::InnerIterator it(jac, k); it; ++it)
        cache_triplets_2.emplace_back(constraint_matrix_row + it.row(), it.col(), it.value());

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
        cache_slack_gradient.emplace_back(coeff);
        cache_slack_gradient.emplace_back(coeff);
        cache_triplets_2.emplace_back(row, current_var_index++, 1.0);
        cache_triplets_2.emplace_back(row, current_var_index++, -1.0);
        cvp.n_slack_vars += 2;
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::kLowerBound)
      {
        cache_slack_gradient.emplace_back(coeff);
        cache_triplets_2.emplace_back(row, current_var_index++, 1.0);
        linear_slack_row_sign[static_cast<std::size_t>(row)] = -1;
        ++cvp.n_slack_vars;
      }
      else if (cnt_bound_type == trajopt_ifopt::BoundsType::kUpperBound)
      {
        cache_slack_gradient.emplace_back(coeff);
        cache_triplets_2.emplace_back(row, current_var_index++, -1.0);
        linear_slack_row_sign[static_cast<std::size_t>(row)] = 1;
        ++cvp.n_slack_vars;
      }
      else
      {
        throw std::runtime_error("Unsupported bounds type!");
      }
    }

    constraint_matrix_row += info.rows;
  }

  // Each one-sided row of an objective term takes the next QP row and the next slack, below. Its slack is
  // charged in the Hessian, so its gradient entry stays zero.
  cvp.n_slack_vars += cvp.n_objective_slack_rows;
  cache_slack_gradient.insert(cache_slack_gradient.end(), static_cast<std::size_t>(cvp.n_objective_slack_rows), 0.0);

  cvp.num_qp_vars = cvp.n_nlp_vars + cvp.n_slack_vars;
  cvp.num_qp_cnts = cvp.n_constraint_terms + cvp.num_qp_vars;

  // Initialize the constraint bounds
  // We default to slack variable bounds to avoid having to set those seperatly
  cvp.bounds_lower.conservativeResize(cvp.num_qp_cnts);
  cvp.bounds_upper.conservativeResize(cvp.num_qp_cnts);
  cvp.bounds_lower.tail(cvp.n_slack_vars).setConstant(0.0);
  cvp.bounds_upper.tail(cvp.n_slack_vars).setConstant(double(INFINITY));

  ////////////////////////////////////////////////////////
  // Initialize Hessian — populated below from squared cost terms
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
  // Sized without objective rows too, so the expression never holds the rows of an earlier convexification
  cvp.squared_objective_nlp.constants.setZero(cvp.n_objective_terms);
  cvp.squared_objective_nlp.linear_coeffs.resize(cvp.n_objective_terms, cvp.n_nlp_vars);
  cvp.squared_objective_nlp.linear_coeffs.setZero();
  cvp.squared_objective_nlp.objective_linear_coeffs.setZero(cvp.n_nlp_vars);
  cvp.squared_objective_nlp.objective_quadratic_coeffs.resize(cvp.n_nlp_vars, cvp.n_nlp_vars);
  cvp.squared_objective_nlp.objective_quadratic_coeffs.setZero();
  cvp.squared_objective_nlp.quadratic_coeffs.resize(static_cast<std::size_t>(cvp.n_objective_terms));

  if (cvp.n_objective_terms > 0)
  {
    Eigen::Index row = 0;
    bool has_obj_quad = false;
    for (std::size_t i = 0; i < objective_terms.size(); ++i)
    {
      const auto& info = cvp.objective_term_infos[i];
      auto& slack_info = cvp.objective_slack_infos[i];
      const auto& obj = objective_terms[i];

      trajopt_ifopt::Jacobian jac = obj->getJacobian();
      // Filter the linearized rows and not their square: a product of retained entries may itself be small.
      // The rows are filtered before the affine model is built, so it reproduces the cost values at x_initial.
      zeroSmallEntries(jac);
      cache_aff_expr.create(obj->getValues(), jac, x_initial);
      const Eigen::VectorXd weights = obj->getCoefficients();

      // An equality row is charged in the objective, its weight times its squared distance from its target. A
      // one-sided row is charged its weight times its squared violation: its slack absorbs the violation of the
      // linearized row and carries the weight on its Hessian diagonal, and the row leaves the objective expression.
      slack_info.coeffs.resize(slack_info.rows);
      Eigen::Index slack_idx{ 0 };
      for (Eigen::Index r = 0; r < info.rows; ++r)
      {
        const auto& b = info.bounds[static_cast<std::size_t>(r)];
        const bool is_equality = (b.getType() == trajopt_ifopt::BoundsType::kEquality);
        cvp.squared_objective_target(row + r) = is_equality ? b.getLower() : 0.0;
        if (is_equality)
          continue;

        for (trajopt_ifopt::Jacobian::InnerIterator it(cache_aff_expr.linear_coeffs, r); it; ++it)
          cache_triplets_2.emplace_back(constraint_matrix_row, it.col(), it.value());

        const double constant = cache_aff_expr.constants(r);
        cvp.constraint_constant(constraint_matrix_row) = constant;
        cvp.bounds_lower(constraint_matrix_row) = b.getLower() - constant;
        cvp.bounds_upper(constraint_matrix_row) = b.getUpper() - constant;

        const double slack_sign = (b.getType() == trajopt_ifopt::BoundsType::kLowerBound) ? 1.0 : -1.0;
        cache_triplets_2.emplace_back(constraint_matrix_row, current_var_index++, slack_sign);
        ++constraint_matrix_row;

        slack_info.coeffs(slack_idx++) = weights(r);
        cache_aff_expr.constants(r) = 0.0;
      }
      // A set without equality rows adds nothing to the objective expression
      if (slack_info.rows == info.rows)
      {
        for (Eigen::Index r = 0; r < info.rows; ++r)
          cvp.squared_objective_nlp.quadratic_coeffs[static_cast<std::size_t>(row + r)].resize(0, 0);
        row += info.rows;
        continue;
      }

      if (slack_info.rows > 0)
      {
        cache_aff_expr.linear_coeffs.prune([&info](Eigen::Index r, Eigen::Index /*col*/, double /*value*/) {
          return info.bounds[static_cast<std::size_t>(r)].getType() == trajopt_ifopt::BoundsType::kEquality;
        });
      }

      cache_aff_expr.constants = (cvp.squared_objective_target.segment(row, obj->getRows()) - cache_aff_expr.constants);
      cache_aff_expr.linear_coeffs *= -1;
      cache_aff_expr.square(cache_quad_expr, weights);

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
      assert(cache_quad_expr.quadratic_coeffs.size() == static_cast<std::size_t>(obj->getRows()));
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
    if (has_obj_quad || cvp.n_objective_slack_rows > 0)
    {
      cvp.hessian.reserve(cvp.squared_objective_nlp.objective_quadratic_coeffs.nonZeros() + cvp.n_objective_slack_rows);
      for (Eigen::Index r = 0; r < cvp.squared_objective_nlp.objective_quadratic_coeffs.outerSize(); ++r)
      {
        cvp.hessian.startVec(r);  // start row k (RowMajor)

        // RowMajor Q: r == it.row(), col is sorted, perfect for insertBack
        for (trajopt_ifopt::Jacobian::InnerIterator it(cvp.squared_objective_nlp.objective_quadratic_coeffs, r); it;
             ++it)
          cvp.hessian.insertBack(r, it.col()) = it.value();
      }

      // The slacks of the one-sided rows are the last QP variables; each carries its row's weight on its diagonal
      if (cvp.n_objective_slack_rows > 0)
      {
        Eigen::Index r = cvp.num_qp_vars - cvp.n_objective_slack_rows;
        for (Eigen::Index empty = cvp.n_nlp_vars; empty < r; ++empty)
          cvp.hessian.startVec(empty);

        for (const auto& slack_info : cvp.objective_slack_infos)
        {
          for (Eigen::Index k = 0; k < slack_info.rows; ++k, ++r)
          {
            cvp.hessian.startVec(r);
            cvp.hessian.insertBack(r, r) = slack_info.coeffs(k);
          }
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
  constraint_matrix_row = cvp.n_constraint_terms;
  for (Eigen::Index i = 0; i < cvp.num_qp_vars; ++i)
    cache_triplets_2.emplace_back(constraint_matrix_row + i, i, 1.0);

  // Insert the triplet list into the sparse matrix
  cvp.constraint_matrix.resize(cvp.num_qp_cnts, cvp.num_qp_vars);
  cvp.constraint_matrix.reserve(cvp.n_constraint_term_non_zeros + cvp.n_slack_vars + cvp.num_qp_vars);
  cvp.constraint_matrix.setFromTriplets(cache_triplets_2.begin(), cache_triplets_2.end());
  cvp.constraint_matrix.makeCompressed();

  // Update NLP Bounds
  updateNLPVariableBounds(x_initial);

  buildReducedQP(x_initial);
}

void TrajOptQPProblem::Implementation::buildReducedQP(const Eigen::VectorXd& x0)
{
  if (!qp_row_reduction)
    return;

  using Fate = ReductionRow::Fate;
  static constexpr double kRelTol = 1e-12;
  const Eigen::Index n_nlp = cvp.n_nlp_vars;
  const Eigen::Index nr = cvp.n_constraint_terms;
  const auto& A = cvp.constraint_matrix;
  const auto* outer = A.outerIndexPtr();
  const auto* inner = A.innerIndexPtr();
  const double* vals = A.valuePtr();
  const auto box_lower = cvp.bounds_lower.segment(nr, n_nlp);
  const auto box_upper = cvp.bounds_upper.segment(nr, n_nlp);

  // Classify every row; the constraint matrix is row major and compressed, so a row's NLP columns come first
  reduction_rows.assign(static_cast<std::size_t>(nr), ReductionRow{});
  reduction_order.clear();
  // Every removed row takes its one slack with it
  Eigen::Index n_removed = 0;
  for (Eigen::Index r = 0; r < nr; ++r)
  {
    auto& R = reduction_rows[static_cast<std::size_t>(r)];
    R.begin = outer[r];
    R.leader = r;
    const Eigen::Index end = outer[r + 1];
    Eigen::Index k = R.begin;
    while (k < end && inner[k] < n_nlp)
      ++k;
    R.n_nlp = k - R.begin;

    const std::int8_t slack_sign = linear_slack_row_sign[static_cast<std::size_t>(r)];
    if (slack_sign == 0)
      continue;
    assert(end - k == 1);  // NOLINT
    R.sign = slack_sign;
    R.ub = (slack_sign > 0) ? cvp.bounds_upper[r] : -cvp.bounds_lower[r];
    R.w = cvp.gradient[inner[k]];
    if (!(R.w > 0))
      continue;

    double ax0 = 0;
    double d_max = 0;
    double d_min = 0;
    double norm2 = 0;
    double scale = std::abs(R.ub);
    std::uint64_t columns = 0xcbf29ce484222325ULL;
    for (Eigen::Index j = R.begin; j < k; ++j)
    {
      const Eigen::Index c = inner[j];
      columns = (columns ^ static_cast<std::uint64_t>(c)) * 0x100000001b3ULL;
      const double a = R.sign * vals[j];
      const double to_upper = a * (box_upper[c] - x0[c]);
      const double to_lower = a * (box_lower[c] - x0[c]);
      ax0 += a * x0[c];
      d_max += std::max(to_upper, to_lower);
      d_min += std::min(to_upper, to_lower);
      norm2 += a * a;
      scale += std::abs(a) * std::max({ std::abs(x0[c]), std::abs(box_lower[c]), std::abs(box_upper[c]) });
    }
    R.norm = std::sqrt(norm2);
    R.group_w = R.w;
    const double r0 = ax0 - R.ub;
    const double margin = kRelTol * scale;
    if (r0 + d_max <= -margin)
      R.fate = Fate::kDrop;
    else if (r0 + d_min >= margin)
      R.fate = Fate::kFold;
    else
      R.fate = Fate::kKeep;

    if (R.fate != Fate::kKeep)
      ++n_removed;
    else if (R.norm > 0)
    {
      reduction_order.push_back({ columns, R.ub / R.norm, r });
    }
  }

  // Merge kept rows that are positive multiples of one another: equal NLP columns, normalized row and bound
  const auto row_of = [this](Eigen::Index r) -> ReductionRow& { return reduction_rows[static_cast<std::size_t>(r)]; };
  // Order by columns, then normalized bound, then row: rows that can merge are neighbours, the first of them leads
  std::sort(reduction_order.begin(), reduction_order.end());
  const auto same_columns = [inner](const ReductionRow& a, const ReductionRow& b) {
    return a.n_nlp == b.n_nlp && std::equal(inner + a.begin, inner + a.begin + a.n_nlp, inner + b.begin);
  };
  // Whether two rows with the same NLP columns point the same way; each coefficient is compared at its own scale
  const auto same_direction = [vals](const ReductionRow& a, const ReductionRow& b) {
    for (Eigen::Index j = 0; j < a.n_nlp; ++j)
    {
      const double da = a.sign * vals[a.begin + j] / a.norm;
      const double db = b.sign * vals[b.begin + j] / b.norm;
      if (std::abs(da - db) > kRelTol * std::max(std::abs(da), std::abs(db)))
        return false;
    }
    return true;
  };
  std::size_t window_begin = 0;
  for (std::size_t i = 0; i < reduction_order.size(); ++i)
  {
    const MergeCandidate& candidate = reduction_order[i];
    ReductionRow& R = row_of(candidate.row);
    // Rows earlier in the order with equal columns and an equal bound are the merge candidates
    while (window_begin < i)
    {
      const MergeCandidate& first = reduction_order[window_begin];
      if (first.columns == candidate.columns && candidate.t - first.t <= kRelTol * std::max(1.0, std::abs(first.t)))
        break;
      ++window_begin;
    }
    for (std::size_t j = window_begin; j < i; ++j)
    {
      const Eigen::Index l = reduction_order[j].row;
      ReductionRow& L = row_of(l);
      if (L.leader != l || !same_columns(L, R) || !same_direction(L, R))
        continue;
      R.leader = l;
      L.group_w += R.w * R.norm / L.norm;
      ++n_removed;
      break;
    }
  }

  if (n_removed == 0)
    return;

  // Emit the kept rows in their original order, each with fresh slack columns after the NLP variables
  reduced.n_rows = nr - n_removed;
  reduced.num_vars = cvp.num_qp_vars - n_removed;
  reduced.num_cnts = reduced.n_rows + reduced.num_vars;

  reduced.gradient.resize(reduced.num_vars);
  reduced.gradient.head(n_nlp) = cvp.gradient.head(n_nlp);
  reduced.bounds_lower.resize(reduced.num_cnts);
  reduced.bounds_upper.resize(reduced.num_cnts);

  // Both matrices are row major and filled in storage order: rows ascending, columns ascending within a row
  reduced.constraint_matrix.resize(reduced.num_cnts, reduced.num_vars);
  reduced.constraint_matrix.reserve(A.nonZeros());
  // The Hessian keeps its NLP block, which holds no slack columns, and the diagonal of the squared-cost slacks
  reduced.hessian.resize(reduced.num_vars, reduced.num_vars);
  reduced.hessian.reserve(cvp.hessian.nonZeros());
  for (Eigen::Index k = 0; k < n_nlp; ++k)
  {
    reduced.hessian.startVec(k);
    for (trajopt_ifopt::Jacobian::InnerIterator it(cvp.hessian, k); it; ++it)
      reduced.hessian.insertBack(k, it.col()) = it.value();
  }

  Eigen::Index row = 0;
  Eigen::Index next_var = n_nlp;
  for (Eigen::Index r = 0; r < nr; ++r)
  {
    const ReductionRow& R = row_of(r);
    if (R.fate == Fate::kDrop || R.leader != r)
      continue;
    if (R.fate == Fate::kFold)
    {
      for (Eigen::Index j = R.begin; j < R.begin + R.n_nlp; ++j)
        reduced.gradient[inner[j]] += R.w * R.sign * vals[j];
      continue;
    }

    reduced.constraint_matrix.startVec(row);
    for (Eigen::Index j = R.begin; j < R.begin + R.n_nlp; ++j)
      reduced.constraint_matrix.insertBack(row, inner[j]) = vals[j];
    for (Eigen::Index j = R.begin + R.n_nlp; j < outer[r + 1]; ++j)
    {
      const Eigen::Index col = inner[j];
      reduced.constraint_matrix.insertBack(row, next_var) = vals[j];
      reduced.gradient[next_var] = (R.fate == Fate::kKeep) ? R.group_w : cvp.gradient[col];
      reduced.hessian.startVec(next_var);
      // Only the slack of a squared cost's row, which is copied, has a Hessian entry
      if (R.fate == Fate::kCopy)
      {
        const double h = cvp.hessian.coeff(col, col);
        if (h != 0.0)
          reduced.hessian.insertBack(next_var, next_var) = h;
      }
      reduced.bounds_lower[reduced.n_rows + next_var] = cvp.bounds_lower[nr + col];
      reduced.bounds_upper[reduced.n_rows + next_var] = cvp.bounds_upper[nr + col];
      ++next_var;
    }
    reduced.bounds_lower[row] = cvp.bounds_lower[r];
    reduced.bounds_upper[row] = cvp.bounds_upper[r];
    ++row;
  }
  assert(row == reduced.n_rows && next_var == reduced.num_vars);

  for (Eigen::Index i = 0; i < reduced.num_vars; ++i)
  {
    reduced.constraint_matrix.startVec(reduced.n_rows + i);
    reduced.constraint_matrix.insertBack(reduced.n_rows + i, i) = 1.0;
  }
  reduced.constraint_matrix.finalize();
  reduced.hessian.finalize();

  reduced.box_lower = box_lower;
  reduced.box_upper = box_upper;
  reduced.active = true;
  syncReducedBounds();
}

void TrajOptQPProblem::Implementation::syncReducedBounds()
{
  if (!reduced.active)
    return;
  reduced.bounds_lower.segment(reduced.n_rows, cvp.n_nlp_vars) =
      cvp.bounds_lower.segment(cvp.n_constraint_terms, cvp.n_nlp_vars).cwiseMax(reduced.box_lower);
  reduced.bounds_upper.segment(reduced.n_rows, cvp.n_nlp_vars) =
      cvp.bounds_upper.segment(cvp.n_constraint_terms, cvp.n_nlp_vars).cwiseMin(reduced.box_upper);
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
    g(cost_idx++) = weightedSum(err.array().square(), c->getCoefficients());
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
    g(cost_idx++) = weightedSum(err.array(), c->getCoefficients());
  }

  return g;
}

ConstraintViolations TrajOptQPProblem::Implementation::getExactConstraintViolations() const
{
  const auto n = static_cast<Eigen::Index>(merit_constraints.size());
  ConstraintViolations result{ Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n) };
  Eigen::Index cnt_idx{ 0 };

  for (const auto& c : merit_constraints)
  {
    if (c->getRows() == 0)
    {
      ++cnt_idx;
      continue;
    }

    if (scratch_err.size() < c->getRows())
      scratch_err.resize(c->getRows());
    auto err = scratch_err.head(c->getRows());

    trajopt_ifopt::calcBoundsViolations(err, c->getValues(), c->getBounds());
    reduceSetViolation(result, cnt_idx++, err, c->getCoefficients(), sum_constraint_set_violations);
  }

  return result;
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
  assert(merit_coeff.size() == static_cast<Eigen::Index>(constraints.size() + dyn_constraint.size()));
  constraint_merit_coeff = merit_coeff;
}

void TrajOptQPProblem::Implementation::print() const
{
  const Eigen::IOFormat format(3);

  // The QP handed to the solver
  const ServedQP qp = servedQP();

  std::cout << "-------------- QPProblem::print() --------------" << '\n';
  std::cout << "Num NLP Vars: " << cvp.n_nlp_vars << '\n';
  std::cout << "Num QP Vars: " << qp.num_vars << '\n';
  std::cout << "Num NLP Constraints: " << qp.num_cnts << '\n';
  if (reduced.active)
    std::cout << "Reduced from QP Vars: " << cvp.num_qp_vars << ", QP Constraints: " << cvp.num_qp_cnts << '\n';
  std::cout << "Box Size: " << box_size.transpose().format(format) << '\n';  // NOLINT
  std::cout << "Constraint Merit Coeff: " << constraint_merit_coeff.transpose().format(format) << '\n';

  std::cout << "Hessian:\n" << qp.hessian.toDense().format(format) << '\n';
  std::cout << "Gradient: " << qp.gradient.transpose().format(format) << '\n';
  std::cout << "Constraint Matrix:\n" << qp.constraint_matrix.toDense().format(format) << '\n';
  std::cout << "Constraint Lower Bounds: " << qp.bounds_lower.head(qp.n_rows).transpose().format(format) << '\n';
  std::cout << "Constraint Upper Bounds: " << qp.bounds_upper.head(qp.n_rows).transpose().format(format) << '\n';
  std::cout << "Variable Lower Bounds: "
            << qp.bounds_lower.tail(qp.bounds_lower.rows() - qp.n_rows).transpose().format(format) << '\n';
  std::cout << "Variable Upper Bounds: "
            << qp.bounds_upper.tail(qp.bounds_upper.rows() - qp.n_rows).transpose().format(format) << '\n';
  std::cout << "All Lower Bounds: " << qp.bounds_lower.transpose().format(format) << '\n';
  std::cout << "All Upper Bounds: " << qp.bounds_upper.transpose().format(format) << '\n';
  std::cout << "NLP values: " << '\n' << variables->getValues().transpose().format(format) << '\n';
}

void TrajOptQPProblem::Implementation::updateNLPVariableBounds(const Eigen::Ref<const Eigen::VectorXd>& nlp_values)
{
  // Equivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  const Eigen::Index idx = cvp.n_constraint_terms;

  auto lower = cvp.bounds_lower.segment(idx, cvp.n_nlp_vars);
  auto upper = cvp.bounds_upper.segment(idx, cvp.n_nlp_vars);

  // Calculate box constraints, clamped to variable bounds. The iterate is first
  // clamped into [lb, ub] so the box stays non-empty when x has drifted outside
  // its bounds (failed step, bad warm-start); for x inside [lb, ub] this is a
  // no-op and the box is the standard strict-shrink trust region.
  for (Eigen::Index i = 0; i < cvp.n_nlp_vars; ++i)
  {
    const double bi = box_size[i];
    const double lb = var_bounds_lower[i];
    const double ub = var_bounds_upper[i];
    const double xi = std::clamp(nlp_values[i], lb, ub);

    lower[i] = std::max(xi - bi, lb);
    upper[i] = std::min(xi + bi, ub);
  }
  syncReducedBounds();
}

TrajOptQPProblem::TrajOptQPProblem(std::shared_ptr<trajopt_ifopt::Variables> variables)
  : impl_(std::make_unique<Implementation>(std::move(variables)))
{
}

TrajOptQPProblem::~TrajOptQPProblem() = default;
TrajOptQPProblem::TrajOptQPProblem(TrajOptQPProblem&&) noexcept = default;
TrajOptQPProblem& TrajOptQPProblem::operator=(TrajOptQPProblem&&) noexcept = default;

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

ConstraintViolations
TrajOptQPProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const
{
  const Implementation& impl = *impl_;
  return impl.cvp.evaluateConvexConstraintViolations(var_vals, impl.sum_constraint_set_violations);
}

ConstraintViolations TrajOptQPProblem::getExactConstraintViolations() const
{
  return std::as_const<Implementation>(*impl_).getExactConstraintViolations();  // NOLINT
}

void TrajOptQPProblem::scaleBoxSize(double& scale) { impl_->scaleBoxSize(scale); }

void TrajOptQPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) { impl_->setBoxSize(box_size); }

void TrajOptQPProblem::setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff)
{
  impl_->setConstraintMeritCoeff(merit_coeff);
}

void TrajOptQPProblem::setSumConstraintSetViolations(bool sum) { impl_->sum_constraint_set_violations = sum; }

void TrajOptQPProblem::setQPRowReduction(bool reduce) { impl_->qp_row_reduction = reduce; }

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
  return impl_->servedQP().num_vars;
}

Eigen::Index TrajOptQPProblem::getNumQPConstraints() const
{
  assert(impl_->initialized);
  return impl_->servedQP().num_cnts;
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

const trajopt_ifopt::Jacobian& TrajOptQPProblem::getHessian() const { return impl_->servedQP().hessian; }
const Eigen::VectorXd& TrajOptQPProblem::getGradient() const { return impl_->servedQP().gradient; }

const trajopt_ifopt::Jacobian& TrajOptQPProblem::getConstraintMatrix() const
{
  return impl_->servedQP().constraint_matrix;
}
const Eigen::VectorXd& TrajOptQPProblem::getBoundsLower() const { return impl_->servedQP().bounds_lower; }
const Eigen::VectorXd& TrajOptQPProblem::getBoundsUpper() const { return impl_->servedQP().bounds_upper; }

}  // namespace trajopt_sqp
