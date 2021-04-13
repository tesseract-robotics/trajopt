#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <functional>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/modeling.hpp>

/*
 * Algorithms for non-convex, constrained optimization
 */

namespace sco
{
enum OptStatus
{
  OPT_CONVERGED,
  OPT_SCO_ITERATION_LIMIT,  // hit iteration limit before convergence
  OPT_PENALTY_ITERATION_LIMIT,
  OPT_TIME_LIMIT,
  OPT_FAILED,
  INVALID
};
static const char* OptStatus_strings[] = { "CONVERGED",
                                           "SCO_ITERATION_LIMIT",
                                           "PENALTY_ITERATION_LIMIT",
                                           "FAILED",
                                           "INVALID" };
inline std::string statusToString(OptStatus status) { return OptStatus_strings[status]; }
struct OptResults
{
  DblVec x;  // solution estimate
  OptStatus status;
  double total_cost;
  DblVec cost_vals;
  DblVec cnt_viols;
  int n_func_evals, n_qp_solves;
  void clear()
  {
    x.clear();
    status = INVALID;
    cost_vals.clear();
    cnt_viols.clear();
    n_func_evals = 0;
    n_qp_solves = 0;
  }
  OptResults() { clear(); }
};

std::ostream& operator<<(std::ostream& o, const OptResults& r);

class Optimizer
{
  /*
   * Solves an optimization problem
   */
public:
  using Ptr = std::shared_ptr<Optimizer>;

  Optimizer() = default;
  virtual ~Optimizer() = default;
  Optimizer(const Optimizer&) = default;
  Optimizer& operator=(const Optimizer&) = default;
  Optimizer(Optimizer&&) = default;
  Optimizer& operator=(Optimizer&&) = default;

  virtual OptStatus optimize() = 0;
  virtual void setProblem(OptProb::Ptr prob) { prob_ = std::move(prob); }
  void initialize(const DblVec& x);
  DblVec& x() { return results_.x; }
  OptResults& results() { return results_; }
  using Callback = std::function<void(OptProb*, OptResults&)>;
  void addCallback(const Callback& cb);  // called before each iteration
protected:
  std::vector<Callback> callbacks_;
  void callCallbacks();
  OptProb::Ptr prob_;
  OptResults results_;
};

struct BasicTrustRegionSQPParameters
{
  double improve_ratio_threshold;  // minimum ratio true_improve/approx_improve
                                   // to accept step
  double min_trust_box_size;       // if trust region gets any smaller, exit and
                                   // report convergence
  double min_approx_improve;       // if model improves less than this, exit and
                                   // report convergence
  double min_approx_improve_frac;  // if model improves less than this, exit and
                                   // report convergence
  double max_iter;                 // The max number of iterations
  double trust_shrink_ratio;       // if improvement is less than
  // improve_ratio_threshold, shrink trust region by
  // this ratio
  double trust_expand_ratio;  // if improvement is less than
                              // improve_ratio_threshold, shrink trust region by
                              // this ratio
  double cnt_tolerance;       // after convergence of penalty subproblem, if
  // constraint violation is less than this, we're done
  /** @brief Max number of times that the constraints' cost will be increased */
  double max_merit_coeff_increases;
  /** @brief Max number of times the QP solver can fail before optimization is aborted*/
  int max_qp_solver_failures;

  double merit_coeff_increase_ratio;  // ratio that we increate coeff each time
  /** @brief Max time in seconds that the optimizer will run*/
  double max_time;
  /** @brief Initial coefficient that is used to scale the constraints. The total constaint cost is constaint_value *
   * coeff * merit_coeff */
  double initial_merit_error_coeff;
  /** @brief If true, merit coeffs will only be inflated for the constaints that failed. This can help when there are
   * lots of constaints*/
  bool inflate_constraints_individually;
  double trust_box_size;  // current size of trust region (component-wise)

  bool log_results;     // Log results to file
  std::string log_dir;  // Directory to store log results (Default: /tmp)

  BasicTrustRegionSQPParameters();
};

/**
 * @brief This struct stores iteration results for the BasicTrustRegionSQP
 *
 * Variable with model_ indicates values calcuated using the Convex approximation
 * of the cost or constraint.
 *
 * Variables without model_ indicates values calcuated using the exact
 * function provided for the cost or constraint
 *
 */
struct BasicTrustRegionSQPResults
{
  /** @brief The model optimization variables */
  DblVec model_var_vals;
  /** @brief The model (Covex Representation) cost values using the new_x values */
  DblVec model_cost_vals;
  /** @brief The model (Covex Representation) constraint violations using the new_x values */
  DblVec model_cnt_viols;
  /** @brief The new optimzation variable values */
  DblVec new_x;
  /** @brief The cost (exact) values calculated using the new_x values */
  DblVec new_cost_vals;
  /** @brief The previous iterations cost (exact) values */
  DblVec old_cost_vals;
  /** @brief The constraint violations (exact) calculated using the new_x values */
  DblVec new_cnt_viols;
  /** @brief The previous iterations constraint (exact) values */
  DblVec old_cnt_viols;
  /** @brief The previous iterations (exact) merit = vecSum(old_cost_vals) + merit_error_coeff * vecSum(old_cnt_viols)
   */
  double old_merit;
  /** @brief The models merit = vecSum(model_cost_vals) + merit_error_coeff * vecSum(model_cnt_viols) */
  double model_merit;
  /** @brief The exact merit = vecSum(new_cost_vals) + merit_error_coeff * vecSum(new_cnt_viols) */
  double new_merit;
  /**
   * @brief A measure of improvement approximated using values from the model.
   * approx_merit_improve = old_merit - model_merit;
   */
  double approx_merit_improve;
  /**
   * @brief A measure of improvement using exact values
   * exact_merit_improve = old_merit - new_merit;
   */
  double exact_merit_improve;
  /**
   * @brief The ratio between the exact and model merit improvement
   * merit_improve_ratio = exact_merit_improve / approx_merit_improve;
   */
  double merit_improve_ratio;
  /** @brief This is the penalty applied to the constraints for this iteration */
  std::vector<double> merit_error_coeffs;
  /** @brief Variable names */
  const std::vector<std::string> var_names;
  /** @brief Cost Names */
  const std::vector<std::string> cost_names;
  /** @brief Constraint Names */
  const std::vector<std::string> cnt_names;

  BasicTrustRegionSQPResults(std::vector<std::string> var_names,
                             std::vector<std::string> cost_names,
                             std::vector<std::string> cnt_names);

  /**
   * @brief Update the structure data for a new iteration
   * @param prev_opt_results The previous optimization results
   * @param model The current model
   * @param cost_models The current cost models
   * @param cnt_models The current constraint models
   * @param cnt_cost_models The current constraint cost models
   * @param constraints The current exact constraints
   * @param costs The current exact costs
   * @param merit_error_coeff The iteration penalty to apply to constraints
   */
  void update(const OptResults& prev_opt_results,
              const Model& model,
              const std::vector<ConvexObjective::Ptr>& cost_models,
              const std::vector<ConvexConstraints::Ptr>& cnt_models,
              const std::vector<ConvexObjective::Ptr>& cnt_cost_models,
              const std::vector<Constraint::Ptr>& constraints,
              const std::vector<Cost::Ptr>& costs,
              std::vector<double> merit_error_coeffs);

  /** @brief Print current results to the terminal */
  void print() const;
  /** @brief Write solver results to a file */
  void writeSolver(std::FILE* stream, bool header = false) const;
  /** @brief Write variable values to a file */
  void writeVars(std::FILE* stream, bool header = false) const;
  /** @brief Write cost results to file */
  void writeCosts(std::FILE* stream, bool header = false) const;
  /** @brief Write constraint results to file */
  void writeConstraints(std::FILE* stream, bool header = false) const;
  /** @brief Prints the raw values to the terminal */
  void printRaw() const;
};

class BasicTrustRegionSQP : public Optimizer
{
  /*
   * Alternates between convexifying objectives and constraints and then solving
   * convex subproblem
   * Uses a merit function to decide whether or not to accept the step
   * merit function = objective + merit_err_coeff * | constraint_error |
   * Note: sometimes the convexified objectives and constraints lead to an
   * infeasible subproblem
   * In that case, you should turn them into penalties and solve that problem
   * (todo: implement penalty-based sqp that gracefully handles infeasible
   * constraints)
   */
public:
  using Ptr = std::shared_ptr<BasicTrustRegionSQP>;

  BasicTrustRegionSQP() = default;
  BasicTrustRegionSQP(const OptProb::Ptr& prob);
  void setProblem(OptProb::Ptr prob) override;
  void setParameters(const BasicTrustRegionSQPParameters& param) { param_ = param; }
  const BasicTrustRegionSQPParameters& getParameters() const { return param_; }
  BasicTrustRegionSQPParameters& getParameters() { return param_; }
  OptStatus optimize() override;

protected:
  void ctor(const OptProb::Ptr& prob);
  void adjustTrustRegion(double ratio);
  void setTrustBoxConstraints(const DblVec& x);
  Model::Ptr model_;
  BasicTrustRegionSQPParameters param_;
};
}  // namespace sco
