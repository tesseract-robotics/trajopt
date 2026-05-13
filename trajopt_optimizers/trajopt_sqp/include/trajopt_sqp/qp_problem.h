#ifndef TRAJOPT_SQP_QP_PROBLEM_BASE_H
#define TRAJOPT_SQP_QP_PROBLEM_BASE_H

#include <memory>
#include <trajopt_ifopt/core/eigen_types.h>
#include <trajopt_ifopt/fwd.h>

namespace trajopt_sqp
{
enum class CostPenaltyType : std::uint8_t;

/** @brief QP Problem Base */
class QPProblem
{
public:
  using Ptr = std::shared_ptr<QPProblem>;
  using ConstPtr = std::shared_ptr<const QPProblem>;

  virtual ~QPProblem() = default;

  /**
   * @brief Add a set of multiple constraints to the optimization problem.
   * @param constraint_set  This can be 1 to infinity number of constraints.
   *
   * This function can be called multiple times for different sets of
   * constraints. It makes sure the overall constraint and Jacobian correctly
   * considers all individual constraint sets.
   */
  virtual void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set) = 0;

  /**
   * @brief Add a squared cost term to the problem.
   * @param constraint_set The constraint set to be evaluated as a squared cost.
   *
   * This function can be called multiple times if the constraint function is
   * composed of different cost terms. It makes sure the overall value and
   * gradient is considering each individual cost.
   */
  virtual void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                          CostPenaltyType penalty_type) = 0;

  /**
   * @brief This setups the QP problems based on the constraints and cost sets added to the problem.
   * @details This must be called after all constraints and costs have been added to the problem
   */
  virtual void setup() = 0;

  /** @brief Set the current Optimization variables */
  virtual void setVariables(const double* x) = 0;

  /** @brief Get the current Optimization variable values */
  virtual Eigen::VectorXd getVariableValues() const = 0;

  /** @brief Run the full convexification routine */
  virtual void convexify() = 0;

  /**
   * @brief Evaluates the cost of the convexified function (ie using the stored gradient and hessian) at var_vals
   * @param var_vals Point at which the convex cost is calculated. Should be size num_qp_vars
   */
  virtual double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const = 0;

  /**
   * @brief Evaluated the cost of the convexified function (ie using the stored gradient and hessian) at var_vals
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   * @param var_vals Point at which the convex cost is calculated. Should be size num_qp_vars
   * @return Cost associated with each cost term in the problem (for debugging)
   */
  virtual Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const = 0;

  /**
   * @brief Get the sum of the cost functions
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   */
  virtual double getTotalExactCost() const = 0;

  /**
   * @brief Get the current NLP costs.
   * @return Vector of costs.
   */
  virtual Eigen::VectorXd getExactCosts() const = 0;

  /**
   * @brief Evaluated the costraint violation of the convexified function (ie using the stored constraint matrix) at
   * var_vals
   * @param var_vals
   * @return
   */
  virtual Eigen::VectorXd
  evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const = 0;

  /**
   * @brief get the current NLP constraint violations. Values > 0 are violations
   * @return Vector of constraint violations. Values > 0 are violations
   */
  virtual Eigen::VectorXd getExactConstraintViolations() const = 0;
  /**
   * @brief Uniformly scales the box size  (box_size_ = box_size_ * scale)
   * @param scale Value by which the box size is scaled
   */
  virtual void scaleBoxSize(double& scale) = 0;
  /**
   * @brief Sets the box size to the input vector
   * @param box_size New box size
   */
  virtual void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) = 0;

  /**
   * @brief Set the constraint merit coeff
   * @details This controls the gradient for the constraint slack variables.
   * @param merit_coeff
   */
  virtual void setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff) = 0;

  /** @brief Prints all members to the terminal in a human readable form */
  virtual void print() const = 0;

  virtual Eigen::Index getNumNLPVars() const = 0;
  virtual Eigen::Index getNumNLPConstraints() const = 0;
  virtual Eigen::Index getNumNLPCosts() const = 0;
  virtual Eigen::Index getNumQPVars() const = 0;
  virtual Eigen::Index getNumQPConstraints() const = 0;

  virtual const std::vector<std::string>& getNLPConstraintNames() const = 0;
  virtual const std::vector<std::string>& getNLPCostNames() const = 0;

  virtual const Eigen::VectorXd& getBoxSize() const = 0;
  virtual const Eigen::VectorXd& getConstraintMeritCoeff() const = 0;

  virtual const trajopt_ifopt::Jacobian& getHessian() const = 0;
  virtual const Eigen::VectorXd& getGradient() const = 0;

  virtual const trajopt_ifopt::Jacobian& getConstraintMatrix() const = 0;
  virtual const Eigen::VectorXd& getBoundsLower() const = 0;
  virtual const Eigen::VectorXd& getBoundsUpper() const = 0;
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_QP_PROBLEM_BASE_H
