#ifndef TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H
#define TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H

#include <trajopt_sqp/qp_problem.h>

#include <memory>

namespace trajopt_sqp
{
/** @brief Converts a general NLP into a convexified QP that can be solved by a QP solver */
class TrajOptQPProblem : public QPProblem
{
public:
  using Ptr = std::shared_ptr<TrajOptQPProblem>;
  using ConstPtr = std::shared_ptr<const TrajOptQPProblem>;

  TrajOptQPProblem(std::shared_ptr<trajopt_ifopt::Variables> variables);
  ~TrajOptQPProblem() override;
  TrajOptQPProblem(const TrajOptQPProblem&) = delete;
  TrajOptQPProblem& operator=(const TrajOptQPProblem&) = delete;
  TrajOptQPProblem(TrajOptQPProblem&&) = default;
  TrajOptQPProblem& operator=(TrajOptQPProblem&&) = default;

  void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set) override;

  void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set, CostPenaltyType penalty_type) override;

  void setup() override;

  void setVariables(const double* x) override;

  Eigen::VectorXd getVariableValues() const override;

  void convexify() override;

  double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override;

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override;

  double getTotalExactCost() const override;

  Eigen::VectorXd getExactCosts() const override;

  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override;

  Eigen::VectorXd getExactConstraintViolations() const override;

  void scaleBoxSize(double& scale) override;

  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) override;

  void setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff) override;

  void print() const override;

  Eigen::Index getNumNLPVars() const override;
  Eigen::Index getNumNLPConstraints() const override;
  Eigen::Index getNumNLPCosts() const override;
  Eigen::Index getNumQPVars() const override;
  Eigen::Index getNumQPConstraints() const override;

  const std::vector<std::string>& getNLPConstraintNames() const override;
  const std::vector<std::string>& getNLPCostNames() const override;

  const Eigen::VectorXd& getBoxSize() const override;
  const Eigen::VectorXd& getConstraintMeritCoeff() const override;

  const trajopt_ifopt::Jacobian& getHessian() const override;
  const Eigen::VectorXd& getGradient() const override;

  const trajopt_ifopt::Jacobian& getConstraintMatrix() const override;
  const Eigen::VectorXd& getBoundsLower() const override;
  const Eigen::VectorXd& getBoundsUpper() const override;

private:
  struct Implementation;
  std::unique_ptr<Implementation> impl_;
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H
