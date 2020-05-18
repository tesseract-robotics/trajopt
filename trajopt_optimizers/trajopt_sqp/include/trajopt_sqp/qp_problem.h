/**
 * @file qp_problem.h
 * @brief Converts general NLP to QP for SQP routine
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TRAJOPT_SQP_INCLUDE_QP_PROBLEM_H_
#define TRAJOPT_SQP_INCLUDE_QP_PROBLEM_H_

#include <memory>
#include <trajopt_sqp/types.h>
#include <ifopt/problem.h>

namespace trajopt_sqp
{
enum class ConstraintType
{
  EQ,
  INEQ
};

class QPProblem
{
public:
  using Ptr = std::shared_ptr<QPProblem>;
  using ConstPtr = std::shared_ptr<const QPProblem>;

  /** @brief Sets up the problem and initializes matrices
   * @param nlp
   */
  void init(ifopt::Problem& nlp);
  /** @brief Run the full convexification routine. If in doubt, init(nlp) then convexify() */
  void convexify();

  /** @brief Updates the cost hessian (Currently does nothing since IFOPT doesn't support Hessians*/
  void updateHessian();
  /** @brief Update the cost gradient including slack variables */
  void updateGradient();

  void linearizeConstraints();

  void updateNLPConstraintBounds();

  void updateNLPVariableBounds();

  void updateSlackVariableBounds();

  void convexifyAndSolve();

  double evaluateTotalConvexCost(const Eigen::Ref<Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<Eigen::VectorXd>& var_vals);

  Eigen::VectorXd getExactConstraintViolations();

  void scaleBoxSize(double& scale);

  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size);

  Eigen::VectorXd getBoxSize() const;

  void print() const;

protected:
  ifopt::Problem* nlp_;

  // TODO: I don't really want people to have write access to these. make const getters
public:
  Eigen::Index num_nlp_vars_;
  Eigen::Index num_nlp_cnts_;
  Eigen::Index num_qp_vars_;
  Eigen::Index num_qp_cnts_;
  std::vector<ConstraintType> constraint_types_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;
  Eigen::VectorXd constraint_merit_coeff_;

  Eigen::SparseMatrix<double> hessian_;
  Eigen::VectorXd gradient_;

  Eigen::SparseMatrix<double> constraint_matrix_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
};

}  // namespace trajopt_sqp

#endif
