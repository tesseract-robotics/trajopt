/**
 * @file osqp_eigen_solver.cpp
 * @brief Interface to the OSQPEigen solver
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
#include <trajopt_sqp/osqp_eigen_solver.h>

#include <OsqpEigen/OsqpEigen.h>

namespace trajopt_sqp
{
bool OSQPEigenSolver::init(Eigen::Index num_vars, Eigen::Index num_cnts)
{
  // Set the solver size
  num_cnts_ = num_cnts;
  num_vars_ = num_vars;
  solver_.data()->setNumberOfVariables(static_cast<int>(num_vars_));
  solver_.data()->setNumberOfConstraints(static_cast<int>(num_cnts_));

  // Initialize the bounds
  bounds_lower_ = Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY;
  bounds_upper_ = Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY;

  return solver_.initSolver();
}

bool OSQPEigenSolver::clear()
{
  // Clear all data
  solver_.clearSolver();
  solver_.data()->clearHessianMatrix();
  solver_.data()->clearLinearConstraintsMatrix();
  return true;
}

bool OSQPEigenSolver::solve() { return solver_.solve(); }

Eigen::VectorXd OSQPEigenSolver::getSolution()
{
  Eigen::VectorXd solution = solver_.getSolution();
  return solution;
}

bool OSQPEigenSolver::updateHessianMatrix(const Hessian& hessian)
{
  if (solver_.isInitialized())
    return solver_.updateHessianMatrix(hessian);
  solver_.data()->clearHessianMatrix();
  Hessian cleaned = hessian.pruned(1e-7);
  std::cout << "Hessian nonzero: " << cleaned.nonZeros() << std::endl;

  return solver_.data()->setHessianMatrix(cleaned);
}

bool OSQPEigenSolver::updateGradient(const Eigen::Ref<Eigen::VectorXd>& gradient)
{
  if (solver_.isInitialized())
    return solver_.updateGradient(gradient);

  return solver_.data()->setGradient(gradient);
}

bool OSQPEigenSolver::updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound)
{
  bounds_lower_ = lowerBound.cwiseMax(Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY);
  return solver_.updateLowerBound(bounds_lower_);
}

bool OSQPEigenSolver::updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  bounds_upper_ = upperBound.cwiseMin(Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY);
  return solver_.updateUpperBound(bounds_upper_);
}

bool OSQPEigenSolver::updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                                   const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  bounds_lower_ = lowerBound.cwiseMax(Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY);
  bounds_upper_ = upperBound.cwiseMin(Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY);
  if (solver_.isInitialized())
    return solver_.updateBounds(bounds_lower_, bounds_upper_);

  solver_.data()->setLowerBound(bounds_lower_);
  solver_.data()->setUpperBound(bounds_upper_);
  return true;
}

bool OSQPEigenSolver::updateLinearConstraintsMatrix(const Jacobian& linearConstraintsMatrix)
{
  solver_.data()->clearLinearConstraintsMatrix();
  Jacobian cleaned = linearConstraintsMatrix.pruned(1e-7);
  std::cout << "Constraint Matrix nonzero: " << cleaned.nonZeros() << std::endl;
  return solver_.data()->setLinearConstraintsMatrix(cleaned);
  //  return solver_.updateLinearConstraintsMatrix(linearConstraintsMatrix);
}

QP_SOLVER_STATUS OSQPEigenSolver::getSolverStatus() { return QP_SOLVER_STATUS::INITIALIZED; }

}  // namespace trajopt_sqp
