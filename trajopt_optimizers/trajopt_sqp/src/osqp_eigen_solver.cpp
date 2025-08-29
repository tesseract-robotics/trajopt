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

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <OsqpEigen/OsqpEigen.h>
TRAJOPT_IGNORE_WARNINGS_POP

const bool OSQP_COMPARE_DEBUG_MODE = false;

namespace trajopt_sqp
{
OSQPEigenSolver::OSQPEigenSolver() : solver_(std::make_unique<OsqpEigen::Solver>())
{
  setDefaultOSQPSettings(*solver_->settings());
  if (verbosity > 0)
    solver_->settings()->setVerbosity(true);
  else
    solver_->settings()->setVerbosity(false);
}

OSQPEigenSolver::~OSQPEigenSolver() = default;

void OSQPEigenSolver::setDefaultOSQPSettings(OsqpEigen::Settings& settings)
{
  settings.resetDefaultSettings();
  settings.setVerbosity(false);
  settings.setWarmStart(true);
  settings.setPolish(true);
  settings.setAdaptiveRho(true);
  settings.setMaxIteration(8192);
  settings.setAbsoluteTolerance(1e-4);
  settings.setRelativeTolerance(1e-6);
  settings.setCheckDualGap(false);
}

bool OSQPEigenSolver::init(Eigen::Index num_vars, Eigen::Index num_cnts)
{
  // Set the solver size
  num_cnts_ = num_cnts;
  num_vars_ = num_vars;
  solver_->data()->setNumberOfVariables(static_cast<int>(num_vars_));
  solver_->data()->setNumberOfConstraints(static_cast<int>(num_cnts_));

  solver_status_ = QPSolverStatus::INITIALIZED;

  return true;
}

bool OSQPEigenSolver::clear()
{
  // Clear all data
  solver_->clearSolver();
  solver_->data()->clearHessianMatrix();
  solver_->data()->clearLinearConstraintsMatrix();
  return true;
}

bool OSQPEigenSolver::solve()
{
  // In order to call initSolver, everything must have already been set, so we call it right before solving
  if (!solver_->isInitialized())  // NOLINT
    solver_->initSolver();

  if (OSQP_COMPARE_DEBUG_MODE)
  {
    const Eigen::IOFormat format(5);
    const auto* osqp_data = solver_->data()->getData();
    std::cout << "OSQP Number of Variables:" << osqp_data->n << '\n';
    std::cout << "OSQP Number of Constraints:" << osqp_data->m << '\n';

    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> P_p_vec(osqp_data->P->p, osqp_data->P->n + 1);
    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> P_i_vec(osqp_data->P->i, osqp_data->P->nzmax);
    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> P_x_vec(osqp_data->P->x, osqp_data->P->nzmax);
    std::cout << "OSQP Hessian:" << '\n';
    std::cout << "     nzmax:" << osqp_data->P->nzmax << '\n';
    std::cout << "        nz:" << osqp_data->P->nz << '\n';
    std::cout << "         m:" << osqp_data->P->m << '\n';
    std::cout << "         n:" << osqp_data->P->n << '\n';
    std::cout << "         p:" << P_p_vec.transpose().format(format) << '\n';
    std::cout << "         i:" << P_i_vec.transpose().format(format) << '\n';
    std::cout << "         x:" << P_x_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::VectorXd> q_vec(osqp_data->q, osqp_data->n);
    std::cout << "OSQP Gradient: " << q_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> A_p_vec(osqp_data->A->p, osqp_data->A->n + 1);
    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> A_i_vec(osqp_data->A->i, osqp_data->A->nzmax);
    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> A_x_vec(osqp_data->A->x, osqp_data->A->nzmax);
    std::cout << "OSQP Constraint Matrix:" << '\n';
    std::cout << "     nzmax:" << osqp_data->A->nzmax << '\n';
    std::cout << "         m:" << osqp_data->A->m << '\n';
    std::cout << "         n:" << osqp_data->A->n << '\n';
    std::cout << "         p:" << A_p_vec.transpose().format(format) << '\n';
    std::cout << "         i:" << A_i_vec.transpose().format(format) << '\n';
    std::cout << "         x:" << A_x_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> l_vec(osqp_data->l, osqp_data->m);
    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> u_vec(osqp_data->u, osqp_data->m);
    std::cout << "OSQP Lower Bounds: " << l_vec.transpose().format(format) << '\n';
    std::cout << "OSQP Upper Bounds: " << u_vec.transpose().format(format) << '\n';

    std::cout << "OSQP Variable Names: " << '\n';
  }

  /** @todo Need to check if this is what we want in the new version */
  const auto solveExitFlag = solver_->solveProblem();
  const auto status = solver_->getStatus();
  if (OSQP_COMPARE_DEBUG_MODE)
    std::cout << "OSQP Status Value: " << static_cast<int>(solver_->getStatus()) << '\n';

  if ((solveExitFlag == OsqpEigen::ErrorExitFlag::NoError) &&
      ((status == OsqpEigen::Status::Solved) || (status == OsqpEigen::Status::SolvedInaccurate)))
  {
    if (OSQP_COMPARE_DEBUG_MODE)
    {
      const Eigen::IOFormat format(5);
      std::cout << "OSQP Solution: " << solver_->getSolution().transpose().format(format) << '\n';
    }
    return true;
  }

  if (verbosity > 0)  // NOLINT
  {
    // If primal infeasible
    if (status == OsqpEigen::Status::PrimalInfeasible)
    {
      Eigen::Map<Eigen::VectorXd> primal_certificate(solver_->solver()->solution->prim_inf_cert, num_cnts_, 1);
      std::cout << "OSQP Status: " << static_cast<int>(status) << '\n';
      std::cout << "\n---------------------------------------\n";
      std::cout << std::scientific << "Primal Certificate (v): " << primal_certificate.transpose() << '\n';

      const double first = bounds_lower_.transpose() * primal_certificate;
      const double second = bounds_upper_.transpose() * primal_certificate;

      std::cout << "A.transpose() * v = 0\n"
                << "l.transpose() * v = " << first << "    u.transpose() * v = " << second << '\n';
      std::cout << "l.transpose() * v + u.transpose() * v  = " << first + second << " < 0\n";
      std::cout << "Bounds_lower: " << bounds_lower_.transpose() << '\n';
      std::cout << "Bounds_upper: " << bounds_upper_.transpose() << '\n';
      std::cout << std::fixed;
      std::cout << "\n---------------------------------------\n";
    }

    // If dual infeasible
    if (status == OsqpEigen::Status::DualInfeasible)  // NOLINT
    {
      Eigen::Map<Eigen::VectorXd> dual_certificate(solver_->solver()->solution->dual_inf_cert, num_vars_, 1);
      std::cout << "OSQP Status: " << static_cast<int>(status) << '\n';
      std::cout << "\n---------------------------------------\n";
      std::cout << "Dual Certificate (x): " << dual_certificate.transpose() << '\n';  // NOLINT

      // TODO: Find a way to get the hessian here
      //        std::cout << "P*x = " << (qp_problem_->getHessian() * dual_certificate).transpose() << " = 0" <<
      //        '\n';
      std::cout << "q.transpose() * x = " << gradient_.transpose() * dual_certificate << " < 0" << '\n';
      std::cout << std::fixed;
      std::cout << "\n---------------------------------------\n";
    }
  }

  solver_status_ = QPSolverStatus::QP_ERROR;
  return false;
}

Eigen::VectorXd OSQPEigenSolver::getSolution()
{
  Eigen::VectorXd solution = solver_->getSolution();
  return solution;
}

bool OSQPEigenSolver::updateHessianMatrix(const SparseMatrix& hessian)
{
  // Clean up values close to 0
  // Also multiply by 2 because OSQP is multiplying by (1/2) for the objective fuction
  const SparseMatrix cleaned = 2.0 * hessian.pruned(1e-7, 1);  // Any value < 1e-7 will be removed

  if (solver_->isInitialized())
  {
    const bool success = solver_->updateHessianMatrix(cleaned);
    return success;
  }

  solver_->data()->clearHessianMatrix();
  const bool success = solver_->data()->setHessianMatrix(cleaned);

  return success;
}

bool OSQPEigenSolver::updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient)
{
  gradient_ = gradient;

  if (solver_->isInitialized())
    return solver_->updateGradient(gradient_);

  return solver_->data()->setGradient(gradient_);
}

bool OSQPEigenSolver::updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound)
{
  bounds_lower_ = lowerBound.cwiseMax(Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY);
  return solver_->updateLowerBound(bounds_lower_);
}

bool OSQPEigenSolver::updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  bounds_upper_ = upperBound.cwiseMin(Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY);
  return solver_->updateUpperBound(bounds_upper_);
}

bool OSQPEigenSolver::updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                                   const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  bounds_lower_ = lowerBound.cwiseMax(Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY);
  bounds_upper_ = upperBound.cwiseMin(Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY);

  if (solver_->isInitialized())
    return solver_->updateBounds(bounds_lower_, bounds_upper_);

  bool success = solver_->data()->setLowerBound(bounds_lower_);
  success &= solver_->data()->setUpperBound(bounds_upper_);
  return success;
}

bool OSQPEigenSolver::updateLinearConstraintsMatrix(const SparseMatrix& linearConstraintsMatrix)
{
  assert(num_cnts_ == linearConstraintsMatrix.rows());
  assert(num_vars_ == linearConstraintsMatrix.cols());

  solver_->data()->clearLinearConstraintsMatrix();
  const SparseMatrix cleaned = linearConstraintsMatrix.pruned(1e-7, 1);  // Any value < 1e-7 will be removed

  if (solver_->isInitialized())
  {
    const bool success = solver_->updateLinearConstraintsMatrix(cleaned);
    return success;
  }

  const bool success = solver_->data()->setLinearConstraintsMatrix(cleaned);
  return success;
}

}  // namespace trajopt_sqp
