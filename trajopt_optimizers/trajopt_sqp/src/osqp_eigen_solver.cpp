/**
 * @file osqp_eigen_solver.cpp
 * @brief Interface to the OSQPEigen solver
 *
 * @author Matthew Powelson
 * @date May 18, 2020
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
#include <trajopt_sqp/qp_problem.h>

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <OsqpEigen/OsqpEigen.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace
{
constexpr bool OSQP_COMPARE_DEBUG_MODE = false;
}

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
  x0_.setZero(num_vars_);
  y0_.setZero(num_cnts_);
  solver_->data()->setNumberOfVariables(static_cast<int>(num_vars_));
  solver_->data()->setNumberOfConstraints(static_cast<int>(num_cnts_));

  solver_status_ = QPSolverStatus::kInitialized;

  return true;
}

bool OSQPEigenSolver::clear()
{
  // Clear all data
  solver_->clearSolver();
  solver_->data()->clearHessianMatrix();
  solver_->data()->clearLinearConstraintsMatrix();

  num_vars_ = 0;
  num_cnts_ = 0;
  x0_.resize(0);
  y0_.resize(0);
  gradient_.resize(0);
  bounds_lower_.resize(0);
  bounds_upper_.resize(0);
  solver_status_ = QPSolverStatus::kUninitialized;
  return true;
}

bool OSQPEigenSolver::solve()
{
  // In order to call initSolver, everything must have already been set, so we call it right before solving
  if (!solver_->isInitialized())  // NOLINT
  {
    if (!solver_->initSolver())
    {
      solver_status_ = QPSolverStatus::kFailed;
      return false;
    }

    // Apply stored warm start if the setting is enabled
    if (solver_->settings()->getSettings()->warm_starting == 1)
      solver_->setWarmStart(x0_, y0_);
  }

  const Eigen::IOFormat format(8);
  if (OSQP_COMPARE_DEBUG_MODE)
  {
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

    // Eigen::SparseMatrix<OSQPFloat> P_dense;
    // OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToEigenSparseMatrix<OSQPFloat>(osqp_data->P, P_dense);
    // std::cout << "         d:\n" << P_dense.toDense().format(format) << '\n';

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

    // Eigen::SparseMatrix<OSQPFloat> A_dense;
    // OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToEigenSparseMatrix<OSQPFloat>(osqp_data->A, A_dense);
    // std::cout << "         d:\n" << A_dense.toDense().format(format) << '\n';

    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> l_vec(osqp_data->l, osqp_data->m);
    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> u_vec(osqp_data->u, osqp_data->m);
    std::cout << "OSQP Lower Bounds: " << l_vec.transpose().format(format) << '\n';
    std::cout << "OSQP Upper Bounds: " << u_vec.transpose().format(format) << '\n';
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

  solver_status_ = QPSolverStatus::kFailed;
  return false;
}

Eigen::VectorXd OSQPEigenSolver::getSolution() { return solver_->getSolution(); }

bool OSQPEigenSolver::updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian)
{
  // Also multiply by 2 because OSQP is multiplying by (1/2) for the objective fuction
  auto h2 = 2.0 * hessian; /** @todo This should be handled already by who is calling this function */
  if (solver_->isInitialized())
    return solver_->updateHessianMatrix(h2.eval());

  solver_->data()->clearHessianMatrix();
  return solver_->data()->setHessianMatrix(h2.eval());
}

bool OSQPEigenSolver::updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient)
{
  gradient_ = (gradient.array().abs() < 1e-7).select(0.0, gradient.array());

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

  return solver_->data()->setBounds(bounds_lower_, bounds_upper_);
}

bool OSQPEigenSolver::updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& linearConstraintsMatrix)
{
  assert(num_cnts_ == linearConstraintsMatrix.rows());
  assert(num_vars_ == linearConstraintsMatrix.cols());

  if (solver_->isInitialized())
    return solver_->updateLinearConstraintsMatrix(linearConstraintsMatrix);

  solver_->data()->clearLinearConstraintsMatrix();
  return solver_->data()->setLinearConstraintsMatrix(linearConstraintsMatrix);
}

bool OSQPEigenSolver::setWarmStart(const QPProblem& qp_problem)
{
  if (solver_->settings()->getSettings()->warm_starting != 1)
    return true;

  // Initialize primal variables with NLP variables followed by slack variables
  const Eigen::Index num_nlp_vars = qp_problem.getNumNLPVars();
  const Eigen::Index num_slacks = num_vars_ - num_nlp_vars;
  x0_.setZero(num_vars_);

  // Extract NLP variable values from the problem
  const Eigen::VectorXd nlp_vars = qp_problem.getVariableValues();
  assert(nlp_vars.size() == num_nlp_vars);

  // Set the primal NLP variables
  x0_.head(num_nlp_vars) = nlp_vars;

  // If there are slack variables, compute them from constraint violations
  if (num_slacks > 0)
  {
    // Evaluate constraint violations at current NLP variables
    const Eigen::VectorXd violations = qp_problem.evaluateConvexConstraintViolations(nlp_vars);

    // Get the constraint matrix (row-major)
    const trajopt_ifopt::Jacobian& constraint_matrix = qp_problem.getConstraintMatrix();

    for (Eigen::Index k = 0; k < violations.size(); ++k)
    {
      for (trajopt_ifopt::Jacobian::InnerIterator it(constraint_matrix, k); it; ++it)
      {
        const Eigen::Index col_idx = it.col();
        const double coeff = it.value();

        // Slack variables start at index num_nlp_vars
        if (col_idx >= num_nlp_vars && std::abs(coeff) > 1e-14)
        {
          // Slack is computed as: slack = violation / coefficient
          double slack = violations(k) / coeff;
          // Enforce non-negativity constraint on slack variables
          x0_(col_idx) = std::max(0.0, slack);
        }
      }
    }
  }

  // Initialize dual variables to zero
  y0_.setZero(num_cnts_);

  return true;
}

}  // namespace trajopt_sqp
