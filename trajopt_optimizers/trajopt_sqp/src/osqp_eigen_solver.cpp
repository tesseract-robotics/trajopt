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
#include <trajopt_common/macros.h>

const bool OSQP_COMPARE_DEBUG_MODE = false;

namespace trajopt_sqp
{
OSQPEigenSolver::OSQPEigenSolver()
{
  if (verbosity > 0)
    solver_.settings()->setVerbosity(true);
  else
    solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setPolish(true);
  solver_.settings()->setAdaptiveRho(true);
  solver_.settings()->setMaxIteration(8192);
  solver_.settings()->setAbsoluteTolerance(1e-4);
  solver_.settings()->setRelativeTolerance(1e-6);
}

bool OSQPEigenSolver::init(Eigen::Index num_vars, Eigen::Index num_cnts)
{
  // Set the solver size
  num_cnts_ = num_cnts;
  num_vars_ = num_vars;
  solver_.data()->setNumberOfVariables(static_cast<int>(num_vars_));
  solver_.data()->setNumberOfConstraints(static_cast<int>(num_cnts_));

  solver_status_ = QPSolverStatus::INITIALIZED;

  return true;
}

bool OSQPEigenSolver::clear()
{
  // Clear all data
  solver_.clearSolver();
  solver_.data()->clearHessianMatrix();
  solver_.data()->clearLinearConstraintsMatrix();
  return true;
}

bool OSQPEigenSolver::solve()
{
  // In order to call initSolver, everything must have already been set, so we call it right before solving
  if (!solver_.isInitialized())  // NOLINT
    solver_.initSolver();

  if (OSQP_COMPARE_DEBUG_MODE)
  {
    Eigen::IOFormat format(5);
    const auto* osqp_data = solver_.data()->getData();
    std::cout << "OSQP Number of Variables:" << osqp_data->n << std::endl;
    std::cout << "OSQP Number of Constraints:" << osqp_data->m << std::endl;

    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> P_p_vec(osqp_data->P->p, osqp_data->P->n + 1);
    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> P_i_vec(osqp_data->P->i, osqp_data->P->nzmax);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> P_x_vec(osqp_data->P->x, osqp_data->P->nzmax);
    std::cout << "OSQP Hessian:" << std::endl;
    std::cout << "     nzmax:" << osqp_data->P->nzmax << std::endl;
    std::cout << "        nz:" << osqp_data->P->nz << std::endl;
    std::cout << "         m:" << osqp_data->P->m << std::endl;
    std::cout << "         n:" << osqp_data->P->n << std::endl;
    std::cout << "         p:" << P_p_vec.transpose().format(format) << std::endl;
    std::cout << "         i:" << P_i_vec.transpose().format(format) << std::endl;
    std::cout << "         x:" << P_x_vec.transpose().format(format) << std::endl;

    Eigen::Map<Eigen::VectorXd> q_vec(osqp_data->q, osqp_data->n);
    std::cout << "OSQP Gradient: " << q_vec.transpose().format(format) << std::endl;

    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> A_p_vec(osqp_data->A->p, osqp_data->A->n + 1);
    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> A_i_vec(osqp_data->A->i, osqp_data->A->nzmax);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> A_x_vec(osqp_data->A->x, osqp_data->A->nzmax);
    std::cout << "OSQP Constraint Matrix:" << std::endl;
    std::cout << "     nzmax:" << osqp_data->A->nzmax << std::endl;
    std::cout << "         m:" << osqp_data->A->m << std::endl;
    std::cout << "         n:" << osqp_data->A->n << std::endl;
    std::cout << "         p:" << A_p_vec.transpose().format(format) << std::endl;
    std::cout << "         i:" << A_i_vec.transpose().format(format) << std::endl;
    std::cout << "         x:" << A_x_vec.transpose().format(format) << std::endl;

    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l_vec(osqp_data->l, osqp_data->m);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u_vec(osqp_data->u, osqp_data->m);
    std::cout << "OSQP Lower Bounds: " << l_vec.transpose().format(format) << std::endl;
    std::cout << "OSQP Upper Bounds: " << u_vec.transpose().format(format) << std::endl;

    std::cout << "OSQP Variable Names: " << std::endl;
  }

  /** @todo Need to check if this is what we want in the new version */
  if (solver_.solve() || solver_.workspace()->info->status_val == OSQP_SOLVED_INACCURATE)
  {
    if (OSQP_COMPARE_DEBUG_MODE)
    {
      Eigen::IOFormat format(5);
      std::cout << "OSQP Solution: " << solver_.getSolution().transpose().format(format) << std::endl;
    }
    return true;
  }

  if (verbosity > 0)  // NOLINT
  {
    // If primal infeasible
    if (solver_.workspace()->info->status_val == -3)
    {
      Eigen::Map<Eigen::VectorXd> primal_certificate(solver_.workspace()->delta_x, num_cnts_, 1);
      std::cout << "OSQP Status: " << solver_.workspace()->info->status << std::endl;
      std::cout << "\n---------------------------------------\n";
      std::cout << std::scientific << "Primal Certificate (v): " << primal_certificate.transpose() << std::endl;

      double first = bounds_lower_.transpose() * primal_certificate;
      double second = bounds_upper_.transpose() * primal_certificate;

      std::cout << "A.transpose() * v = 0\n"
                << "l.transpose() * v = " << first << "    u.transpose() * v = " << second << std::endl;
      std::cout << "l.transpose() * v + u.transpose() * v  = " << first + second << " < 0\n";
      std::cout << "Bounds_lower: " << bounds_lower_.transpose() << std::endl;
      std::cout << "Bounds_upper: " << bounds_upper_.transpose() << std::endl;
      std::cout << std::fixed;
      std::cout << "\n---------------------------------------\n";
    }

    // If dual infeasible
    if (solver_.workspace()->info->status_val == -4)  // NOLINT
    {
      Eigen::Map<Eigen::VectorXd> dual_certificate(solver_.workspace()->delta_y, num_vars_, 1);
      std::cout << "OSQP Status: " << solver_.workspace()->info->status << std::endl;
      std::cout << "\n---------------------------------------\n";
      std::cout << "Dual Certificate (x): " << dual_certificate.transpose() << std::endl;  // NOLINT

      // TODO: Find a way to get the hessian here
      //        std::cout << "P*x = " << (qp_problem_->getHessian() * dual_certificate).transpose() << " = 0" <<
      //        std::endl;
      std::cout << "q.transpose() * x = " << gradient_.transpose() * dual_certificate << " < 0" << std::endl;
      std::cout << std::fixed;
      std::cout << "\n---------------------------------------\n";
    }
  }

  solver_status_ = QPSolverStatus::QP_ERROR;
  return false;
}

Eigen::VectorXd OSQPEigenSolver::getSolution()
{
  Eigen::VectorXd solution = solver_.getSolution();
  return solution;
}

// This code is required because there is a bug in OSQP 0.6.X scs_spalloc method
/** @todo Remove when upgrading to OSQP 1.0.0 */
TRAJOPT_IGNORE_WARNINGS_PUSH
static void* csc_malloc(c_int n, c_int size)
{
  return c_malloc(n * size);  // NOLINT
}

static void* csc_calloc(c_int n, c_int size)
{
  return c_calloc(n, size);  // NOLINT
}

void csc_spfree_fix(csc* A)
{
  if (A)  // NOLINT
  {
    if (A->p)        // NOLINT
      c_free(A->p);  // NOLINT
    if (A->i)        // NOLINT
      c_free(A->i);  // NOLINT
    if (A->x)        // NOLINT
      c_free(A->x);  // NOLINT
    c_free(A);       // NOLINT
  }
}

csc* csc_spalloc_fix(c_int m, c_int n, c_int nzmax, c_int values, c_int triplet)
{
  csc* A = (csc*)(csc_calloc(1, sizeof(csc))); /* allocate the csc struct */  // NOLINT

  if (!A)           // NOLINT
    return nullptr; /* out of memory */

  A->m = m; /* define dimensions and nzmax */
  A->n = n;
  A->nzmax = nzmax = c_max(nzmax, 0);
  A->nz = triplet ? 0 : -1; /* allocate triplet or comp.col */          // NOLINT
  A->p = (c_int*)(csc_malloc(triplet ? nzmax : n + 1, sizeof(c_int)));  // NOLINT
  for (int i = 0; i < n + 1; ++i)
    A->p[i] = 0;
  A->i = values ? (c_int*)(csc_malloc(nzmax, sizeof(c_int))) : nullptr;      // NOLINT
  A->x = values ? (c_float*)(csc_malloc(nzmax, sizeof(c_float))) : nullptr;  // NOLINT
  if (!A->p || (values && !A->i) || (values && !A->x))                       // NOLINT
  {
    csc_spfree_fix(A);
    return nullptr;
  }

  return A;  // NOLINT
}
TRAJOPT_IGNORE_WARNINGS_POP

bool OSQPEigenSolver::updateHessianMatrix(const SparseMatrix& hessian)
{
  // Clean up values close to 0
  // Also multiply by 2 because OSQP is multiplying by (1/2) for the objective fuction
  SparseMatrix cleaned = 2.0 * hessian.pruned(1e-7, 1);  // Any value < 1e-7 will be removed

  if (solver_.isInitialized())
  {
    bool success = solver_.updateHessianMatrix(cleaned);
    if (cleaned.nonZeros() == 0) /** @todo Remove when upgrading to OSQP 1.0.0 */
    {
      csc_spfree_fix(solver_.data()->getData()->P);
      solver_.data()->getData()->P = nullptr;
      solver_.data()->getData()->P = csc_spalloc_fix(cleaned.rows(), cleaned.cols(), 0, 1, 0);
    }
    return success;
  }

  solver_.data()->clearHessianMatrix();
  bool success = solver_.data()->setHessianMatrix(cleaned);
  if (cleaned.nonZeros() == 0) /** @todo Remove when upgrading to OSQP 1.0.0 */
  {
    csc_spfree_fix(solver_.data()->getData()->P);
    solver_.data()->getData()->P = nullptr;
    solver_.data()->getData()->P = csc_spalloc_fix(cleaned.rows(), cleaned.cols(), 0, 1, 0);
  }

  return success;
}

bool OSQPEigenSolver::updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient)
{
  gradient_ = gradient;

  if (solver_.isInitialized())
    return solver_.updateGradient(gradient_);

  return solver_.data()->setGradient(gradient_);
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

  bool success = solver_.data()->setLowerBound(bounds_lower_);
  success &= solver_.data()->setUpperBound(bounds_upper_);
  return success;
}

bool OSQPEigenSolver::updateLinearConstraintsMatrix(const SparseMatrix& linearConstraintsMatrix)
{
  assert(num_cnts_ == linearConstraintsMatrix.rows());
  assert(num_vars_ == linearConstraintsMatrix.cols());

  solver_.data()->clearLinearConstraintsMatrix();
  SparseMatrix cleaned = linearConstraintsMatrix.pruned(1e-7, 1);  // Any value < 1e-7 will be removed

  if (solver_.isInitialized())
  {
    bool success = solver_.updateLinearConstraintsMatrix(cleaned);
    if (cleaned.nonZeros() == 0) /** @todo Remove when upgrading to OSQP 1.0.0 */
    {
      csc_spfree(solver_.data()->getData()->A);
      solver_.data()->getData()->A = nullptr;
      solver_.data()->getData()->A = csc_spalloc_fix(cleaned.rows(), cleaned.cols(), 0, 1, 0);
    }
    return success;
  }

  bool success = solver_.data()->setLinearConstraintsMatrix(cleaned);
  if (cleaned.nonZeros() == 0) /** @todo Remove when upgrading to OSQP 1.0.0 */
  {
    csc_spfree(solver_.data()->getData()->A);
    solver_.data()->getData()->A = nullptr;
    solver_.data()->getData()->A = csc_spalloc_fix(cleaned.rows(), cleaned.cols(), 0, 1, 0);
  }
  return success;
}

}  // namespace trajopt_sqp
