/**
 * @file osqp_eigen_solver.h
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
#ifndef TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <OsqpEigen/Settings.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/qp_solver.h>

namespace OsqpEigen
{
class Solver;
}  // namespace OsqpEigen

namespace trajopt_sqp
{
class QPProblem;

/**
 * @brief An Interface to the OSQPEigen QP Solver
 */
class OSQPEigenSolver : public QPSolver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  OSQPEigenSolver();
  ~OSQPEigenSolver() override;
  OSQPEigenSolver(const OSQPEigenSolver&) = delete;
  OSQPEigenSolver& operator=(const OSQPEigenSolver&) = delete;
  OSQPEigenSolver(OSQPEigenSolver&&) = default;
  OSQPEigenSolver& operator=(OSQPEigenSolver&&) = default;

  static void setDefaultOSQPSettings(OsqpEigen::Settings& settings);

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts) override;

  bool clear() override;

  bool solve() override;

  Eigen::VectorXd getSolution() override;

  bool updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian) override;

  bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient) override;

  bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound) override;

  bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                    const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  bool updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& linearConstraintsMatrix) override;

  bool setWarmStart(const QPProblem& qp_problem) override;

  QPSolverStatus getSolverStatus() const override { return solver_status_; }

  std::unique_ptr<OsqpEigen::Solver> solver_;

private:
  // Depending on what they decide to do with this issue, these could be dropped
  // https://github.com/robotology/osqp-eigen/issues/17
  Eigen::VectorXd x0_;
  Eigen::VectorXd y0_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  Eigen::VectorXd gradient_;
  Eigen::Index num_vars_{ 0 };
  Eigen::Index num_cnts_{ 0 };

  QPSolverStatus solver_status_{ QPSolverStatus::kUninitialized };
};

}  // namespace trajopt_sqp

#endif
