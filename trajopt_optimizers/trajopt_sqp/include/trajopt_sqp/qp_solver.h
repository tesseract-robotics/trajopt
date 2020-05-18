/**
 * @file qp_solver.h
 * @brief Base class for QP solvers
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug This is not being used currently. The OSQPEigen solver interface needs to be cleaned up such that this base
 * class can be used in trust_region_sqp_solver.h
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
#ifndef TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_

#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
enum class QP_SOLVER_STATUS
{
  UNITIALIZED,
  INITIALIZED,
  ERROR
};

/**
 * @brief Base class for Quadratic programming solvers
 */
class QPSolver
{
public:
  using Ptr = std::shared_ptr<QPSolver>;
  using ConstPtr = std::shared_ptr<const QPSolver>;

  //  virtual bool init() = 0;

  //  virtual bool solve() = 0;

  //  virtual Eigen::VectorXd getSolution() = 0;

  //  virtual bool updateHessianMatrix(const Hessian hessianMatrix) = 0;

  //  virtual bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>> &gradient) = 0;

  //  virtual bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>> &lowerBound) = 0;

  //  virtual bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>> &upperBound) = 0;

  //  virtual bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>> &lowerBound,
  //                            const Eigen::Ref<const Eigen::VectorXd>> &upperBound) = 0;

  //  virtual bool updateLinearConstraintsMatrix(const Jacobian& linearConstraintsMatrix) = 0;

  //  virtual QP_SOLVER_STATUS getSolverStatus() = 0;
};
}  // namespace trajopt_sqp

#endif
