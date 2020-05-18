/**
 * @file trust_region_sqp_solver.h
 * @brief Contains the main trust region SQP solver. While it is based on the paper below, it has been completely
 * rewritten from trajopt_sco
 *
 * Schulman, J., Ho, J., Lee, A. X., Awwal, I., Bradlow, H., & Abbeel, P. (2013, June). Finding Locally Optimal,
 * Collision-Free Trajectories with Sequential Convex Optimization. In Robotics: science and systems (Vol. 9, No. 1, pp.
 * 1-10).
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
#ifndef TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/solver.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/qp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/sqp_callback.h>

namespace trajopt_sqp
{
/**
 * @brief A simple SQP Solver that uses the QPSolver passed in
 */
class TrustRegionSQPSolver : public ifopt::Solver
{
public:
  using Ptr = std::shared_ptr<TrustRegionSQPSolver>;
  using ConstPtr = std::shared_ptr<const TrustRegionSQPSolver>;

  TrustRegionSQPSolver(OSQPEigenSolver::Ptr qp_solver);

  void Solve(ifopt::Problem& nlp) override;

  /**
   * @brief Take a single QP optimization step, storing the results and calling callbacks
   * @param nlp
   */
  bool stepOptimization(ifopt::Problem& nlp);

  bool callCallbacks();

  void printStepInfo() const;

  SQPStatus status_;
  SQPResults results_;
  SQPParameters params_;

  void registerCallback(const SQPCallback::Ptr& callback) { callbacks_.push_back(callback); };
  std::vector<SQPCallback::Ptr> callbacks_;

  OSQPEigenSolver::Ptr qp_solver_;
  QPProblem::Ptr qp_problem_;

  bool verbose_{ false };

private:
  ifopt::Problem* nlp_;
};

}  // namespace trajopt_sqp
#endif
