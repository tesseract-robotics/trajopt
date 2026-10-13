/**
 * @file scripted_qp_solver.h
 * @brief A QP solver wrapper that scripts solve outcomes for tests
 *
 * @author Roelof Oomen
 * @date October 13, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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
#ifndef TRAJOPT_SQP_TEST_SCRIPTED_QP_SOLVER_H
#define TRAJOPT_SQP_TEST_SCRIPTED_QP_SOLVER_H

#include <deque>
#include <memory>
#include <optional>
#include <utility>

#include <trajopt_sqp/qp_solver.h>

namespace trajopt_sqp::test
{
/** @brief One scripted override of a solve; an empty field leaves the inner solver's result as it is */
struct ScriptedSolve
{
  std::optional<bool> succeeded;
};

/** @brief Forwards to a real solver, then applies the front of its script to each solve's outcome */
class ScriptedQPSolver : public QPSolver
{
public:
  explicit ScriptedQPSolver(std::shared_ptr<QPSolver> inner) : inner_(std::move(inner)) {}

  /** @brief Consumed front first, one entry per solve */
  std::deque<ScriptedSolve> script;
  /** @brief Applied to every solve after the script is exhausted */
  ScriptedSolve every_solve;
  int solves{ 0 };

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts) override { return inner_->init(num_vars, num_cnts); }
  bool clear() override { return inner_->clear(); }
  bool solve() override
  {
    ++solves;
    const bool succeeded = inner_->solve();
    ScriptedSolve step = every_solve;
    if (!script.empty())
    {
      step = script.front();
      script.pop_front();
    }
    return step.succeeded.value_or(succeeded);
  }
  Eigen::VectorXd getSolution() override { return inner_->getSolution(); }
  bool updateHessianMatrix(const trajopt_ifopt::Jacobian& hessian) override
  {
    return inner_->updateHessianMatrix(hessian);
  }
  bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient) override
  {
    return inner_->updateGradient(gradient);
  }
  bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lower) override
  {
    return inner_->updateLowerBound(lower);
  }
  bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upper) override
  {
    return inner_->updateUpperBound(upper);
  }
  bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lower,
                    const Eigen::Ref<const Eigen::VectorXd>& upper) override
  {
    return inner_->updateBounds(lower, upper);
  }
  bool updateLinearConstraintsMatrix(const trajopt_ifopt::Jacobian& matrix) override
  {
    return inner_->updateLinearConstraintsMatrix(matrix);
  }
  bool setWarmStart(const QPProblem& qp_problem) override { return inner_->setWarmStart(qp_problem); }
  QPSolverStatus getSolverStatus() const override { return inner_->getSolverStatus(); }

private:
  std::shared_ptr<QPSolver> inner_;
};
}  // namespace trajopt_sqp::test

#endif
