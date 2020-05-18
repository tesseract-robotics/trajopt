/**
 * @file trust_region_sqp_solver.cpp
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
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <iostream>
#include <console_bridge/console.h>

namespace trajopt_sqp
{
const bool SUPER_DEBUG_MODE = false;

TrustRegionSQPSolver::TrustRegionSQPSolver(OSQPEigenSolver::Ptr qp_solver) : qp_solver_(std::move(qp_solver))
{
  qp_problem_ = std::make_shared<QPProblem>();
}

void TrustRegionSQPSolver::Solve(ifopt::Problem& nlp)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  nlp_ = &nlp;

  qp_problem_->init(nlp);
  qp_problem_->convexify();

  // TODO: Clean up the solver interface and switch to using base class here.
  qp_solver_->solver_.data()->setNumberOfVariables(static_cast<int>(qp_problem_->num_qp_vars_));
  qp_solver_->solver_.data()->setNumberOfConstraints(static_cast<int>(qp_problem_->num_qp_cnts_));
  qp_solver_->solver_.data()->setHessianMatrix(qp_problem_->hessian_);
  qp_solver_->solver_.data()->setGradient(qp_problem_->gradient_);
  Jacobian cleaned = qp_problem_->constraint_matrix_.pruned(1e-7);
  qp_solver_->solver_.data()->setLinearConstraintsMatrix(cleaned);
  qp_solver_->solver_.data()->setUpperBound(qp_problem_->bounds_upper_);
  qp_solver_->solver_.data()->setLowerBound(qp_problem_->bounds_lower_);

  //  qp_solver_->updateHessianMatrix(qp_problem_->hessian_);
  //  qp_solver_->updateGradient(qp_problem_->gradient_);
  //  qp_solver_->updateLinearConstraintsMatrix(qp_problem_->constraint_matrix_);
  //  qp_solver_->updateBounds(qp_problem_->bounds_lower_, qp_problem_->bounds_upper_);
  qp_solver_->init(qp_problem_->num_qp_vars_, qp_problem_->num_qp_cnts_);

  // Initialize optimization parameters
  results_ = SQPResults(nlp.GetNumberOfOptimizationVariables(), nlp.GetNumberOfConstraints());
  results_.box_size = Eigen::VectorXd::Ones(nlp.GetNumberOfOptimizationVariables()) * params_.initial_trust_box_size;
  qp_problem_->setBoxSize(results_.box_size);

  // ---------------------------
  // Penalty Iteration Loop
  // ---------------------------
  for (int penalty_iteration = 0; penalty_iteration < params_.max_merit_coeff_increases; penalty_iteration++)
  {
    // ---------------------------
    // Convexification loop
    // ---------------------------
    for (int convex_iteration = 0; convex_iteration < 100; convex_iteration++)
    {
      qp_problem_->convexify();

      //      qp_solver_->clear();
      //      qp_solver_->updateHessianMatrix(qp_problem_->hessian_);
      //      qp_solver_->updateGradient(qp_problem_->gradient_);
      //      qp_solver_->updateLinearConstraintsMatrix(qp_problem_->constraint_matrix_);
      //      qp_solver_->updateBounds(qp_problem_->bounds_lower_, qp_problem_->bounds_upper_);
      //      qp_solver_->init(qp_problem_->num_qp_vars_, qp_problem_->num_qp_cnts_);

      // Convexify the costs and constraints around their current values
      qp_solver_->solver_.clearSolver();
      qp_solver_->solver_.data()->clearHessianMatrix();
      qp_solver_->solver_.data()->clearLinearConstraintsMatrix();

      qp_solver_->solver_.data()->setNumberOfVariables(static_cast<int>(qp_problem_->num_qp_vars_));
      qp_solver_->solver_.data()->setNumberOfConstraints(static_cast<int>(qp_problem_->num_qp_cnts_));
      qp_solver_->solver_.data()->setHessianMatrix(qp_problem_->hessian_);
      qp_solver_->solver_.data()->setGradient(qp_problem_->gradient_);
      Jacobian cleaned = qp_problem_->constraint_matrix_.pruned(1e-7);

      qp_solver_->solver_.data()->setLinearConstraintsMatrix(cleaned);
      qp_solver_->solver_.data()->setUpperBound(qp_problem_->bounds_upper_);
      qp_solver_->solver_.data()->setLowerBound(qp_problem_->bounds_lower_);
      qp_solver_->init(qp_problem_->num_qp_vars_, qp_problem_->num_qp_cnts_);

      if (SUPER_DEBUG_MODE)
        qp_problem_->print();

      // ---------------------------
      // Trust region loop
      // ---------------------------
      for (int trust_region_iteration = 0; trust_region_iteration < params_.max_trust_region_expansions;
           trust_region_iteration++)
      {
        results_.overall_iteration++;
        results_.penalty_iteration = penalty_iteration;
        results_.convexify_iteration = convex_iteration;
        results_.trust_region_iteration = trust_region_iteration;

        // Take a single step (one QP solve)
        if (!stepOptimization(nlp))
        {
          nlp.SetVariables(results_.best_var_vals.data());
          status_ = SQPStatus::CALLBACK_STOPPED;
          return;
        }

        // Check if the entire NLP Converged
        if (results_.approx_merit_improve < -1e-5)
        {
          CONSOLE_BRIDGE_logError("Approximate merit function got worse (%.3e). (convexification is probably wrong to "
                                  "zeroth order)",
                                  results_.approx_merit_improve);
        }
        if (results_.approx_merit_improve < params_.min_approx_improve)
        {
          CONSOLE_BRIDGE_logInform("Converged because improvement was small (%.3e < %.3e)",
                                   results_.approx_merit_improve,
                                   params_.min_approx_improve);
          status_ = SQPStatus::NLP_CONVERGED;
          break;
        }
        if (results_.approx_merit_improve / results_.best_exact_merit < params_.min_approx_improve_frac)
        {
          CONSOLE_BRIDGE_logInform("Converged because improvement ratio was small (%.3e < %.3e)",
                                   results_.approx_merit_improve / results_.best_exact_merit,
                                   params_.min_approx_improve_frac);
          status_ = SQPStatus::NLP_CONVERGED;
          break;
        }

        // Check if the bounding trust region needs to be shrunk
        // This happens if the exact solution got worse or if the QP approximation deviates from the exact by too much
        if (results_.exact_merit_improve < 0 || results_.merit_improve_ratio < params_.improve_ratio_threshold)
        {
          qp_problem_->scaleBoxSize(params_.trust_shrink_ratio);
          qp_problem_->updateNLPVariableBounds();
          qp_solver_->updateBounds(qp_problem_->bounds_lower_, qp_problem_->bounds_upper_);
          results_.box_size = qp_problem_->getBoxSize();

          CONSOLE_BRIDGE_logInform("Shrunk trust region. new box size: %.4f", results_.box_size[0]);
        }
        else
        {
          results_.best_var_vals = results_.new_var_vals;
          results_.best_exact_merit = results_.new_exact_merit;
          results_.best_approx_merit = results_.best_approx_merit;

          qp_problem_->scaleBoxSize(params_.trust_expand_ratio);
          qp_problem_->updateNLPVariableBounds();
          results_.box_size = qp_problem_->getBoxSize();
          CONSOLE_BRIDGE_logInform("Expanded trust region. new box size: %.4f", results_.box_size[0]);
          break;
        }
      }  // Trust region loop

      // Check if the NLP has converged
      if (status_ == SQPStatus::NLP_CONVERGED)
        break;
      if (results_.box_size.maxCoeff() < params_.min_trust_box_size)
      {
        CONSOLE_BRIDGE_logInform("Converged because trust region is tiny");
        status_ = SQPStatus::NLP_CONVERGED;
        break;
      }
      if (results_.overall_iteration >= params_.max_iterations)
      {
        CONSOLE_BRIDGE_logInform("Iteration limit");
        status_ = SQPStatus::ITERATION_LIMIT;
        break;
      }
    }

    // ---------------------------
    // Penalty Adjustment
    // ---------------------------

    // Check if constrainsts are satisfied
    if (results_.new_constraint_violations.size() == 0)
    {
      CONSOLE_BRIDGE_logInform("Optimization has converged and there are no constraints");
      break;
    }

    if (results_.new_constraint_violations.maxCoeff() < params_.cnt_tolerance)
    {
      CONSOLE_BRIDGE_logInform("woo-hoo! constraints are satisfied (to tolerance %.2e)", params_.cnt_tolerance);
      break;
    }

    if (params_.inflate_constraints_individually)
    {
      assert(results_.new_constraint_violations.size() == results_.merit_error_coeffs.size());
      for (Eigen::Index idx = 0; idx < results_.new_constraint_violations.size(); idx++)
      {
        if (results_.new_constraint_violations[idx] > params_.cnt_tolerance)
        {
          CONSOLE_BRIDGE_logInform("Not all constraints are satisfied. Increasing constraint penalties for %d", idx);
          results_.merit_error_coeffs[idx] *= params_.merit_coeff_increase_ratio;
        }
      }
    }
    else
    {
      CONSOLE_BRIDGE_logInform("Not all constraints are satisfied. Increasing constraint penalties uniformly");
      results_.merit_error_coeffs *= params_.merit_coeff_increase_ratio;
    }
    results_.box_size = Eigen::VectorXd::Ones(results_.box_size.size()) *
                        fmax(results_.box_size[0], params_.min_trust_box_size / params_.trust_shrink_ratio * 1.5);

  }  // Penalty adjustment loop

  // Final Cleanup
  nlp.SetVariables(results_.best_var_vals.data());
}

bool TrustRegionSQPSolver::stepOptimization(ifopt::Problem& nlp)
{
  // TODO: Think about removing nlp_
  nlp_ = &nlp;

  // Solve the QP
  bool succeed = qp_solver_->solve();
  if (succeed)
  {
    results_.new_var_vals = qp_solver_->getSolution();

    // Calculate approximate QP merits (cheap)
    nlp.SetVariables(results_.new_var_vals.data());
    results_.new_approx_merit = qp_problem_->evaluateTotalConvexCost(results_.new_var_vals);
    results_.approx_merit_improve = results_.best_approx_merit - results_.new_approx_merit;

    // Evaluate exact constraint violations (expensive)
    results_.new_constraint_violations = qp_problem_->getExactConstraintViolations();

    // Calculate exact NLP merits (expensive) - TODO: Look into caching for qp_solver->Convexify()
    results_.new_exact_merit = nlp.EvaluateCostFunction(results_.new_var_vals.data()) +
                               results_.new_constraint_violations.dot(results_.merit_error_coeffs);
    results_.exact_merit_improve = results_.best_exact_merit - results_.new_exact_merit;
    results_.merit_improve_ratio = results_.exact_merit_improve / results_.approx_merit_improve;

    // Print debugging info
    if (verbose_)
      printStepInfo();

    // Call callbacks
    succeed &= callCallbacks();

    // Store variables if they improved the exact merit
    if (results_.exact_merit_improve > 1e-5)
    {
      results_.best_var_vals = results_.new_var_vals;
      results_.best_exact_merit = results_.new_exact_merit;
      results_.best_approx_merit = results_.new_approx_merit;
      results_.best_constraint_violations = results_.new_constraint_violations;
      nlp_->SetVariables(results_.best_var_vals.data());
    }
    if (SUPER_DEBUG_MODE)
      results_.print();
  }
  else
  {
    CONSOLE_BRIDGE_logError("Solver Failure");

    //    // If primal infeasible
    //    {
    //      Eigen::Map<Eigen::VectorXd> primal_certificate(
    //          qp_solver_->solver_.workspace()->delta_x, qp_problem_->num_qp_cnts_, 1);
    //      std::cout << "\n---------------------------------------\n";
    //      std::cout << std::scientific << "Primal Certificate (v): " << primal_certificate.transpose() << std::endl;

    //      double first = qp_problem_->bounds_lower_.transpose() * primal_certificate;
    //      double second = qp_problem_->bounds_upper_.transpose() * primal_certificate;

    //      std::cout << "A.transpose() * v = 0\n"
    //                << "l.transpose() * v = " << first << "    u.transpose() * v = " << second << std::endl;
    //      std::cout << "l.transpose() * v + u.transpose() * v  = " << first + second << " < 0\n";
    //      std::cout << "Bounds_lower: " << qp_problem_->bounds_lower_.transpose() << std::endl;
    //      std::cout << "Bounds_upper: " << qp_problem_->bounds_upper_.transpose() << std::endl;
    //      std::cout << std::fixed;
    //      std::cout << "\n---------------------------------------\n";
    //    }

    //    // If dual infeasible
    //    {
    //      Eigen::Map<Eigen::VectorXd> dual_certificate(
    //          qp_solver_->solver_.workspace()->delta_y, qp_problem_->num_qp_vars_, 1);
    //      std::cout << "\n---------------------------------------\n";
    //      std::cout << "Dual Certificate (x): " << dual_certificate.transpose() << std::endl;

    //      std::cout << "P*x = " << (qp_problem_->hessian_ * dual_certificate).transpose() << " = 0" << std::endl;
    //      std::cout << "q.transpose() * x = " << qp_problem_->gradient_.transpose() * dual_certificate << " < 0"
    //                << std::endl;
    //      std::cout << std::fixed;
    //      std::cout << "\n---------------------------------------\n";
    //    }
  }

  return succeed;
}

bool TrustRegionSQPSolver::callCallbacks()
{
  bool success = true;
  for (const auto& callback : callbacks_)
    success &= callback->execute(*nlp_, results_);
  return success;
}

void TrustRegionSQPSolver::printStepInfo() const
{
  // Print Header
  std::printf("\n%15s | %10s===%10s===%10s===%10s===%10s===%10s\n",
              "",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========");
  std::printf("                |                              ROS Industrial \n");
  std::printf("                |                         TrajOpt Motion Planning \n");
  std::printf("%15s | %10s===%10s===%10s===%10s===%10s===%10s\n",
              "",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========");
  std::printf("%15s | %11s %1d/%2d/%2d/%3d\n",
              "",
              "Iteration:",
              results_.penalty_iteration,
              results_.convexify_iteration,
              results_.trust_region_iteration,
              results_.overall_iteration);
  std::printf("%15s | %10s===%10s===%10s===%10s===%10s===%10s\n",
              "",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========");
  std::printf("%15s | %10s | %10s | %10s | %10s | %10s | %10s\n",
              "",
              "merit",
              "oldexact",
              "new_exact",
              "dapprox",
              "dexact",
              "ratio");
  // Costs
  std::printf("%15s | %10s---%10s---%10s---%10s---%10s---%10s\n",
              "COSTS",
              "----------",
              "----------",
              "----------",
              "----------",
              "----------",
              "----------");
  std::vector<ifopt::Component::Ptr> costs = nlp_->GetCosts().GetComponents();
  // Loop over cost sets
  Eigen::Index cost_number = 0;
  for (const auto& cost : costs)
  {
    // Loop over each constraint in the set
    for (Eigen::Index j = 0; j < cost->GetRows(); j++)
    {
      double approx_improve = 0;  // old_cost_vals[i] - model_cost_vals[i];
      double exact_improve = 0;   // old_cost_vals[i] - new_cost_vals[i];
      if (fabs(approx_improve) > 1e-8)
        std::printf("%15s | %10s | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e\n",
                    (cost->GetName() + "_" + std::to_string(j)).c_str(),
                    "----------",
                    results_.best_exact_merit,
                    results_.new_exact_merit,
                    approx_improve,
                    exact_improve,
                    exact_improve / approx_improve);
      else
        std::printf("%15s | %10s | %10.3e | %10.3e | %10.3e | %10.3e | %10s\n",
                    (cost->GetName() + "_" + std::to_string(j)).c_str(),
                    "----------",
                    results_.best_exact_merit,
                    results_.new_exact_merit,
                    approx_improve,
                    exact_improve,
                    "  ------  ");
      cost_number++;
    }
  }

  // Constraints
  // If we want to print the names we will have to add a getConstraints function to IFOPT
  if (results_.new_constraint_violations.size() != 0)
  {
    std::printf("%15s | %10s---%10s---%10s---%10s---%10s---%10s\n",
                "CONSTRAINTS",
                "----------",
                "----------",
                "----------",
                "----------",
                "----------",
                "----------");
    Eigen::VectorXd exact_cnt_improve = results_.best_constraint_violations - results_.new_constraint_violations;
    std::vector<ifopt::Component::Ptr> constraints = nlp_->GetConstraints().GetComponents();
    // Loop over constraint sets
    Eigen::Index cnt_number = 0;
    for (const auto& cnt : constraints)
    {
      // Loop over each constraint in the set
      for (Eigen::Index j = 0; j < cnt->GetRows(); j++)
      {
        double approx_improve = 0;  // old_cnt_viols[i] - model_cnt_viols[i];  // TODO
        if (fabs(approx_improve) > 1e-8)
          std::printf("%15s | %10.3e | %10.3e | %10.3e | %10s | %10.3e | %10.3e\n",
                      (cnt->GetName() + "_" + std::to_string(j)).c_str(),
                      results_.merit_error_coeffs[cnt_number],
                      results_.merit_error_coeffs[cnt_number] * results_.best_constraint_violations[cnt_number],
                      results_.merit_error_coeffs[cnt_number] * results_.new_constraint_violations[cnt_number],
                      "  ------  ",
                      results_.merit_error_coeffs[cnt_number] * exact_cnt_improve[cnt_number],
                      exact_cnt_improve[cnt_number] / approx_improve);
        else
          std::printf("%15s | %10.3e | %10.3e | %10.3e | %10s | %10.3e | %10s\n",
                      (cnt->GetName() + "_" + std::to_string(j)).c_str(),
                      results_.merit_error_coeffs[cnt_number],
                      results_.merit_error_coeffs[cnt_number] * results_.best_constraint_violations[cnt_number],
                      results_.merit_error_coeffs[cnt_number] * results_.new_constraint_violations[cnt_number],
                      "  ------  ",
                      results_.merit_error_coeffs[cnt_number] * exact_cnt_improve[cnt_number],
                      "  ------  ");
        cnt_number++;
      }
    }
  }

  // Total
  std::printf("%15s | %10s===%10s===%10s===%10s===%10s===%10s\n",
              "",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========",
              "==========");
  std::printf("%15s | %10s | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e\n",
              "TOTAL",
              "----------",
              results_.best_exact_merit,
              results_.new_exact_merit,
              results_.approx_merit_improve,
              results_.exact_merit_improve,
              results_.merit_improve_ratio);
}

}  // namespace trajopt_sqp
