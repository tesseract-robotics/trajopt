#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <iostream>
#include <console_bridge/console.h>

namespace trajopt
{
TrustRegionSQPSolver::TrustRegionSQPSolver(OSQPEigenSolver::Ptr qp_solver) : qp_solver_(std::move(qp_solver)) {}

void TrustRegionSQPSolver::Solve(ifopt::Problem& nlp)
{
  qp_solver_->init(nlp);

  // Initialize optimization parameters
  results_ = SQPResults(nlp.GetNumberOfOptimizationVariables(), nlp.GetNumberOfConstraints());
  results_.box_size = Eigen::VectorXd::Ones(nlp.GetNumberOfOptimizationVariables()) * params_.initial_trust_box_size;
  qp_solver_->setBoxSize(results_.box_size);

  // ---------------------------
  // Penalty Iteration Loop
  // ---------------------------
  int overall_iteration = 0;
  for (int penalty_iteration = 0; penalty_iteration < params_.max_merit_coeff_increases; penalty_iteration++)
  {
    // ---------------------------
    // Convexification loop
    // ---------------------------
    for (int convex_iteration = 0; convex_iteration < 100; convex_iteration++)
    {
      // Convexify the costs and constraints around their current values
      qp_solver_->convexify();
      qp_solver_->updateConstraintBounds();
      qp_solver_->updateVariableBounds();

      // ---------------------------
      // Trust region loop
      // ---------------------------
      for (int trust_region_iteration = 0; trust_region_iteration < params_.max_trust_region_expansions;
           trust_region_iteration++)
      {
        overall_iteration++;
        // Take a single step (one QP solve)
        stepOptimization(nlp);

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
          results_.best_var_vals = results_.new_var_vals;  // This is not in trajopt_sco
          results_.best_exact_merit = results_.new_exact_merit;
          results_.best_approx_merit = results_.best_approx_merit;
          break;
        }
        if (results_.approx_merit_improve / results_.best_exact_merit < params_.min_approx_improve_frac)
        {
          CONSOLE_BRIDGE_logInform("Converged because improvement ratio was small (%.3e < %.3e)",
                                   results_.approx_merit_improve / results_.best_exact_merit,
                                   params_.min_approx_improve_frac);
          status_ = SQPStatus::NLP_CONVERGED;
          results_.best_var_vals = results_.new_var_vals;  // This is not in trajopt_sco
          results_.best_exact_merit = results_.new_exact_merit;
          results_.best_approx_merit = results_.best_approx_merit;
          break;
        }

        // Check if the bounding trust region needs to be shrunk
        // This happens if the exact solution got worse or if the QP approximation deviates from the exact by too much
        if (results_.exact_merit_improve < 0 || results_.merit_improve_ratio < params_.improve_ratio_threshold)
        {
          qp_solver_->scaleBoxSize(params_.trust_shrink_ratio);
          qp_solver_->updateVariableBounds();
          results_.box_size = qp_solver_->getBoxSize();

          CONSOLE_BRIDGE_logInform("Shrunk trust region. new box size: %.4f", results_.box_size[0]);
        }
        else
        {
          results_.best_var_vals = results_.new_var_vals;
          results_.best_exact_merit = results_.new_exact_merit;
          results_.best_approx_merit = results_.best_approx_merit;

          qp_solver_->scaleBoxSize(params_.trust_expand_ratio);
          qp_solver_->updateVariableBounds();
          results_.box_size = qp_solver_->getBoxSize();
          CONSOLE_BRIDGE_logInform("Expanded trust region. new box size: %.4f", results_.box_size[0]);
          // TODO: This is not in the shulman paper, but why not try to solve the QP again in this case to get more
          // improvement prior to reconvexifying?
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
      if (overall_iteration >= params_.max_iterations)
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
    if (results_.constraint_violations.size() == 0)
    {
      CONSOLE_BRIDGE_logInform("Optimization has converged and there are no constraints");
      break;
    }

    if (results_.constraint_violations.maxCoeff() < params_.cnt_tolerance)
    {
      CONSOLE_BRIDGE_logInform("woo-hoo! constraints are satisfied (to tolerance %.2e)", params_.cnt_tolerance);
      break;
    }

    if (params_.inflate_constraints_individually)
    {
      assert(results_.constraint_violations.size() == results_.merit_error_coeffs.size());
      for (Eigen::Index idx = 0; idx < results_.constraint_violations.size(); idx++)
      {
        if (results_.constraint_violations[idx] > params_.cnt_tolerance)
        {
          //            CONSOLE_BRIDGE_logInform("Not all constraints are satisfied. Increasing constraint penalties
          //            for %s", CSTR(cnt_names[idx]));
          results_.merit_error_coeffs[idx] *= params_.merit_coeff_increase_ratio;
        }
      }
    }
    else
    {
      CONSOLE_BRIDGE_logInform("Not all constraints are satisfied. Increasing constraint penalties uniformly");
      results_.merit_error_coeffs *= params_.merit_coeff_increase_ratio;
    }
    //      CONSOLE_BRIDGE_logInform("New merit_error_coeffs: %s", CSTR(results_.merit_error_coeffs));
    results_.box_size = Eigen::VectorXd::Ones(results_.box_size.size()) *
                        fmax(results_.box_size[0], params_.min_trust_box_size / params_.trust_shrink_ratio * 1.5);

  }  // Penalty adjustment loop

  // Final Cleanup
  nlp.SetVariables(results_.best_var_vals.data());
  callCallbacks();
}

void TrustRegionSQPSolver::stepOptimization(ifopt::Problem& nlp)
{
  // Solve the QP
  qp_solver_->solve();
  results_.new_var_vals = qp_solver_->getResults();

  // Calculate approximate QP merits (cheap)
  nlp.SetVariables(results_.new_var_vals.data());
  results_.new_approx_merit = qp_solver_->evaluateConvexCost(results_.new_var_vals);
  results_.approx_merit_improve = results_.best_approx_merit - results_.new_approx_merit;

  // Evaluate exact constraint violations (expensive)
  Eigen::VectorXd violations = qp_solver_->getConstraintViolations();

  // Calculate exact NLP merits (expensive) - TODO: Look into caching for qp_solver->Convexify()
  results_.new_exact_merit =
      nlp.EvaluateCostFunction(results_.new_var_vals.data()) + violations.dot(results_.merit_error_coeffs);
  results_.exact_merit_improve = results_.best_exact_merit - results_.new_exact_merit;
  results_.merit_improve_ratio = results_.exact_merit_improve / results_.approx_merit_improve;

  // Store variables if they improved the exact merit
  if (results_.exact_merit_improve > 1e-5)
  {
    results_.best_var_vals = results_.new_var_vals;
    results_.best_exact_merit = results_.new_exact_merit;
    results_.best_approx_merit = results_.new_approx_merit;
  }

  // Call callbacks
  callCallbacks();
}

int TrustRegionSQPSolver::getReturnStatus() { return 0; }
}  // namespace trajopt
