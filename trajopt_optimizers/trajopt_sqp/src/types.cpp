/**
 * @file types.cpp
 * @brief Contains types for the trust region sqp solver
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

#include <trajopt_sqp/types.h>
#include <tesseract_common/utils.h>

#include <iostream>

namespace trajopt_sqp
{
bool SQPParameters::operator==(const SQPParameters& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(improve_ratio_threshold, rhs.improve_ratio_threshold, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(min_trust_box_size, rhs.min_trust_box_size, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(min_approx_improve, rhs.min_approx_improve, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(min_approx_improve_frac, rhs.min_approx_improve_frac, max_diff);
  equal &= (max_iterations == rhs.max_iterations);
  equal &= tesseract_common::almostEqualRelativeAndAbs(trust_shrink_ratio, rhs.trust_shrink_ratio, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(trust_expand_ratio, rhs.trust_expand_ratio, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(cnt_tolerance, rhs.cnt_tolerance, max_diff);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(max_merit_coeff_increases, rhs.max_merit_coeff_increases, max_diff);
  equal &= (max_qp_solver_failures == rhs.max_qp_solver_failures);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(merit_coeff_increase_ratio, rhs.merit_coeff_increase_ratio, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(max_time, rhs.max_time, max_diff);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(initial_merit_error_coeff, rhs.initial_merit_error_coeff, max_diff);
  equal &= (inflate_constraints_individually == rhs.inflate_constraints_individually);
  equal &= tesseract_common::almostEqualRelativeAndAbs(initial_trust_box_size, rhs.initial_trust_box_size, max_diff);
  equal &= (log_results == rhs.log_results);
  equal &= (log_dir == rhs.log_dir);
  // equal &= (num_threads == rhs.num_threads);
  return equal;
}

bool SQPParameters::operator!=(const SQPParameters& rhs) const { return !operator==(rhs); }

SQPResults::SQPResults(Eigen::Index num_vars, Eigen::Index num_cnts, Eigen::Index num_costs)
{
  best_constraint_violations = Eigen::VectorXd::Zero(num_cnts);
  new_constraint_violations = Eigen::VectorXd::Zero(num_cnts);
  best_approx_constraint_violations = Eigen::VectorXd::Zero(num_cnts);
  new_approx_constraint_violations = Eigen::VectorXd::Zero(num_cnts);

  best_costs = Eigen::VectorXd::Zero(num_costs);
  new_costs = Eigen::VectorXd::Zero(num_costs);
  best_approx_costs = Eigen::VectorXd::Zero(num_costs);
  new_approx_costs = Eigen::VectorXd::Zero(num_costs);

  best_var_vals = Eigen::VectorXd::Zero(num_vars);
  new_var_vals = Eigen::VectorXd::Zero(num_vars);
  box_size = Eigen::VectorXd::Ones(num_vars);
  merit_error_coeffs = Eigen::VectorXd::Ones(num_cnts);
}

void SQPResults::print() const
{
  const Eigen::IOFormat format(3);
  std::cout << "-------------- SQPResults::print() --------------" << '\n';
  std::cout << "best_exact_merit: " << best_exact_merit << '\n';
  std::cout << "new_exact_merit: " << new_exact_merit << '\n';
  std::cout << "best_approx_merit: " << best_approx_merit << '\n';
  std::cout << "new_approx_merit: " << new_approx_merit << '\n';

  // NOLINTNEXTLINE
  std::cout << "best_var_vals: " << best_var_vals.transpose().format(format) << '\n';
  std::cout << "new_var_vals: " << new_var_vals.transpose().format(format) << '\n';

  std::cout << "approx_merit_improve: " << approx_merit_improve << '\n';
  std::cout << "exact_merit_improve: " << exact_merit_improve << '\n';
  std::cout << "merit_improve_ratio: " << merit_improve_ratio << '\n';

  std::cout << "box_size: " << box_size.transpose().format(format) << '\n';
  std::cout << "merit_error_coeffs: " << merit_error_coeffs.transpose().format(format) << '\n';

  std::cout << "best_constraint_violations: " << best_constraint_violations.transpose().format(format) << '\n';
  std::cout << "new_constraint_violations: " << new_constraint_violations.transpose().format(format) << '\n';
  std::cout << "best_approx_constraint_violations: " << best_approx_constraint_violations.transpose().format(format)
            << '\n';
  std::cout << "new_approx_constraint_violations: " << new_approx_constraint_violations.transpose().format(format)
            << '\n';

  std::cout << "best_costs: " << best_costs.transpose().format(format) << '\n';
  std::cout << "new_costs: " << new_costs.transpose().format(format) << '\n';
  std::cout << "best_approx_costs: " << best_approx_costs.transpose().format(format) << '\n';
  std::cout << "new_approx_costs: " << new_approx_costs.transpose().format(format) << '\n';

  std::cout << "penalty_iteration: " << penalty_iteration << '\n';
  std::cout << "convexify_iteration: " << convexify_iteration << '\n';
  std::cout << "trust_region_iteration: " << trust_region_iteration << '\n';
  std::cout << "overall_iteration: " << overall_iteration << '\n';
}

}  // namespace trajopt_sqp
