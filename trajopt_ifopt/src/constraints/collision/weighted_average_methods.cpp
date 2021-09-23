/**
 * @file weighted_average_methods.cpp
 * @brief Contains weighted average methods for combining collision results
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
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

#include <trajopt_ifopt/constraints/collision/weighted_average_methods.h>

namespace trajopt_ifopt
{
Eigen::VectorXd getWeightedAvgGradientT0(const GradientResultsSet& grad_results_set,
                                         double max_error_with_buffer,
                                         Eigen::Index size)
{
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(size);
  if (grad_results_set.results.empty())
    return grad_vec;

  assert(max_error_with_buffer > 0);
  double total_weight = 0;
  long cnt{ 0 };
  for (const auto& grad : grad_results_set.results)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (grad.gradients[i].has_gradient &&
          (grad.gradients[i].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time1))
      {
        if (grad_results_set.max_error[i].error_with_buffer[0] > 0)
        {
          assert(grad_results_set.max_error[i].has_error[0]);
          double w = (std::max(grad.error_with_buffer, 0.0) / max_error_with_buffer);
          assert(!(w < 0));
          total_weight += w;
          grad_vec += w * (grad.cc_gradients[i].scale * grad.gradients[i].gradient);
          ++cnt;
        }
      }
    }
  }

  if (cnt == 0)
    return grad_vec;

  assert(total_weight > 0);
  return (1.0 / total_weight) * grad_results_set.coeff * grad_vec;
}

Eigen::VectorXd getWeightedAvgGradientT1(const GradientResultsSet& grad_results_set,
                                         double max_error_with_buffer,
                                         Eigen::Index size)
{
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(size);
  if (grad_results_set.results.empty())
    return grad_vec;

  assert(max_error_with_buffer > 0);
  double total_weight = 0;
  long cnt{ 0 };
  for (const auto& grad : grad_results_set.results)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (grad.cc_gradients[i].has_gradient &&
          (grad.cc_gradients[i].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time0))
      {
        if (grad_results_set.max_error[i].error_with_buffer[1] > 0)
        {
          assert(grad_results_set.max_error[i].has_error[1]);
          double w = (std::max(grad.error_with_buffer, 0.0) / max_error_with_buffer);
          assert(!(w < 0));
          total_weight += w;
          grad_vec += w * (grad.gradients[i].scale * grad.gradients[i].gradient);
          ++cnt;
        }
      }
    }
  }

  if (cnt == 0)
    return grad_vec;

  assert(total_weight > 0);
  return (1.0 / total_weight) * grad_results_set.coeff * grad_vec;
}
}  // namespace trajopt_ifopt
