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
Eigen::VectorXd getWeightedAvgValuesT0(const GradientResultsSet& grad_results_set)
{
  assert(grad_results_set.dof > 0);
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  if (grad_results_set.results.empty())
    return Eigen::VectorXd::Constant(1, -grad_results_set.collision_margin_buffer);

  if (!(grad_results_set.max_error > 0))
    return Eigen::VectorXd::Constant(1, grad_results_set.max_error);

  double total_weight{ 0 };
  for (const GradientResults& grad_result : grad_results_set.results)
  {
    if ((grad_result.cc_gradients[0].has_gradient &&
         (grad_result.cc_gradients[0].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time1)) ||
        (grad_result.cc_gradients[1].has_gradient &&
         (grad_result.cc_gradients[1].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time1)))
    {
      double d = std::max<double>(grad_result.error, 0.) * grad_result.data[2];
      double w = (std::max(grad_result.error_with_buffer * grad_result.data[2], 0.) /
                  grad_results_set.max_weighted_error_with_buffer);
      total_weight += w;
      err[0] += (w * d);
    }
  }

  assert(total_weight > 0);
  (total_weight > 0) ? err[0] = err[0] / total_weight : err[0] = 0;

  return err;
}

Eigen::VectorXd getWeightedAvgValuesT1(const GradientResultsSet& grad_results_set)
{
  assert(grad_results_set.dof > 0);
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  if (grad_results_set.results.empty())
    return Eigen::VectorXd::Constant(1, -grad_results_set.collision_margin_buffer);

  if (!(grad_results_set.max_error > 0))
    return Eigen::VectorXd::Constant(1, grad_results_set.max_error);

  double total_weight{ 0 };
  for (const GradientResults& grad_result : grad_results_set.results)
  {
    if ((grad_result.gradients[0].has_gradient &&
         (grad_result.gradients[0].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time0)) ||
        (grad_result.gradients[1].has_gradient &&
         (grad_result.gradients[1].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time0)))
    {
      double d = std::max<double>(grad_result.error, 0.) * grad_result.data[2];
      double w = (std::max(grad_result.error_with_buffer * grad_result.data[2], 0.) /
                  grad_results_set.max_weighted_error_with_buffer);
      total_weight += w;
      err[0] += (w * d);
    }
  }

  assert(total_weight > 0);
  (total_weight > 0) ? err[0] = err[0] / total_weight : err[0] = 0;

  return err;
}

Eigen::VectorXd getWeightedAvgValues(const GradientResultsSet& grad_results_set)
{
  assert(grad_results_set.dof > 0);
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  if (grad_results_set.results.empty())
    return Eigen::VectorXd::Constant(1, -grad_results_set.collision_margin_buffer);

  if (!(grad_results_set.max_error > 0))
    return Eigen::VectorXd::Constant(1, grad_results_set.max_error);

  double total_weight{ 0 };
  for (const GradientResults& grad_result : grad_results_set.results)
  {
    if ((grad_result.gradients[0].has_gradient &&
         (grad_result.gradients[0].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time1)) ||
        (grad_result.gradients[1].has_gradient &&
         (grad_result.gradients[1].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time1)))
    {
      double d = std::max<double>(grad_result.error, 0.) * grad_result.data[2];
      double w = (std::max(grad_result.error_with_buffer * grad_result.data[2], 0.) /
                  grad_results_set.max_weighted_error_with_buffer);
      total_weight += w;
      err[0] += (w * d);
    }
  }

  assert(total_weight > 0);
  (total_weight > 0) ? err[0] = err[0] / total_weight : err[0] = 0;

  return err;
}

Eigen::VectorXd getWeightedAvgGradientT0(const GradientResultsSet& grad_results_set)
{
  assert(grad_results_set.dof > 0);
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(grad_results_set.dof);
  if (grad_results_set.results.empty() || !(grad_results_set.max_error_with_buffer > 0))
    return grad_vec;

  double total_weight = 0;
  for (const auto& grad : grad_results_set.results)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (grad.cc_gradients[i].has_gradient &&
          (grad.cc_gradients[i].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time1))
      {
        double w =
            ((std::max(grad.error_with_buffer, 0.0) * grad.data[2]) / grad_results_set.max_weighted_error_with_buffer);
        total_weight += w;
        grad_vec += w * grad.data[2] * (grad.cc_gradients[i].scale * grad.gradients[i].gradient);
      }
    }
  }

  assert(total_weight > 0);
  return (1.0 / total_weight) * grad_vec;
}

Eigen::VectorXd getWeightedAvgGradientT1(const GradientResultsSet& grad_results_set)
{
  assert(grad_results_set.dof > 0);
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(grad_results_set.dof);
  if (grad_results_set.results.empty() || !(grad_results_set.max_error_with_buffer > 0))
    return grad_vec;

  double total_weight = 0;
  for (auto& grad : grad_results_set.results)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (grad.gradients[i].has_gradient &&
          (grad.gradients[i].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time0))
      {
        double w =
            ((std::max(grad.error_with_buffer, 0.0) * grad.data[2]) / grad_results_set.max_weighted_error_with_buffer);
        assert(!(w < 0));
        total_weight += w;
        grad_vec += w * grad.data[2] * (grad.gradients[i].scale * grad.gradients[i].gradient);
      }
    }
  }

  assert(total_weight > 0);
  return (1.0 / total_weight) * grad_vec;
}
}  // namespace trajopt_ifopt
