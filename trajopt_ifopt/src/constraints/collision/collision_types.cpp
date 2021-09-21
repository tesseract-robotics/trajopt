/**
 * @file collision_types.h
 * @brief Contains data structures used by collision constraints
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
 * @bug Only Discrete Evaluator is implemented
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

#include <trajopt_ifopt/constraints/collision/collision_types.h>

namespace trajopt_ifopt
{
CollisionCoeffData::CollisionCoeffData(const double& default_collision_coeff)
  : default_collision_coeff_(default_collision_coeff)
{
}

void CollisionCoeffData::setPairCollisionMarginData(const std::string& obj1,
                                                    const std::string& obj2,
                                                    const double& collision_coeff)
{
  auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
  lookup_table_[key] = collision_coeff;
}

const double& CollisionCoeffData::getPairCollisionCoeff(const std::string& obj1, const std::string& obj2) const
{
  auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
  const auto it = lookup_table_.find(key);

  if (it != lookup_table_.end())
  {
    return it->second;
  }
  return default_collision_coeff_;
}

TrajOptCollisionConfig::TrajOptCollisionConfig(double margin, double coeff)
  : CollisionCheckConfig(margin), collision_coeff_data(coeff)
{
}

double LinkMaxError::getMaxError() const
{
  if (has_error[0] && has_error[1])
    return std::max(error[0], error[1]);

  if (has_error[0])
    return error[0];

  if (has_error[1])
    return error[1];

  throw std::runtime_error("Invalid LinkMaxError");
}

double LinkMaxError::getMaxErrorWithBuffer() const
{
  if (has_error[0] && has_error[1])
    return std::max(error_with_buffer[0], error_with_buffer[1]);

  if (has_error[0])
    return error_with_buffer[0];

  if (has_error[1])
    return error_with_buffer[1];

  throw std::runtime_error("Invalid LinkMaxError");
}

void GradientResultsSet::add(const GradientResults& gradient_result)
{
  // Update max error for LinkA and LinkB excluding values at T1
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (gradient_result.gradients[i].has_gradient &&
        gradient_result.gradients[i].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time1)
    {
      max_error[i].has_error[0] = true;

      if (gradient_result.error > max_error[i].error[0])
        max_error[i].error[0] = gradient_result.error;

      if (gradient_result.error_with_buffer > max_error[i].error_with_buffer[0])
        max_error[i].error_with_buffer[0] = gradient_result.error_with_buffer;
    }
  }

  // Update max error for LinkA and LinkB excluding values at T0
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (gradient_result.gradients[i].has_gradient &&
        gradient_result.gradients[i].cc_type != tesseract_collision::ContinuousCollisionType::CCType_Time0)
    {
      max_error[i].has_error[1] = true;

      if (gradient_result.error > max_error[i].error[1])
        max_error[i].error[1] = gradient_result.error;

      if (gradient_result.error_with_buffer > max_error[i].error_with_buffer[1])
        max_error[i].error_with_buffer[1] = gradient_result.error_with_buffer;
    }
  }

  results.push_back(gradient_result);
}

double GradientResultsSet::getMaxError() const
{
  double e{ std::numeric_limits<double>::lowest() };
  bool found{ false };
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (max_error[i].has_error[0] && max_error[i].error[0] > e)
    {
      e = max_error[i].error[0];
      found = true;
    }

    if (max_error[i].has_error[1] && max_error[i].error[1] > e)
    {
      e = max_error[i].error[1];
      found = true;
    }
  }

  if (!found)
    throw std::runtime_error("Max error does not exist.");

  assert(e > (std::numeric_limits<double>::lowest() / 2.0));
  return e;
}

double GradientResultsSet::getMaxErrorT0() const
{
  double e{ std::numeric_limits<double>::lowest() };
  bool found{ false };
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (max_error[i].has_error[0] && max_error[i].error[0] > e)
    {
      e = max_error[i].error[0];
      found = true;
    }
  }

  if (!found)
    throw std::runtime_error("Max error at T0 does not exist.");

  assert(e > (std::numeric_limits<double>::lowest() / 2.0));
  return e;
}

double GradientResultsSet::getMaxErrorT1() const
{
  double e{ std::numeric_limits<double>::lowest() };
  bool found{ false };
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (max_error[i].has_error[1] && max_error[i].error[1] > e)
    {
      e = max_error[i].error[1];
      found = true;
    }
  }

  if (!found)
    throw std::runtime_error("Max error at T1 does not exist.");

  assert(e > (std::numeric_limits<double>::lowest() / 2.0));
  return e;
}

double GradientResultsSet::getMaxErrorWithBuffer() const
{
  double e{ std::numeric_limits<double>::lowest() };
  bool found{ false };
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (max_error[i].has_error[0] && max_error[i].error_with_buffer[0] > e)
    {
      e = max_error[i].error_with_buffer[0];
      found = true;
    }

    if (max_error[i].has_error[1] && max_error[i].error_with_buffer[1] > e)
    {
      e = max_error[i].error_with_buffer[1];
      found = true;
    }
  }

  if (!found)
    throw std::runtime_error("Max error with buffer does not exist.");

  assert(e > (std::numeric_limits<double>::lowest() / 2.0));
  return e;
}

double GradientResultsSet::getMaxErrorWithBufferT0() const
{
  double e{ std::numeric_limits<double>::lowest() };
  bool found{ false };
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (max_error[i].has_error[0] && max_error[i].error_with_buffer[0] > e)
    {
      e = max_error[i].error_with_buffer[0];
      found = true;
    }
  }

  if (!found)
    throw std::runtime_error("Max error with buffer at T0 does not exist.");

  assert(e > (std::numeric_limits<double>::lowest() / 2.0));
  return e;
}

double GradientResultsSet::getMaxErrorWithBufferT1() const
{
  double e{ std::numeric_limits<double>::lowest() };
  bool found{ false };
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (max_error[i].has_error[1] && max_error[i].error_with_buffer[1] > e)
    {
      e = max_error[i].error_with_buffer[1];
      found = true;
    }
  }

  if (!found)
    throw std::runtime_error("Max error with buffer at T1 does not exist.");

  assert(e > (std::numeric_limits<double>::lowest() / 2.0));
  return e;
}

}  // namespace trajopt_ifopt
