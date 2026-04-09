/**
 * @file collision_types.h
 * @brief Contains data structures used by collision constraints
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
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

#include <trajopt_common/collision_types.h>
#include <tesseract/common/utils.h>
#include <cassert>

namespace trajopt_common
{
bool CoeffEntry::operator==(const CoeffEntry& other) const
{
  return name1 == other.name1 && name2 == other.name2 &&
         tesseract::common::almostEqualRelativeAndAbs(
             coeff, other.coeff, static_cast<double>(std::numeric_limits<float>::epsilon()));
}

bool CoeffEntry::operator!=(const CoeffEntry& other) const { return !operator==(other); }

CollisionCoeffData::CollisionCoeffData(double default_collision_coeff)
  : default_collision_coeff_(default_collision_coeff)
{
}

void CollisionCoeffData::setDefaultCollisionCoeff(double default_collision_coeff)
{
  default_collision_coeff_ = default_collision_coeff;
}

double CollisionCoeffData::getDefaultCollisionCoeff() const { return default_collision_coeff_; }

void CollisionCoeffData::setCollisionCoeff(const std::string& obj1, const std::string& obj2, double collision_coeff)
{
  using tesseract::common::LinkId;
  using tesseract::common::LinkIdPair;

  const LinkIdPair key = LinkIdPair::make(LinkId::fromName(obj1), LinkId::fromName(obj2));

  // Canonical name ordering: match the id ordering
  const bool swap = (LinkId::fromName(obj1).value > LinkId::fromName(obj2).value);
  const std::string& canonical_first = swap ? obj2 : obj1;
  const std::string& canonical_second = swap ? obj1 : obj2;

  // Hash collision check
  auto it = lookup_table_.find(key);
  if (it != lookup_table_.end())
  {
    if (it->second.name1 != canonical_first || it->second.name2 != canonical_second)
      throw std::runtime_error("CollisionCoeffData: hash collision detected between pair (" + it->second.name1 + ", " +
                               it->second.name2 + ") and (" + canonical_first + ", " + canonical_second + ")");
  }

  lookup_table_.insert_or_assign(key, CoeffEntry{ canonical_first, canonical_second, collision_coeff });

  if (tesseract::common::almostEqualRelativeAndAbs(collision_coeff, 0.0))
    zero_coeff_.insert(key);
  else
    zero_coeff_.erase(key);
}

double CollisionCoeffData::getCollisionCoeff(tesseract::common::LinkId id1, tesseract::common::LinkId id2) const
{
  const auto it = lookup_table_.find(tesseract::common::LinkIdPair::make(id1, id2));
  if (it != lookup_table_.end())
    return it->second.coeff;
  return default_collision_coeff_;
}

double CollisionCoeffData::getCollisionCoeff(const std::string& obj1, const std::string& obj2) const
{
  return getCollisionCoeff(tesseract::common::LinkId::fromName(obj1), tesseract::common::LinkId::fromName(obj2));
}

bool CollisionCoeffData::hasZeroCoeff(tesseract::common::LinkId id1, tesseract::common::LinkId id2) const
{
  return zero_coeff_.count(tesseract::common::LinkIdPair::make(id1, id2)) != 0;
}

bool CollisionCoeffData::hasZeroCoeff(const std::string& obj1, const std::string& obj2) const
{
  return hasZeroCoeff(tesseract::common::LinkId::fromName(obj1), tesseract::common::LinkId::fromName(obj2));
}

const PairsCollisionCoeffData& CollisionCoeffData::getCollisionCoeffPairData() const { return lookup_table_; }

const std::unordered_set<tesseract::common::LinkIdPair, tesseract::common::LinkIdPair::Hash>&
CollisionCoeffData::getPairsWithZeroCoeff() const
{
  return zero_coeff_;
}

bool CollisionCoeffData::operator==(const CollisionCoeffData& rhs) const
{
  static constexpr auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &=
      tesseract::common::almostEqualRelativeAndAbs(default_collision_coeff_, rhs.default_collision_coeff_, max_diff);

  // Compare lookup tables by size and per-entry coefficient
  equal &= (lookup_table_.size() == rhs.lookup_table_.size());
  if (equal)
  {
    for (const auto& [key, entry] : lookup_table_)
    {
      auto it = rhs.lookup_table_.find(key);
      if (it == rhs.lookup_table_.end() ||
          !tesseract::common::almostEqualRelativeAndAbs(entry.coeff, it->second.coeff, max_diff))
      {
        equal = false;
        break;
      }
    }
  }

  equal &= (zero_coeff_ == rhs.zero_coeff_);
  return equal;
}

bool CollisionCoeffData::operator!=(const CollisionCoeffData& rhs) const { return !operator==(rhs); }

TrajOptCollisionConfig::TrajOptCollisionConfig(double margin,
                                               double coeff,
                                               tesseract::collision::ContactRequest request,
                                               tesseract::collision::CollisionEvaluatorType type,
                                               double longest_valid_segment_length,
                                               tesseract::collision::CollisionCheckProgramType check_program_mode)
  : contact_manager_config(margin)
  , collision_check_config(std::move(request), type, longest_valid_segment_length, check_program_mode)
  , collision_coeff_data(coeff)
{
}

bool TrajOptCollisionConfig::operator==(const TrajOptCollisionConfig& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (enabled == rhs.enabled);
  equal &= (contact_manager_config == rhs.contact_manager_config);
  equal &= (collision_check_config == rhs.collision_check_config);
  equal &= (collision_coeff_data == rhs.collision_coeff_data);
  equal &= tesseract::common::almostEqualRelativeAndAbs(collision_margin_buffer, rhs.collision_margin_buffer, max_diff);
  equal &= (max_num_cnt == rhs.max_num_cnt);
  return equal;
}

bool TrajOptCollisionConfig::operator!=(const TrajOptCollisionConfig& rhs) const { return !operator==(rhs); }

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

void LinkGradientResults::clear()
{
  has_gradient = false;
  scale = 1.0;
  cc_type = tesseract::collision::ContinuousCollisionType::CCType_None;
}

void GradientResults::clear()
{
  error = 0;
  error_with_buffer = 0;
  gradients[0].clear();
  gradients[1].clear();
  cc_gradients[0].clear();
  cc_gradients[1].clear();
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

void GradientResultsSet::add(GradientResults gradient_result)
{
  for (std::size_t i = 0; i < 2; ++i)
  {
    const auto& g = gradient_result.gradients[i];
    if (!g.has_gradient)
      continue;

    // Update max error for LinkA and LinkB excluding values at T1
    if (g.cc_type != tesseract::collision::ContinuousCollisionType::CCType_Time1)
    {
      max_error[i].has_error[0] = true;
      max_error[i].error[0] = std::max(max_error[i].error[0], gradient_result.error);
      max_error[i].error_with_buffer[0] =
          std::max(max_error[i].error_with_buffer[0], gradient_result.error_with_buffer);
    }

    // Update max error for LinkA and LinkB excluding values at T0
    if (g.cc_type != tesseract::collision::ContinuousCollisionType::CCType_Time0)
    {
      max_error[i].has_error[1] = true;
      max_error[i].error[1] = std::max(max_error[i].error[1], gradient_result.error);
      max_error[i].error_with_buffer[1] =
          std::max(max_error[i].error_with_buffer[1], gradient_result.error_with_buffer);
    }
  }

  results.emplace_back(std::move(gradient_result));
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

}  // namespace trajopt_common
