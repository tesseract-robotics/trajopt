/**
 * @author Levi Armstrong
 * @date Jan 7, 2025
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <tesseract/common/logging.h>

#include <cassert>

namespace trajopt_ifopt
{
Var::Var(Eigen::Index index, std::string name, double value, Bounds bounds, Node* parent)
  : index_(index)
  , identifier_(std::move(name))
  , names_({ identifier_ })
  , values_(Eigen::VectorXd::Constant(1, value))
  , bounds_({ bounds })
  , parent_(parent)
{
  clampValuesToBounds();
}

Var::Var(Eigen::Index index,
         std::string identifier,
         std::vector<std::string> names,
         const Eigen::VectorXd& values,
         std::vector<Bounds> bounds,
         Node* parent)
  : index_(index), identifier_(std::move(identifier)), names_(std::move(names)), values_(values), parent_(parent)
{
  if (static_cast<Eigen::Index>(names_.size()) != values_.size())
    throw std::runtime_error("Variable: '" + identifier_ + "' has mismatched sizes for names and values");

  setBounds(std::move(bounds));
}

const std::string& Var::getIdentifier() const { return identifier_; }
const Node* Var::getParent() const { return parent_; }
Eigen::Index Var::getIndex() const { return index_; }
Eigen::Index Var::size() const { return values_.rows(); }
const Eigen::VectorXd& Var::value() const { return values_; }
const std::vector<std::string>& Var::name() const { return names_; }
const std::vector<Bounds>& Var::bounds() const { return bounds_; }

void Var::setBounds(std::vector<Bounds> bounds)
{
  if (bounds.size() != names_.size())
    throw std::runtime_error("Variable: '" + identifier_ + "' has mismatched sizes for names and bounds");

  bounds_ = std::move(bounds);
  clampValuesToBounds();
}

void Var::clampValuesToBounds()
{
  Eigen::VectorXd clamped = trajopt_ifopt::getClosestValidPoint(values_, bounds_);

  // Clamping returns in-bound values unchanged, so compare exactly: a difference means a value was clamped or is NaN.
  // A tolerance would misreport infinite values (inf - inf is NaN) and hide small clamps.
  const bool changed = (clamped != values_);
  values_ = std::move(clamped);
  if (changed)
    TESSERACT_LOG_WARN("The values are not within the provided bounds. Adjusting to be within the bounds.");
}

void Var::incrementIndex(Eigen::Index value) { index_ += value; }
void Var::setVariables(const Eigen::Ref<const Eigen::VectorXd>& x)
{
  assert(index_ > -1 && index_ < x.size());
  assert(values_.rows() > 0 && (index_ + (values_.rows() - 1)) < x.size());
  values_ = x.segment(index_, values_.rows());
}

}  // namespace trajopt_ifopt
