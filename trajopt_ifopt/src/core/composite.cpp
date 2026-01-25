/******************************************************************************
Copyright (c) 2017, Alexander W Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <trajopt_ifopt/core/composite.h>

#include <iostream>

namespace trajopt_ifopt
{
CompositeVariables::CompositeVariables(std::string name) : Variables(std::move(name), 0) {}

void CompositeVariables::markDirty()
{
  values_dirty_ = true;
  bounds_dirty_ = true;
}

void CompositeVariables::addComponent(Variables::Ptr c)
{
  // at this point the number of rows must be specified.
  assert(c->getRows() != -1);

  rows_ += c->getRows();
  components_.push_back(std::move(c));
  markDirty();
}

const CompositeVariables::ComponentVec& CompositeVariables::getComponents() const { return components_; }

void CompositeVariables::clearComponents()
{
  components_.clear();
  rows_ = 0;
  markDirty();
}

Variables::Ptr CompositeVariables::getComponent(const std::string& name) const
{
  for (const auto& c : components_)
    if (c->getName() == name)
      return c;

  assert(false);  // component with name doesn't exist, abort program
  return {};
}

bool CompositeVariables::empty() const { return components_.empty(); }

// ---- Variables / Component ----
const Eigen::VectorXd& CompositeVariables::getValues() const
{
  if (!values_dirty_ && values_cache_.size() == rows_)
    return values_cache_;

  values_cache_.resize(rows_);
  int row = 0;
  for (const auto& c : components_)
  {
    const int n_rows = c->getRows();
    values_cache_.segment(row, n_rows) = c->getValues();
    row += n_rows;
  }

  values_dirty_ = false;
  return values_cache_;
}

const std::vector<Bounds>& CompositeVariables::getBounds() const
{
  if (!bounds_dirty_ && static_cast<int>(bounds_cache_.size()) == rows_)
    return bounds_cache_;

  bounds_cache_.clear();
  bounds_cache_.reserve(static_cast<std::size_t>(rows_));

  for (const auto& c : components_)
  {
    const auto& b = c->getBounds();
    bounds_cache_.insert(bounds_cache_.end(), b.begin(), b.end());
  }

  bounds_dirty_ = false;
  return bounds_cache_;
}

void CompositeVariables::setVariables(const Eigen::VectorXd& x)
{
  int row = 0;
  for (auto& c : components_)
  {
    c->setVariables(x.middleRows(row, c->getRows()));
    row += c->getRows();
  }
  values_cache_ = x.head(row);
  values_dirty_ = false;
}

void CompositeVariables::print(int& index, double tolerance) const
{
  std::cout << name_ << ":\n";
  for (const auto& c : components_)
  {
    std::cout << "   ";  // indent components
    c->print(index, tolerance);
  }
  std::cout << "\n";
}

CompositeDifferentiable::CompositeDifferentiable(std::string name, Mode mode, bool dynamic)
  : Differentiable(std::move(name), 0, mode, dynamic)
{
}

void CompositeDifferentiable::markDirty()
{
  values_dirty_ = true;
  coeffs_dirty_ = true;
  bounds_dirty_ = true;
}

void CompositeDifferentiable::addComponent(Differentiable::Ptr c)
{
  // at this point the number of rows must be specified.
  assert(c->getRows() != -1);

  rows_ += c->getRows();
  components_.push_back(std::move(c));
  markDirty();
}

const CompositeDifferentiable::ComponentVec& CompositeDifferentiable::getComponents() const { return components_; }

void CompositeDifferentiable::clearComponents()
{
  components_.clear();
  rows_ = 0;
  n_vars_ = -1;
  markDirty();
}

Differentiable::Ptr CompositeDifferentiable::getComponent(const std::string& name) const
{
  for (const auto& c : components_)
    if (c->getName() == name)
      return c;

  assert(false);  // component with name doesn't exist, abort program
  return {};
}

bool CompositeDifferentiable::empty() const { return components_.empty(); }

int CompositeDifferentiable::update()
{
  int rows{ 0 };
  for (auto& c : components_)
    rows += c->update();

  if (dynamic_)
    rows_ = rows;

  assert(rows_ == rows);

  // dynamic sizing/state changed → invalidate caches
  markDirty();
  return rows_;
}

const Eigen::VectorXd& CompositeDifferentiable::getValues() const
{
  if (!values_dirty_ && values_cache_.size() == rows_)
    return values_cache_;

  values_cache_.setZero(rows_);

  int row = 0;
  for (const auto& c : components_)
  {
    const auto& v = c->getValues();
    values_cache_.segment(row, c->getRows()) += v;

    if (mode_ == Mode::kStackRows)
      row += c->getRows();
  }

  values_dirty_ = false;
  return values_cache_;
}

const Eigen::VectorXd& CompositeDifferentiable::getCoefficients() const
{
  if (!coeffs_dirty_ && coeffs_cache_.size() == rows_)
    return coeffs_cache_;

  coeffs_cache_.setZero(rows_);
  int row = 0;

  for (const auto& c : components_)
  {
    const auto& w = c->getCoefficients();
    coeffs_cache_.segment(row, c->getRows()) += w;

    if (mode_ == Mode::kStackRows)
      row += c->getRows();
  }

  coeffs_dirty_ = false;
  return coeffs_cache_;
}

Jacobian CompositeDifferentiable::getJacobian() const
{
  // set number of variables only the first time this function is called,
  // since number doesn't change during the optimization. Improves efficiency.
  if (n_vars_ == -1)
    n_vars_ = components_.empty() ? 0 : components_.front()->getJacobian().cols();

  Jacobian jacobian(rows_, n_vars_);

  if (n_vars_ == 0)
    return jacobian;

  Eigen::Index row = 0;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& c : components_)
  {
    const Jacobian jac = c->getJacobian();
    triplet_list.reserve(triplet_list.size() + static_cast<std::size_t>(jac.nonZeros()));

    for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.emplace_back(row + it.row(), it.col(), it.value());

    if (mode_ == Mode::kStackRows)
      row += c->getRows();
  }

  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

const std::vector<Bounds>& CompositeDifferentiable::getBounds() const
{
  if (!bounds_dirty_ && static_cast<int>(bounds_cache_.size()) == rows_)
    return bounds_cache_;

  bounds_cache_.clear();
  bounds_cache_.reserve(static_cast<std::size_t>(rows_));

  for (const auto& c : components_)
  {
    const auto& b = c->getBounds();
    bounds_cache_.insert(bounds_cache_.end(), b.begin(), b.end());
  }

  bounds_dirty_ = false;
  return bounds_cache_;
}

void CompositeDifferentiable::print(int& index, double tolerance) const
{
  std::cout << name_ << ":\n";
  for (const auto& c : components_)
  {
    std::cout << "   ";  // indent components
    c->print(index, tolerance);
  }
  std::cout << "\n";
}

}  // namespace trajopt_ifopt
