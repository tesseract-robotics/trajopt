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
#include <trajopt_common/collision_utils.h>

#include <iostream>

namespace trajopt_ifopt
{
CompositeDifferentiable::CompositeDifferentiable(std::string name, Eigen::Index num_vars, Mode mode, bool dynamic)
  : Differentiable(std::move(name), 0, mode, dynamic), num_vars_(num_vars)
{
}

void CompositeDifferentiable::addComponent(Differentiable::Ptr c)
{
  // at this point the number of rows must be specified.
  assert(c->getRows() != -1);

  rows_ += c->getRows();
  components_.push_back(std::move(c));
}

const CompositeDifferentiable::ComponentVec& CompositeDifferentiable::getComponents() const { return components_; }

void CompositeDifferentiable::clearComponents()
{
  components_.clear();
  rows_ = 0;
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
  Eigen::Index non_zeros{ 0 };
  for (auto& c : components_)
  {
    rows += c->update();
    non_zeros += c->getNonZeros();
    assert(c->getRows() >= 0);
    assert(c->getNonZeros() >= 0);
  }

  if (dynamic_)
  {
    rows_ = rows;
    non_zeros_ = non_zeros;
  }

  assert(rows_ == rows);
  return rows_;
}

Eigen::VectorXd CompositeDifferentiable::getValues() const
{
  Eigen::VectorXd g_all = Eigen::VectorXd::Zero(rows_);

  int row = 0;
  for (const auto& c : components_)
  {
    g_all.middleRows(row, c->getRows()) += c->getValues();
    if (mode_ == Mode::kStackRows)
      row += c->getRows();
  }
  return g_all;
}

Eigen::VectorXd CompositeDifferentiable::getCoefficients() const
{
  Eigen::VectorXd g_all = Eigen::VectorXd::Zero(rows_);

  int row = 0;
  for (const auto& c : components_)
  {
    g_all.middleRows(row, c->getRows()) += c->getCoefficients();
    if (mode_ == Mode::kStackRows)
      row += c->getRows();
  }
  return g_all;
}

Jacobian CompositeDifferentiable::getJacobian() const
{
  Jacobian jacobian(rows_, num_vars_);

  if (num_vars_ == 0)
    return jacobian;

  Eigen::Index row = 0;
  std::vector<Eigen::Triplet<double>> triplet_list;
  triplet_list.reserve(static_cast<std::size_t>(non_zeros_ >= 0 ? non_zeros_ : 0));

  for (const auto& c : components_)
  {
    const Jacobian jac = c->getJacobian();

    for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.emplace_back(row + it.row(), it.col(), it.value());

    if (mode_ == Mode::kStackRows)
      row += c->getRows();
  }

  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

std::vector<Bounds> CompositeDifferentiable::getBounds() const
{
  std::vector<Bounds> bounds;
  bounds.reserve(static_cast<std::size_t>(rows_));
  for (const auto& c : components_)
  {
    std::vector<Bounds> b = c->getBounds();
    bounds.insert(bounds.end(), b.begin(), b.end());
  }

  return bounds;
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
