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

#include <iomanip>
#include <iostream>

namespace trajopt_ifopt
{
Component::Component(std::string name, int num_rows, bool is_dynamic)
  : name_(std::move(name)), num_rows_(num_rows), is_dynamic_(is_dynamic)
{
}

int Component::GetRows() const { return num_rows_; }

void Component::SetRows(int num_rows) { num_rows_ = num_rows; }

const std::string& Component::GetName() const { return name_; }

bool Component::IsDynamic() const { return is_dynamic_; }

void Component::Print(double tol, int& index) const
{
  // calculate squared bound violation
  Eigen::VectorXd x = GetValues();
  std::vector<Bounds> bounds = GetBounds();

  std::vector<int> viol_idx;
  for (std::size_t i = 0; i < bounds.size(); ++i)
  {
    const auto& b = bounds.at(i);
    double val = x(static_cast<Eigen::Index>(i));
    if (val < b.lower - tol || b.upper + tol < val)
      viol_idx.push_back(static_cast<int>(i));  // constraint out of bounds
  }

  std::string black = "\033[0m";
  std::string red = "\033[31m";
  std::string color = viol_idx.empty() ? black : red;

  std::cout.precision(2);
  std::cout << std::fixed << std::left << std::setw(30) << name_ << std::right << std::setw(4) << num_rows_
            << std::setw(9) << index << std::setfill('.') << std::setw(7) << index + num_rows_ - 1 << std::setfill(' ')
            << color << std::setw(12) << viol_idx.size() << black << "\n";

  index += num_rows_;
}

Composite::Composite(std::string name, bool is_cost, bool is_dynamic)
  : Component(std::move(name), 0, is_dynamic), is_cost_(is_cost)
{
}

void Composite::AddComponent(Component::Ptr c)
{
  // at this point the number of rows must be specified.
  assert(c->GetRows() != kSpecifyLater);
  assert(c->IsDynamic() == IsDynamic());

  if (is_cost_)
    SetRows(1);
  else if (!IsDynamic())
    SetRows(GetRows() + c->GetRows());

  components_.push_back(std::move(c));
}

void Composite::ClearComponents()
{
  components_.clear();
  SetRows(0);
}

Component::Ptr Composite::GetComponent(const std::string& name) const
{
  for (const auto& c : components_)
    if (c->GetName() == name)
      return c;

  assert(false);  // component with name doesn't exist, abort program
  return {};
}

int Composite::Update()
{
  if (!IsDynamic())
    return GetRows();

  int rows{ 0 };
  for (auto& c : components_)
    rows += c->Update();

  SetRows(rows);
  return rows;
}

Eigen::VectorXd Composite::GetValues() const
{
  Eigen::VectorXd g_all = Eigen::VectorXd::Zero(GetRows());

  int row = 0;
  for (const auto& c : components_)
  {
    int n_rows = c->GetRows();
    Eigen::VectorXd g = c->GetValues();
    g_all.middleRows(row, n_rows) += g;

    if (!is_cost_)
      row += n_rows;
  }
  return g_all;
}

void Composite::SetVariables(const Eigen::VectorXd& x)
{
  int row = 0;
  for (auto& c : components_)
  {
    int n_rows = c->GetRows();
    c->SetVariables(x.middleRows(row, n_rows));
    row += n_rows;
  }
}

Jacobian Composite::GetJacobian() const
{
  // set number of variables only the first time this function is called,
  // since number doesn't change during the optimization. Improves efficiency.
  if (n_var == -1)
    n_var = components_.empty() ? 0 : components_.front()->GetJacobian().cols();

  Jacobian jacobian(GetRows(), n_var);

  if (n_var == 0)
    return jacobian;

  Eigen::Index row = 0;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& c : components_)
  {
    const Jacobian& jac = c->GetJacobian();
    triplet_list.reserve(triplet_list.size() + static_cast<std::size_t>(jac.nonZeros()));

    for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.emplace_back(row + it.row(), it.col(), it.value());

    if (!is_cost_)
      row += c->GetRows();
  }

  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

std::vector<Bounds> Composite::GetBounds() const
{
  std::vector<Bounds> bounds;
  bounds.reserve(static_cast<std::size_t>(GetRows()));
  for (const auto& c : components_)
  {
    std::vector<Bounds> b = c->GetBounds();
    bounds.insert(bounds.end(), b.begin(), b.end());
  }

  return bounds;
}

const Composite::ComponentVec& Composite::GetComponents() const { return components_; }

void Composite::PrintAll() const
{
  int index = 0;
  // tolerance when printing out constraint/bound violation.
  constexpr double tol = 0.001;

  std::cout << GetName() << ":\n";
  for (const auto& c : components_)
  {
    std::cout << "   ";  // indent components
    c->Print(tol, index);
  }
  std::cout << "\n";
}

}  // namespace trajopt_ifopt
