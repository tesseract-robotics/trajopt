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

#include <trajopt_ifopt/core/constraint_set.h>

namespace trajopt_ifopt
{
ConstraintSet::ConstraintSet(std::string name, int n_constraints) : Component(std::move(name), n_constraints, false) {}

Jacobian ConstraintSet::GetJacobian() const
{
  Jacobian jacobian(GetRows(), variables_->GetRows());

  int col = 0;
  Jacobian jac;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& vars : variables_->GetComponents())
  {
    int n = vars->GetRows();
    jac.resize(GetRows(), n);

    FillJacobianBlock(vars->GetName(), jac);
    // reserve space for the new elements to reduce memory allocation
    triplet_list.reserve(triplet_list.size() + static_cast<std::size_t>(jac.nonZeros()));

    // create triplets for the derivative at the correct position in the overall Jacobian
    for (Eigen::Index k = 0; k < jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.emplace_back(it.row(), col + it.col(), it.value());
    col += n;
  }

  // transform triplet vector into sparse matrix
  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

void ConstraintSet::LinkWithVariables(const VariablesPtr& x)
{
  variables_ = x;
  InitVariableDependedQuantities(x);
}

void ConstraintSet::InitVariableDependedQuantities(const VariablesPtr& /*x_init*/) {}

void ConstraintSet::SetVariables(const Eigen::VectorXd& /*x*/)
{
  throw std::runtime_error("not implemented for fixed size constraint sets");
};

int ConstraintSet::Update() { throw std::runtime_error("not implemented for fixed size constraint sets"); }

}  // namespace trajopt_ifopt
