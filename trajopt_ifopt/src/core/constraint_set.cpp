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
ConstraintSet::ConstraintSet(std::string name, bool dynamic)
  : Differentiable(std::move(name), Mode::kStackRows, dynamic)
{
}

ConstraintSet::ConstraintSet(std::string name, int n_constraints)
  : Differentiable(std::move(name), n_constraints, Mode::kStackRows, false)
{
}

Jacobian ConstraintSet::getJacobian() const
{
  Jacobian jacobian(rows_, variables_->getRows());
  jacobian.reserve(non_zeros_.load());

  // This is an optimization for when you only have one variable set
  if (variables_->getComponents().size() == 1)
  {
    fillJacobianBlock(jacobian, variables_->getComponents().front()->getName());
    return jacobian;
  }

  int col = 0;
  Jacobian jac;
  jac.reserve(non_zeros_.load());
  std::vector<Eigen::Triplet<double>> triplet_list;
  triplet_list.reserve(static_cast<std::size_t>(non_zeros_.load()));

  for (const auto& vars : variables_->getComponents())
  {
    int n = vars->getRows();
    jac.resize(rows_, n);

    fillJacobianBlock(jac, vars->getName());

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

void ConstraintSet::linkWithVariables(const CompositeVariables::Ptr& x)
{
  variables_ = x;
  initVariableDependedQuantities(x);
}

void ConstraintSet::initVariableDependedQuantities(const Variables::Ptr& /*x_init*/) {}

}  // namespace trajopt_ifopt
