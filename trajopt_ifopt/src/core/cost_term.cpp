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

#include <trajopt_ifopt/core/cost_term.h>

#include <iomanip>
#include <iostream>

namespace trajopt_ifopt
{
CostTerm::CostTerm(std::string name) : ConstraintSet(std::move(name), 1) {}

Eigen::VectorXd CostTerm::getValues() const
{
  Eigen::VectorXd cost(1);
  cost(0) = getCost();
  return cost;
}

std::vector<Bounds> CostTerm::getBounds() const
{
  return std::vector<Bounds>(static_cast<std::size_t>(rows_), NoBound);  // NOLINT
}

void CostTerm::print(int& index, double /*tol*/) const
{
  // only one scalar cost value
  double cost = getValues()(0);

  std::cout.precision(2);
  std::cout << std::fixed << std::left << std::setw(30) << name_ << std::right << std::setw(4) << rows_ << std::setw(9)
            << index << std::setfill('.') << std::setw(7) << index + rows_ - 1 << std::setfill(' ') << std::setw(12)
            << cost << "\n";
}

}  // namespace trajopt_ifopt
