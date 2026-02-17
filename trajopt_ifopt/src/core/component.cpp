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

#include <trajopt_ifopt/core/component.h>

#include <iomanip>
#include <iostream>

namespace trajopt_ifopt
{
// Component

Component::Component(std::string name) : Component(std::move(name), -1){};

Component::Component(std::string name, int num_rows) : name_(std::move(name)), rows_(num_rows) {}

const std::string& Component::getName() const { return name_; }

int Component::getRows() const
{
  assert(rows_ >= 0);
  return rows_;
}

void Component::print(int& index, double tolerance) const
{
  // calculate squared bound violation
  Eigen::VectorXd x = getValues();
  std::vector<Bounds> bounds = getBounds();

  std::vector<int> viol_idx;
  for (std::size_t i = 0; i < bounds.size(); ++i)
  {
    const auto& b = bounds.at(i);
    double val = x(static_cast<Eigen::Index>(i));
    if (val < b.getLower() - tolerance || b.getUpper() + tolerance < val)
      viol_idx.push_back(static_cast<int>(i));  // constraint out of bounds
  }

  std::string black = "\033[0m";
  std::string red = "\033[31m";
  std::string color = viol_idx.empty() ? black : red;

  std::cout.precision(2);
  std::cout << std::fixed << std::left << std::setw(30) << name_ << std::right << std::setw(4) << rows_ << std::setw(9)
            << index << std::setfill('.') << std::setw(7) << index + rows_ - 1 << std::setfill(' ') << color
            << std::setw(12) << viol_idx.size() << black << "\n";

  index += rows_;
}

// Differentiable

Differentiable::Differentiable(std::string name, Mode mode, bool dynamic)
  : Component(std::move(name), dynamic ? 0 : -1), mode_(mode), dynamic_(dynamic){};

Differentiable::Differentiable(std::string name, int num_rows, Mode mode, bool dynamic)
  : Component(std::move(name), num_rows), mode_(mode), dynamic_(dynamic){};

bool Differentiable::isDynamic() const { return dynamic_; }
bool Differentiable::isScalar() const { return mode_ == Mode::kSumToScalar; }
Eigen::Index Differentiable::getNonZeros() const
{
  assert(non_zeros_ >= 0);
  return non_zeros_;
}

}  // namespace trajopt_ifopt
