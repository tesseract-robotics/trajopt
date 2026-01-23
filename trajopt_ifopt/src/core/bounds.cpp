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

#include <trajopt_ifopt/core/bounds.h>

namespace trajopt_ifopt
{
bool isFinite(double value) { return std::isfinite(value) && (value < 1e20) && (value > -1e20); }

Bounds::Bounds(double lower, double upper) : lower_(lower), upper_(upper) { updateType(); }

void Bounds::set(double lower, double upper)
{
  lower_ = lower;
  upper_ = upper;
  updateType();
}

void Bounds::setLower(double lower)
{
  lower_ = lower;
  updateType();
}

double Bounds::getLower() const { return lower_; }

void Bounds::setUpper(double upper)
{
  upper_ = upper;
  updateType();
}

double Bounds::getUpper() const { return upper_; }

BoundsType Bounds::getType() const { return type_; }

void Bounds::operator+=(double scalar)
{
  lower_ += scalar;
  upper_ += scalar;
  updateType();
}

void Bounds::operator-=(double scalar)
{
  lower_ -= scalar;
  upper_ -= scalar;
  updateType();
}

void Bounds::updateType()
{
  if (!isFinite(lower_) && !isFinite(upper_))
    type_ = BoundsType::UNBOUNDED;
  else if (isFinite(lower_) && isFinite(upper_))
    type_ = (std::abs(upper_ - lower_) < 1e-8) ? BoundsType::EQUALITY : BoundsType::RANGE_BOUND;
  else
    type_ = (isFinite(lower_)) ? BoundsType::LOWER_BOUND : BoundsType::UPPER_BOUND;
}
}  // namespace trajopt_ifopt
