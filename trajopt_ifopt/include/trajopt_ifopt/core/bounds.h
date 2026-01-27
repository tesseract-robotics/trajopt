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

#ifndef TRAJOPT_IFOPT_CORE_BOUNDS_H
#define TRAJOPT_IFOPT_CORE_BOUNDS_H

#include <cmath>
#include <cstdint>

namespace trajopt_ifopt
{
/**
 * @brief Classification of scalar bounds for an expression g(x).
 *
 * @details
 * This enum describes the shape of a bound interval [lower, upper] associated with a scalar expression g(x).
 * It is commonly used to determine how a constraint row should be modeled in a QP/NLP:
 *
 *   - UNBOUNDED:   (-inf, +inf)            => no restriction on g(x)
 *   - EQUALITY:    (v, v)                  => g(x) == v
 *   - LOWER_BOUND: (l, +inf)               => l <= g(x)
 *   - UPPER_BOUND: (-inf, u)               => g(x) <= u
 *   - RANGE_BOUND: (l, u) with l < u       => l <= g(x) <= u
 *
 * Notes:
 *   - "Finite" means the value is not +/-infinity.
 *   - EQUALITY is typically detected when |upper - lower| <= eps.
 *   - RANGE_BOUND is the two-sided finite bound case (sometimes called a "boxed bound" or "interval bound").
 */
enum class BoundsType : std::uint8_t
{
  /** @brief No finite lower or upper bound: (-inf, +inf). */
  UNBOUNDED = 0,

  /** @brief Equality bound: (v, v) meaning g(x) == v (within a tolerance). */
  EQUALITY,

  /** @brief Lower-bounded only: (l, +inf) meaning l <= g(x). */
  LOWER_BOUND,

  /** @brief Upper-bounded only: (-inf, u) meaning g(x) <= u. */
  UPPER_BOUND,

  /** @brief Two-sided bound: (l, u) with l < u meaning l <= g(x) <= u. */
  RANGE_BOUND,
};

/**
 * @brief Lower/upper bounds for scalar optimization variables and scalar constraint rows.
 *
 * @details
 * A Bounds instance represents an interval [lower, upper] applied to either:
 *   - a single decision variable x_i, or
 *   - a single scalar constraint expression g_i(x).
 *
 * Bounds may be:
 *   - equality:      lower == upper          (g(x) == value)
 *   - one-sided:     [lower, +inf) or (-inf, upper]
 *   - two-sided:     [lower, upper] with lower < upper
 *   - unbounded:     (-inf, +inf)
 *
 * Infinity is represented using +/-INFINITY. The bound "type" is cached (BoundsType) and
 * automatically updated whenever the limits change.
 *
 * @note
 * Equality detection may be tolerance-based inside updateType() (e.g., |upper-lower| <= eps).
 */
class Bounds
{
public:
  /**
   * @brief Construct bounds with the given lower and upper limits.
   *
   * @param lower Lower bound value (may be -INFINITY for no lower bound).
   * @param upper Upper bound value (may be +INFINITY for no upper bound).
   *
   * @post getLower() == lower and getUpper() == upper
   * @post getType() reflects the interval shape after construction
   */
  Bounds(double lower = 0.0, double upper = 0.0);

  /**
   * @brief Set both the lower and upper bounds.
   *
   * @param lower New lower bound value (may be -INFINITY).
   * @param upper New upper bound value (may be +INFINITY).
   *
   * @post getType() is updated to match the new interval.
   */
  void set(double lower, double upper);

  /**
   * @brief Set the lower bound.
   * @param lower New lower bound value (may be -INFINITY).
   * @post getType() is updated to match the new interval.
   */
  void setLower(double lower);

  /**
   * @brief Get the lower bound value.
   * @return The lower bound (may be -INFINITY).
   */
  double getLower() const;

  /**
   * @brief Set the upper bound.
   * @param upper New upper bound value (may be +INFINITY).
   * @post getType() is updated to match the new interval.
   */
  void setUpper(double upper);

  /**
   * @brief Get the upper bound value.
   * @return The upper bound (may be +INFINITY).
   */
  double getUpper() const;

  /**
   * @brief Get the cached bound classification.
   *
   * @return A BoundsType describing whether this interval is unbounded, equality, one-sided,
   *         or two-sided (range/boxed).
   */
  BoundsType getType() const;

  /**
   * @brief Shift the interval by adding a scalar to both bounds.
   *
   * @details After this operation:
   *   lower := lower + scalar
   *   upper := upper + scalar
   *
   * Useful when converting between centered and absolute bounds.
   *
   * @param scalar Value to add to both lower and upper.
   * @post getType() is updated to match the shifted interval.
   */
  void operator+=(double scalar);

  /**
   * @brief Shift the interval by subtracting a scalar from both bounds.
   *
   * @details After this operation:
   *   lower := lower - scalar
   *   upper := upper - scalar
   *
   * @param scalar Value to subtract from both lower and upper.
   * @post getType() is updated to match the shifted interval.
   */
  void operator-=(double scalar);

private:
  /** @brief Lower bound value (may be -INFINITY). */
  double lower_{ 0 };

  /** @brief Upper bound value (may be +INFINITY). */
  double upper_{ 0 };

  /** @brief Cached classification of the current [lower, upper] interval. */
  BoundsType type_{ BoundsType::EQUALITY };

  /**
   * @brief Recompute @ref type_ from the current @ref lower_ and @ref upper_.
   *
   * @details This should be called whenever lower_ or upper_ changes.
   */
  void updateType();
};

/** @brief Convenience constant representing (-inf, +inf). */
static const Bounds NoBound = Bounds(-double(INFINITY), +double(INFINITY));

/** @brief Convenience constant representing an equality bound at zero: (0, 0). */
static const Bounds BoundZero = Bounds(0.0, 0.0);

/** @brief Convenience constant representing a nonnegative bound: [0, +inf). */
static const Bounds BoundGreaterZero = Bounds(0.0, +double(INFINITY));

/** @brief Convenience constant representing a nonpositive bound: (-inf, 0]. */
static const Bounds BoundSmallerZero = Bounds(-double(INFINITY), 0.0);

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_CORE_BOUNDS_H
