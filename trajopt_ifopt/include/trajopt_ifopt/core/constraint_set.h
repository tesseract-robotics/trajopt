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

#ifndef TRAJOPT_IFOPT_CORE_CONSTRAINT_SET_H
#define TRAJOPT_IFOPT_CORE_CONSTRAINT_SET_H

#include <trajopt_ifopt/core/component.h>
#include <trajopt_ifopt/core/composite.h>

namespace trajopt_ifopt
{
/**
 * @brief Controls how a component should represent "range" bounds (both lower and upper bounds).
 *
 * Some constraints naturally produce a single row with a bound interval:
 * @code
 *   lb <= g(x) <= ub
 * @endcode
 *
 * Downstream formulations (e.g., slack-variable handling or certain QP/SQP builders) may prefer
 * representing this as two separate inequality rows:
 * @code
 *   g(x) >= lb
 *   g(x) <= ub
 * @endcode
 *
 * This enum specifies whether to keep range bounds as a single row or to split them into two
 * inequality rows when building the constraint representation.
 *
 * @note This setting typically only affects rows whose bounds are true ranges (finite lower and
 *       finite upper with lb < ub). Equality bounds (lb == ub) and one-sided bounds are usually
 *       unchanged.
 */
enum class RangeBoundHandling : std::uint8_t
{
  /** @brief Do not split range bounds; keep them as a single row with [lb, ub]. */
  kKeepAsIs = 0,

  /**
   * @brief Split range bounds into two inequality rows.
   *
   * A single row with bounds [lb, ub] is represented as two rows:
   *  - Row A: g(x) >= lb  (lower-bound inequality)
   *  - Row B: g(x) <= ub  (upper-bound inequality)
   *
   * @note Enabling this increases the reported row count for each range-bounded row (typically by +1)
   *       and requires consistent handling in getValues(), getBounds(), and getJacobian() to reflect
   *       the expanded representation.
   */
  kSplitToTwoInequalities
};

/**
 * @brief A container holding a set of related constraints.
 *
 * This container holds constraints representing a single concept, e.g.
 * @c n constraints keeping a foot inside its range of motion. Each of the
 * @c n rows is given by:
 * lower_bound < g(x) < upper_bound
 *
 * These constraint sets are later then stitched together to form the overall
 * problem.
 *
 * @sa Component
 */
class ConstraintSet : public Differentiable
{
public:
  using Ptr = std::shared_ptr<ConstraintSet>;

  /**
   * @brief Creates constraints on the variables @c x.
   * @param name  What these constraints represent.
   * @param n_constraints  The number of constraints.
   */
  ConstraintSet(std::string name, bool dynamic);
  ConstraintSet(std::string name, int n_constraints);
  ~ConstraintSet() override = default;

  /**
   * @brief Connects the constraint with the optimization variables.
   * @param x  A pointer to the current values of the optimization variables.
   *
   * The optimization variable values are necessary for calculating constraint
   * violations and Jacobians.
   */
  void linkWithVariables(const Variables::Ptr& x);

protected:
  Variables::Ptr variables_;

  /**
   * @brief Read access to the value of the optimization variables.
   *
   * This must be used to formulate the constraint violation and Jacobian.
   */
  Variables::Ptr getVariables() const { return variables_; };

private:
  /**
   * @brief  Initialize quantities that depend on the optimization variables.
   * @param x  A pointer to the initial values of the optimization variables.
   *
   * Sometimes the number of constraints depends on the variable representation,
   * or shorthands to specific variable sets want to be saved for quicker
   * access later. This function can be overwritten for that.
   */
  virtual void initVariableDependedQuantities(const Variables::Ptr& x_init);
};
}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_CORE_CONSTRAINT_SET_H
