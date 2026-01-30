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

#ifndef TRAJOPT_IFOPT_CORE_DYNAMIC_COMPOSITE_H
#define TRAJOPT_IFOPT_CORE_DYNAMIC_COMPOSITE_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <trajopt_ifopt/core/component.h>

namespace trajopt_ifopt
{
/**
 * @brief A collection of variable components which is treated as another variables component.
 *
 * This composite does not perform any evaluation itself; it stitches together:
 * @li values (concatenation)
 * @li bounds (concatenation)
 * and partitions a solver-provided decision vector into per-variable-set segments.
 *
 * @note This composite assumes variables are fixed-sized. Its row count is the sum
 *       of its child variable set sizes.
 *
 * @ingroup ProblemFormulation
 */
class CompositeVariables : public Variables
{
public:
  using Ptr = std::shared_ptr<CompositeVariables>;
  using ComponentVec = std::vector<Variables::Ptr>;

  explicit CompositeVariables(std::string name);
  ~CompositeVariables() override = default;

  /**
   * @brief Adds a variables component to this composite.
   *
   * @attention The number of rows must be specified by the component and must be non-negative.
   */
  void addComponent(Variables::Ptr component);

  /** @brief Removes all components and resets row count to zero. */
  void clearComponents();

  /** @brief Returns read access to the components. */
  const ComponentVec& getComponents() const;

  /** @brief True if no components have been added. */
  bool empty() const;

  /**
   * @brief Access component with the specified name.
   * @param name The component name.
   * @return The variables component pointer.
   *
   * @warning This is a linear search. Prefer storing handles if performance matters.
   */
  Variables::Ptr getComponent(const std::string& name) const;

  // ---- Variables / Component ----
  Eigen::VectorXd getValues() const override;
  std::vector<Bounds> getBounds() const override;
  void setVariables(const Eigen::VectorXd& x) override;
  std::size_t getHash() const override;
  void print(int& index, double tolerance = 0.001) const override;

private:
  ComponentVec components_;
  std::size_t hash_{ 0 };
};

/**
 * @brief A collection of differentiable components which is treated as another differentiable component.
 *
 * This composite stitches together:
 * @li values (stacked for constraints, summed for costs)
 * @li bounds (concatenated)
 * @li Jacobians (stacked for constraints, summed for costs)
 *
 * It also supports dynamic-sized components through @ref update().
 *
 * @ingroup ProblemFormulation
 */
class CompositeDifferentiable : public Differentiable
{
public:
  using Ptr = std::shared_ptr<CompositeDifferentiable>;
  using ComponentVec = std::vector<Differentiable::Ptr>;

  CompositeDifferentiable(std::string name, Mode mode, bool dynamic);
  ~CompositeDifferentiable() override = default;

  /**
   * @brief Adds a differentiable component to this composite.
   *
   * @attention The child component's @ref DifferentiableInterface::isDynamic() must match this composite.
   */
  void addComponent(Differentiable::Ptr component);

  /** @brief Removes all components and resets row count to zero (or 1 for cost mode). */
  void clearComponents();

  /** @brief Returns read access to the components. */
  const ComponentVec& getComponents() const;

  /** @brief True if no components have been added. */
  bool empty() const;

  /**
   * @brief Access component with the specified name.
   * @param name The component name.
   * @return The differentiable component pointer.
   *
   * @warning This is a linear search. Prefer storing handles if performance matters.
   */
  Differentiable::Ptr getComponent(const std::string& name) const;

  // ---- DifferentiableInterface / ComponentInterface ----
  int update() override;
  Eigen::VectorXd getValues() const override;
  Eigen::VectorXd getCoefficients() const override;
  Jacobian getJacobian() const override;
  std::vector<Bounds> getBounds() const override;
  void print(int& index, double tolerance = 0.001) const override;

private:
  ComponentVec components_;
  mutable long n_vars_{ -1 };
};

}  // namespace trajopt_ifopt

#endif  // COMPOSITE_H
