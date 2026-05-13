/**
 * @author Levi Armstrong
 * @date Jan 7, 2025
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TRAJOPT_IFOPT_VAR_H
#define TRAJOPT_IFOPT_VAR_H

#include <Eigen/Core>
#include <vector>
#include <trajopt_ifopt/core/bounds.h>

namespace trajopt_ifopt
{
class Node;
/**
 * @brief Variable segment used by trajopt_ifopt.
 *
 * This class represents a *contiguous* block of decision variables belonging to
 * a single @ref Node. It stores:
 *
 * - The starting index of its block within the full optimization vector
 * - A stable identifier string for logging / debugging
 * - Per-element names (for debugging / introspection)
 * - The current values of the variables
 * - Per-element bounds (ifopt::Bounds)
 * - A pointer back to the owning Node
 *
 * Constraints should store @ref Var instances (or pointers/references to them)
 * and use @ref getIndex(), @ref size(), and @ref value() to correctly fill
 * Jacobian and residual entries.
 *
 * The class supports both scalar and vector-valued variables:
 * - Scalar: use the single-value constructor
 * - Vector: use the multi-value constructor
 */
class Var
{
public:
  Var() = default;
  ~Var() = default;

  /**
   * @brief Construct a scalar variable.
   *
   * @param index     Starting index of this variable in the full decision vector.
   * @param name      Identifier for this variable (also used as its element name).
   * @param value     Initial value of the scalar variable.
   * @param bounds    Bounds associated with this scalar (default: unbounded).
   * @param parent    Owning node (may be nullptr if not attached yet).
   *
   * This constructor creates a variable segment of size 1. The @p name is used
   * both as the identifier and as the single element name in @ref names_.
   */
  Var(Eigen::Index index, std::string name, double value, Bounds bounds = NoBound, Node* parent = nullptr);

  /**
   * @brief Construct a vector-valued variable.
   *
   * @param index     Starting index of the first element in the full decision vector.
   * @param identifier Stable identifier for this variable segment (e.g. joint group name).
   * @param names     Per-element names (size must match @p values.size()).
   * @param values    Initial values for each element of the variable segment.
   * @param bounds    Per-element bounds (size must match @p values.size()).
   * @param parent    Owning node (may be nullptr if not attached yet).
   *
   * This constructor creates a variable segment of size @c values.size(). The
   * @p identifier is stored in @ref identifier_ and is intended for debugging
   * and logging; the human-readable per-element names are stored in
   * @ref names_.
   */
  Var(Eigen::Index index,
      std::string identifier,
      std::vector<std::string> names,
      const Eigen::VectorXd& values,
      const std::vector<Bounds>& bounds,
      Node* parent = nullptr);

  Var(const Var& other) = default;
  Var& operator=(const Var&) = default;
  Var(Var&&) = default;
  Var& operator=(Var&&) = default;

  /**
   * @brief Get the identifier for the variable segment.
   *
   * The identifier is a stable label for this block of variables, typically used
   * for logging, debugging, or grouping (e.g., a joint group name or task name).
   *
   * @return Constant reference to the identifier string.
   */
  const std::string& getIdentifier() const;

  /**
   * @brief Get the parent node.
   *
   * The parent node is the object that owns this @ref Var and is responsible
   * for interpreting its values (e.g., mapping them to robot states).
   *
   * @return Pointer to the parent node, or nullptr if not attached.
   */
  const Node* getParent() const;

  /**
   * @brief Set the variable values from the full decision vector.
   *
   * This function extracts the subsegment of @p x that corresponds to this
   * variable block, based on @ref index_ and @ref size(), and stores it in
   * @ref values_.
   *
   * @param x The full decision vector for the optimization problem.
   *
   * @note It is the caller's responsibility to ensure that
   *       @c index_ + size() does not exceed @c x.size().
   */
  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x);

  /**
   * @brief Get the starting index of this variable segment.
   *
   * This is the index in the full decision vector where the first element of
   * this variable block resides.
   *
   * @return The starting index within the full decision vector.
   */
  Eigen::Index getIndex() const;

  /**
   * @brief Get the number of elements in this variable segment.
   *
   * @return The number of scalar entries (1 for a scalar, N for a vector of length N).
   */
  Eigen::Index size() const;

  /**
   * @brief Get the current variable values.
   *
   * @return Constant reference to the internal value vector.
   *
   * The returned vector has length @ref size(). The i-th element corresponds to
   * the decision variable at global index @c getIndex() + i.
   */
  const Eigen::VectorXd& value() const;

  /**
   * @brief Get the variable names.
   *
   * @return Constant reference to the per-element names.
   *
   * The size of the returned vector matches @ref size(). Each entry is a
   * human-readable name associated with the corresponding element of
   * @ref value().
   */
  const std::vector<std::string>& name() const;

  /**
   * @brief Get the variable bounds.
   *
   * @return Constant reference to the per-element bounds.
   *
   * The size of the returned vector matches @ref size(). Each entry is an
   * ifopt::Bounds object describing lower and upper bounds for the
   * corresponding element.
   */
  const std::vector<Bounds>& bounds() const;

private:
  friend class Node;

  /**
   * @brief Starting index of this segment in the full decision vector.
   *
   * A value of -1 indicates an uninitialized / invalid index.
   */
  Eigen::Index index_{ -1 };

  /**
   * @brief Identifier for this variable segment.
   *
   * Intended primarily for logging and debugging. This does not need to be
   * unique globally, but should be meaningful within the context of the parent
   * node or problem.
   */
  std::string identifier_;

  /**
   * @brief Per-element variable names.
   *
   * This vector has the same length as @ref values_ and @ref bounds_. It
   * provides human-readable labels for debugging, logging, and introspection.
   */
  std::vector<std::string> names_;

  /**
   * @brief Current values of the variable segment.
   *
   * The size of this vector defines the dimension of the variable block.
   */
  Eigen::VectorXd values_;

  /**
   * @brief Per-element bounds for the variable segment.
   *
   * This vector has the same length as @ref values_. Each element is an
   * ifopt::Bounds describing the lower and upper bounds of the corresponding
   * variable.
   */
  std::vector<Bounds> bounds_;

  /**
   * @brief Pointer to the parent node that owns this variable.
   */
  Node* parent_{ nullptr };

  /**
   * @brief Increment the starting index by the provided offset.
   *
   * This is typically used when variables are laid out sequentially in the full
   * decision vector and a new block is appended before this one.
   *
   * @param value The offset by which to increment @ref index_.
   */
  void incrementIndex(Eigen::Index value);
};

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_VAR_H
