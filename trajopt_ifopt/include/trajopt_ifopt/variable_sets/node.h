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
#ifndef TRAJOPT_IFOPT_NODE_H
#define TRAJOPT_IFOPT_NODE_H

#include <trajopt_ifopt/core/bounds.h>

#include <Eigen/Core>
#include <unordered_map>
#include <memory>
#include <vector>

namespace trajopt_ifopt
{
class NodesVariables;
class Var;

/**
 * @brief Group of related variables in the trajopt_ifopt problem.
 *
 * A Node encapsulates one or more @ref Var instances that together form a
 * contiguous segment (or set of segments) of the global decision vector.
 * Each variable is identified by a string name and may itself be scalar or
 * vector-valued.
 *
 * Typical usage:
 * - Create a Node with a descriptive name (e.g. "manipulator_states").
 * - Add variables via @ref addVar().
 * - Query all values and bounds via @ref getValues() and @ref getBounds().
 * - Let @ref NodesVariables manage indexing and variable layout.
 */
class Node
{
public:
  /**
   * @brief Construct a Node with an optional name.
   *
   * @param node_name Descriptive name for this node (for logging/debugging).
   */
  explicit Node(std::string node_name = "Node");
  ~Node();
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;
  Node(Node&&) = default;
  Node& operator=(Node&&) = default;

  /**
   * @brief Get the name of this node.
   *
   * @return Constant reference to the node name.
   */
  const std::string& getName() const;

  /**
   * @brief Get the parent NodesVariables container.
   *
   * The parent is responsible for managing the layout of this Node's variables
   * in the global decision vector.
   *
   * @return Pointer to the parent NodesVariables, or nullptr if not attached.
   */
  const NodesVariables* getParent() const;

  /**
   * @brief Check whether this node already has a variable by the given name.
   *
   * @param name Variable identifier (as passed to @ref addVar()).
   * @return True if a variable with the given name exists, false otherwise.
   */
  bool hasVar(const std::string& name) const;

  /**
   * @brief Add a scalar variable to this node.
   *
   * @param name   Variable name (must be unique within this node).
   * @param value  Initial scalar value.
   * @param bounds Bounds associated with the variable (default: unbounded).
   *
   * @return Shared pointer to the newly created @ref Var. The returned pointer
   *         is shared so it can be held by constraints or other components.
   *
   * @note The global index of the variable (within the full decision vector)
   *       is managed by @ref NodesVariables and will be updated via
   *       @ref incrementIndex() when nodes are assembled.
   */
  std::shared_ptr<const Var> addVar(const std::string& name, double value, Bounds bounds = NoBound);

  /**
   * @brief Add a vector-valued variable to this node.
   *
   * @param name        Variable name (identifier) for this block.
   * @param child_names Per-element names (size must match @p values.size()).
   * @param values      Initial values for each element.
   * @param bounds      Per-element bounds (size must match @p values.size()).
   *
   * @return Shared pointer to the newly created @ref Var.
   *
   * This is used for multi-DOF or grouped variables (e.g., a joint group).
   * The @p name identifies the block, while @p child_names provides more
   * detailed labels per element.
   */
  std::shared_ptr<const Var> addVar(const std::string& name,
                                    const std::vector<std::string>& child_names,
                                    const Eigen::VectorXd& values,
                                    const std::vector<Bounds>& bounds);

  /**
   * @brief Get a variable by name.
   *
   * @param name Variable name as passed to @ref addVar().
   * @return Shared pointer to the corresponding @ref Var, or nullptr if not found.
   */
  std::shared_ptr<const Var> getVar(const std::string& name) const;

  /**
   * @brief Get all variable values for this node as a single vector.
   *
   * The returned vector is the concatenation of the values of all variables
   * owned by this node, in the internal order of @ref vars_.
   *
   * @return Concatenated vector of all variable values in this node.
   */
  Eigen::VectorXd getValues() const;

  /**
   * @brief Get all variable bounds for this node as a single vector.
   *
   * The returned vector is the concatenation of the bounds for all variables
   * owned by this node, in the same order as @ref getValues().
   *
   * @return Concatenated vector of all bounds in this node.
   */
  std::vector<Bounds> getBounds() const;

  /**
   * @brief Get the total number of scalar decision variables in this node.
   *
   * @return The sum of the sizes of all @ref Var instances owned by this node.
   */
  Eigen::Index size() const;

  /**
   * @brief Update all variable values from the full decision vector.
   *
   * This function extracts the segments of @p x corresponding to this node's
   * variables (based on their indices and sizes) and updates each @ref Var's
   * internal value vector.
   *
   * @param x Full decision vector for the optimization problem.
   *
   * @note It is assumed that all variable segments managed by this node have
   *       already been assigned valid indices by @ref NodesVariables.
   */
  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x);

protected:
  friend class NodesVariables;

  /**
   * @brief Descriptive name for this node.
   */
  std::string name_;

  /**
   * @brief Variables owned by this node.
   *
   * Each Var may represent a scalar or vector-valued block of the global
   * decision vector. The order in this vector defines the order in
   * @ref getValues() and @ref getBounds().
   */
  std::vector<std::shared_ptr<Var>> vars_;

  /**
   * @brief Lookup table from variable name to Var pointer.
   *
   * This allows fast access to variables by their string name.
   */
  std::unordered_map<std::string, std::shared_ptr<Var>> vars_by_name_;

  /**
   * @brief Total number of scalar variables owned by this node.
   *
   * This is the sum of the sizes of all variables in @ref vars_. It is
   * typically maintained by @ref NodesVariables when variables are added or
   * when indices are updated.
   */
  Eigen::Index length_{ 0 };

  /**
   * @brief Pointer to the parent NodesVariables container.
   */
  NodesVariables* parent_{ nullptr };

  /**
   * @brief Increment the starting index of all variables in this node.
   *
   * This is used by @ref NodesVariables when constructing the global layout
   * of the decision vector: after placing previous nodes, each subsequent
   * node's variable indices must be shifted by the number of already-placed
   * variables.
   *
   * @param value Offset by which to increment each variable's starting index.
   */
  void incrementIndex(Eigen::Index value);
};

}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_NODE_H
