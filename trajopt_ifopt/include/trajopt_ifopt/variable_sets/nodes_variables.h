/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

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

#ifndef TRAJOPT_IFOPT_NODES_VARIABLES_H
#define TRAJOPT_IFOPT_NODES_VARIABLES_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/variable_set.h>
#include <ifopt/bounds.h>
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/variable_sets/node.h>

namespace trajopt_ifopt
{
class NodesObserver;

class NodesVariables : public ifopt::VariableSet
{
public:
  using Ptr = std::shared_ptr<NodesVariables>;

  /**
   * @param variable_name  The name of the variables in the optimization problem.
   */
  NodesVariables(const std::string& variable_name, std::vector<std::unique_ptr<Node>> nodes);
  ~NodesVariables() override = default;

  /**
   * @brief Get node based on index
   * @param opt_idx The node index
   * @return The node
   */
  std::shared_ptr<const Node> GetNode(std::size_t opt_idx) const;

  /**
   * @brief Pure optimization variables that define the nodes.
   *
   * Not all node position and velocities are independent or optimized over, so
   * usually the number of optimization variables is less than all nodes' pos/vel.
   *
   * @sa GetNodeInfoAtOptIndex()
   */
  VectorXd GetValues() const override;

  /**
   * @brief Sets some node positions and velocity from the optimization variables.
   * @param x The optimization variables.
   *
   * Not all node position and velocities are independent or optimized over, so
   * usually the number of optimization variables is less than
   * all nodes pos/vel.
   *
   * @sa GetNodeValuesInfo()
   */
  void SetVariables(const VectorXd& x) override;

  /**
   * @returns the bounds on position and velocity of each node and dimension.
   */
  VecBound GetBounds() const override;

  /**
   * @returns All the nodes that can be used to reconstruct the spline.
   */
  std::vector<std::shared_ptr<const Node>> GetNodes() const;

  /**
   * @brief Adds a dependent observer that gets notified when the nodes change.
   * @param spline Usually a pointer to a spline which uses the node values.
   */
  void AddObserver(std::shared_ptr<NodesObserver> observer);

  /**
   * @returns  The dimensions (x,y,z) of every node.
   */
  Eigen::Index GetDim() const;

protected:
  Eigen::VectorXd values_;
  VecBound bounds_;  ///< the bounds on the node values.
  std::vector<std::shared_ptr<Node>> nodes_;
  Eigen::Index n_dim_{ 0 };
  std::vector<std::shared_ptr<NodesObserver>> observers_;

  /**
   * @brief Add node to the variable set
   * @param node The node to append
   */
  void AddNode(std::unique_ptr<Node> node);

  /** @brief Notifies the subscribed observers that the node values changes. */
  void UpdateObservers();
};

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_NODES_VARIABLES_H
