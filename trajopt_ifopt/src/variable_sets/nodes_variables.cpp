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

#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/nodes_observer.h>
#include <trajopt_ifopt/variable_sets/node.h>

namespace trajopt_ifopt
{
NodesVariables::NodesVariables(std::string name, std::vector<std::unique_ptr<Node>> nodes)
  : VariableSet(std::move(name), kSpecifyLater)
{
  nodes_.reserve(nodes.size());
  for (auto& node : nodes)
    AddNode(std::move(node));

  // Set the size
  SetRows(static_cast<int>(n_dim_));

  // Get the initial values
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(n_dim_));

  for (const auto& node : nodes_)
  {
    Eigen::VectorXd node_values = node->getValues();
    values.insert(values.end(), node_values.data(), node_values.data() + node_values.size());
  }

  values_ = Eigen::Map<Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
  assert(values_.size() == bounds_.size());
}

void NodesVariables::AddNode(std::unique_ptr<Node> node)
{
  node->incrementIndex(n_dim_);
  node->parent_ = this;

  Eigen::Index length = node->size();
  std::vector<Bounds> bounds = node->getBounds();

  nodes_.emplace_back(std::move(node));
  bounds_.insert(bounds_.end(), bounds.begin(), bounds.end());
  n_dim_ += length;
}

std::shared_ptr<const Node> NodesVariables::GetNode(std::size_t opt_idx) const { return nodes_.at(opt_idx); }

void NodesVariables::SetVariables(const Eigen::VectorXd& x)
{
  for (auto& node : nodes_)
    node->setVariables(x);

  values_ = x;

  UpdateObservers();
}

Eigen::VectorXd NodesVariables::GetValues() const { return values_; }

void NodesVariables::UpdateObservers()
{
  for (auto& o : observers_)
    o->UpdateNodes();
}

void NodesVariables::AddObserver(std::shared_ptr<NodesObserver> observer) { observers_.push_back(std::move(observer)); }

Eigen::Index NodesVariables::GetDim() const { return n_dim_; }

std::vector<Bounds> NodesVariables::GetBounds() const { return bounds_; }

std::vector<std::shared_ptr<const Node>> NodesVariables::GetNodes() const
{
  std::vector<std::shared_ptr<const Node>> nodes;
  nodes.reserve(nodes_.size());
  std::copy(nodes_.begin(), nodes_.end(), std::back_inserter(nodes));
  return nodes;
}

}  // namespace trajopt_ifopt
