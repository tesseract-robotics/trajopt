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

#include <trajopt_ifopt/trajopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/trajopt/variable_sets/nodes_observer.h>

namespace trajopt_ifopt
{
NodesVariables::NodesVariables(const std::string& name) : VariableSet(kSpecifyLater, name) {}

void NodesVariables::AddNode(Node node)
{
  Eigen::Index length = node.size();
  nodes_.emplace_back(n_dim_, std::move(node));
  n_dim_ += length;
}

const Node& NodesVariables::GetNode(std::size_t opt_idx) const { return nodes_.at(opt_idx).second; }

void NodesVariables::SetVariables(const VectorXd& x)
{
  for (auto& pair : nodes_)
    pair.second.setVariables(x.segment(pair.first, pair.second.size()));

  values_ = x;

  UpdateObservers();
}

Eigen::VectorXd NodesVariables::GetValues() const { return values_; }

void NodesVariables::UpdateObservers()
{
  for (auto& o : observers_)
    o->UpdateNodes();
}

void NodesVariables::AddObserver(std::shared_ptr<NodesObserver> observer) { observers_.push_back(observer); }

Eigen::Index NodesVariables::GetDim() const { return n_dim_; }

NodesVariables::VecBound NodesVariables::GetBounds() const { return bounds_; }

const std::vector<Node> NodesVariables::GetNodes() const
{
  std::vector<Node> nodes;
  nodes.reserve(nodes_.size());
  for (const auto& pair : nodes_)
    nodes.push_back(pair.second);
  return nodes;
}

}  // namespace trajopt_ifopt
