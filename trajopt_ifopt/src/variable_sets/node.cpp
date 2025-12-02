#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <memory>

namespace trajopt_ifopt
{
Node::Node(std::string node_name) : name_(std::move(node_name)) {}

const std::string& Node::getName() const { return name_; }
const NodesVariables* Node::getParent() const { return parent_; }
std::shared_ptr<const trajopt_ifopt::Var> Node::addVar(const std::string& name,
                                                       const std::vector<std::string>& child_names)
{
  auto var = std::make_shared<Var>(length_, child_names.size(), name, child_names, this);
  vars_[name] = var;
  length_ += static_cast<Eigen::Index>(child_names.size());
  return var;
}

std::shared_ptr<const trajopt_ifopt::Var> Node::addVar(const std::string& name)
{
  auto var = std::make_shared<Var>(length_, name, this);
  vars_[name] = var;
  length_ += 1;
  return var;
}

std::shared_ptr<const trajopt_ifopt::Var> Node::getVar(const std::string& name) const { return vars_.at(name); }

bool Node::hasVar(const std::string& name) const { return (vars_.find(name) != vars_.end()); }

Eigen::Index Node::size() const { return length_; }

void Node::setVariables(const Eigen::Ref<const Eigen::VectorXd>& x)
{
  for (auto& pair : vars_)
    pair.second->setVariables(x);
}

void Node::incrementIndex(Eigen::Index value)
{
  for (auto& pair : vars_)
    pair.second->incrementIndex(value);
}
}  // namespace trajopt_ifopt
