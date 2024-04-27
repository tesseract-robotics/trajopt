#include <trajopt_ifopt/trajopt/variable_sets/node.h>
#include <memory>

namespace trajopt_ifopt
{
Node::Node(std::string node_name) : name_(std::move(node_name)) {}

Var Node::addVar(const std::string& name, const std::vector<std::string>& child_names)
{
  Var var(std::make_unique<VarRep>(length_, child_names.size(), name, child_names));
  vars_[name] = var;
  length_ += static_cast<Eigen::Index>(child_names.size());
  return var;
}

Var Node::addVar(const std::string& name)
{
  Var var(std::make_unique<VarRep>(length_, name));
  vars_[name] = var;
  length_ += 1;
  return var;
}

Var Node::getVar(const std::string& name) const { return vars_.at(name); }

bool Node::hasVar(const std::string& name) const { return (vars_.find(name) != vars_.end()); }

Eigen::Index Node::size() const { return length_; }

void Node::setVariables(const Eigen::Ref<const Eigen::VectorXd>& x)
{
  for (auto& pair : vars_)
    pair.second.setVariables(x);
}
}  // namespace trajopt_ifopt
