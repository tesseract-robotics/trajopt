#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <memory>

namespace trajopt_ifopt
{
Node::Node(std::string node_name) : name_(std::move(node_name)) {}

const std::string& Node::getName() const { return name_; }
const NodesVariables* Node::getParent() const { return parent_; }

bool Node::hasVar(const std::string& name) const { return (vars_by_name_.find(name) != vars_by_name_.end()); }

std::shared_ptr<const Var> Node::addVar(const std::string& name, double value, ifopt::Bounds bounds)
{
  vars_.emplace_back(std::make_shared<Var>(length_, name, value, bounds, this));
  vars_by_name_[name] = vars_.back();
  length_ += 1;
  return vars_.back();
}

std::shared_ptr<const Var> Node::addVar(const std::string& name,
                                        const std::vector<std::string>& child_names,
                                        const Eigen::VectorXd& values,
                                        const std::vector<ifopt::Bounds>& bounds)
{
  vars_.emplace_back(std::make_shared<Var>(length_, name, child_names, values, bounds, this));
  vars_by_name_[name] = vars_.back();
  length_ += static_cast<Eigen::Index>(child_names.size());
  return vars_.back();
}

std::shared_ptr<const Var> Node::getVar(const std::string& name) const { return vars_by_name_.at(name); }

Eigen::VectorXd Node::getValues() const
{
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(length_));
  for (const auto& var : vars_)
  {
    const Eigen::VectorXd& value = var->value();
    values.insert(values.end(), value.data(), value.data() + value.size());
  }
  return Eigen::Map<Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

std::vector<ifopt::Bounds> Node::getBounds() const
{
  std::vector<ifopt::Bounds> bounds;
  for (const auto& var : vars_)
  {
    const std::vector<ifopt::Bounds>& var_bounds = var->bounds();
    bounds.insert(bounds.end(), var_bounds.begin(), var_bounds.end());
  }
  return bounds;
}

Eigen::Index Node::size() const { return length_; }

void Node::setVariables(const Eigen::Ref<const Eigen::VectorXd>& x)
{
  for (auto& var : vars_)
    var->setVariables(x);
}

void Node::incrementIndex(Eigen::Index value)
{
  for (auto& var : vars_)
    var->incrementIndex(value);
}
}  // namespace trajopt_ifopt
