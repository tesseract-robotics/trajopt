#ifndef TRAJOPT_IFOPT_TRAJOPT_NODE_H
#define TRAJOPT_IFOPT_TRAJOPT_NODE_H

#include <trajopt_ifopt/trajopt/variable_sets/var.h>
#include <unordered_map>

namespace trajopt_ifopt
{
class NodesVariables;
class Node
{
public:
  Node(std::string node_name = "Node");
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;
  Node(Node&&) = default;
  Node& operator=(Node&&) = default;

  const std::string& getName() const;

  NodesVariables* getParent() const;

  std::shared_ptr<const Var> addVar(const std::string& name);

  std::shared_ptr<const Var> addVar(const std::string& name, const std::vector<std::string>& child_names);

  std::shared_ptr<const Var> getVar(const std::string& name) const;

  bool hasVar(const std::string& name) const;

  Eigen::Index size() const;

  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x);

  void incrementIndex(Eigen::Index value);

protected:
  friend class NodesVariables;
  std::string name_;
  std::unordered_map<std::string, std::shared_ptr<Var>> vars_;
  Eigen::Index length_{ 0 };
  NodesVariables* parent_{ nullptr };
};

}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_TRAJOPT_NODE_H
