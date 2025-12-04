#ifndef TRAJOPT_IFOPT_NODE_H
#define TRAJOPT_IFOPT_NODE_H

#include <trajopt_ifopt/variable_sets/var.h>
#include <unordered_map>
#include <memory>

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

  const NodesVariables* getParent() const;

  bool hasVar(const std::string& name) const;

  std::shared_ptr<const Var> addVar(const std::string& name, double value, ifopt::Bounds bounds = ifopt::NoBound);

  std::shared_ptr<const Var> addVar(const std::string& name,
                                    const std::vector<std::string>& child_names,
                                    const Eigen::VectorXd& values,
                                    const std::vector<ifopt::Bounds>& bounds);

  std::shared_ptr<const Var> getVar(const std::string& name) const;

  Eigen::VectorXd getValues() const;

  std::vector<ifopt::Bounds> getBounds() const;

  Eigen::Index size() const;

  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x);

protected:
  friend class NodesVariables;
  std::string name_;
  std::vector<std::shared_ptr<Var>> vars_;
  std::unordered_map<std::string, std::shared_ptr<Var>> vars_by_name_;
  Eigen::Index length_{ 0 };
  NodesVariables* parent_{ nullptr };

  void incrementIndex(Eigen::Index value);
};

}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_NODE_H
