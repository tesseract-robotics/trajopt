#ifndef TRAJOPT_IFOPT_TRAJOPT_NODE_H
#define TRAJOPT_IFOPT_TRAJOPT_NODE_H

#include <trajopt_ifopt/trajopt/variable_sets/var.h>
#include <unordered_map>
#include <boost/uuid/uuid.hpp>

namespace trajopt_ifopt
{
class NodesVariables;
class Node
{
public:
  Node(std::string node_name = "Node");

  boost::uuids::uuid getUUID() const;

  const std::string& getName() const;

  NodesVariables* const getParent() const;

  Var addVar(const std::string& name);

  Var addVar(const std::string& name, const std::vector<std::string>& child_names);

  Var getVar(const std::string& name) const;

  bool hasVar(const std::string& name) const;

  Eigen::Index size() const;

protected:
  friend class NodesVariables;
  boost::uuids::uuid uuid_{};
  std::string name_;
  std::unordered_map<std::string, Var> vars_;
  Eigen::Index length_{ 0 };
  NodesVariables* parent_{ nullptr };

  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x);
  void incrementIndex(Eigen::Index value);
};

}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_TRAJOPT_NODE_H
