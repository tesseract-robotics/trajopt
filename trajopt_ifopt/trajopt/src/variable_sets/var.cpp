#include <trajopt_ifopt/trajopt/variable_sets/var.h>

namespace trajopt_ifopt
{
// VarRep::VarRep(Eigen::Index _index, std::string _name)
//   : index(_index), name(std::move(_name)) {}
// VarRep::VarRep(Eigen::Index _index, Eigen::Index _length, std::string _name, std::vector<std::string> _child_names)
//   : index(_index), length(_length), name(std::move(_name)), child_names(std::move(_child_names)) {}

// Var::Var(std::shared_ptr<const VarRep> var_rep) : var_rep_(std::move(var_rep)) {}

VarRep::VarRep(Eigen::Index _index, std::string _name)
  : index(_index), identifier(std::move(_name)), names({ identifier }), values(Eigen::VectorXd::Zero(1))
{
}

VarRep::VarRep(Eigen::Index _index, Eigen::Index _length, std::string _identifier, std::vector<std::string> _names)
  : index(_index)
  , length(_length)
  , identifier(std::move(_identifier))
  , names(std::move(_names))
  , values(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(names.size())))
{
}

Var::Var(std::unique_ptr<VarRep> var_rep) : var_rep_(std::move(var_rep)) {}
}  // namespace trajopt_ifopt
