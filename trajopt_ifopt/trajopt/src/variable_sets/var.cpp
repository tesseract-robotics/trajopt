#include <trajopt_ifopt/trajopt/variable_sets/var.h>

namespace trajopt_ifopt
{
// VarRep::VarRep(Eigen::Index _index, std::string _name)
//   : index(_index), name(std::move(_name)) {}
// VarRep::VarRep(Eigen::Index _index, Eigen::Index _length, std::string _name, std::vector<std::string> _child_names)
//   : index(_index), length(_length), name(std::move(_name)), child_names(std::move(_child_names)) {}

// Var::Var(std::shared_ptr<const VarRep> var_rep) : var_rep_(std::move(var_rep)) {}

Var::Var(Eigen::Index index, std::string name)
  : index_(index), identifier_(std::move(name)), names_({ identifier_ }), values_(Eigen::VectorXd::Zero(1))
{
}

Var::Var(Eigen::Index index, Eigen::Index length, std::string identifier, std::vector<std::string> names)
  : index_(index)
  , length_(length)
  , identifier_(std::move(identifier))
  , names_(std::move(names))
  , values_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(names.size())))
{
}

Eigen::Index Var::getIndex() const { return index_; }
Eigen::Index Var::size() const { return length_; }

void Var::incrementIndex(Eigen::Index value) { index_ += value; }
void Var::setVariables(const Eigen::Ref<const Eigen::VectorXd>& x)
{
  assert(index_ > -1 && index_ < x.size());
  assert(length_ > 0 && (index_ + length_) < x.size());
  values_ = x.segment(index_, length_);
}

}  // namespace trajopt_ifopt
