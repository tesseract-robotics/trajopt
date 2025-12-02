#include <trajopt_ifopt/variable_sets/var.h>

namespace trajopt_ifopt
{
Var::Var(Eigen::Index index, std::string name, Node* parent)
  : index_(index)
  , identifier_(std::move(name))
  , names_({ identifier_ })
  , values_(Eigen::VectorXd::Zero(1))
  , parent_(parent)
{
}

Var::Var(Eigen::Index index, Eigen::Index length, std::string identifier, std::vector<std::string> names, Node* parent)
  : index_(index)
  , length_(length)
  , identifier_(std::move(identifier))
  , names_(std::move(names))
  , values_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(names_.size())))
  , parent_(parent)
{
}

const std::string& Var::getIdentifier() const { return identifier_; }
const Node* Var::getParent() const { return parent_; }
Eigen::Index Var::getIndex() const { return index_; }
Eigen::Index Var::size() const { return length_; }
const Eigen::VectorXd& Var::value() const { return values_; }
const std::vector<std::string>& Var::name() const { return names_; }

void Var::incrementIndex(Eigen::Index value) { index_ += value; }
void Var::setVariables(const Eigen::Ref<const Eigen::VectorXd>& x)
{
  assert(index_ > -1 && index_ < x.size());
  assert(length_ > 0 && (index_ + (length_ - 1)) < x.size());
  values_ = x.segment(index_, length_);
}

}  // namespace trajopt_ifopt
