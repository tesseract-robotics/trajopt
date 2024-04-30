#ifndef TRAJOPT_IFOPT_TRAJOPT_VAR_H
#define TRAJOPT_IFOPT_TRAJOPT_VAR_H

#include <Eigen/Core>
#include <vector>
#include <memory>

namespace trajopt_ifopt
{
// struct VarRep
// {
//   VarRep(Eigen::Index _index, std::string _name);
//   VarRep(Eigen::Index _index, Eigen::Index _length, std::string _name, std::vector<std::string> _child_names);

//   Eigen::Index index{-1};
//   Eigen::Index length{-1};
//   std::string name;
//   std::vector<std::string> child_names;
// };

// class Var
// {
// public:
//   Var() = default;
//   ~Var() = default;
//   Var(std::shared_ptr<const VarRep> var_rep);
//   Var(const Var& other) = default;
//   Var& operator=(const Var&) = default;
//   Var(Var&&) = default;
//   Var& operator=(Var&&) = default;

//   template<typename T>
//   T value(const Eigen::VectorXd& x) const
//   {
//     throw std::runtime_error("This should never be used");
//   }

//   template<typename T>
//   T name() const
//   {
//     throw std::runtime_error("This should never be used");
//   }

// private:
//   friend class Node;
//   std::shared_ptr<const VarRep> var_rep_{ nullptr };
// };

// template<>
// double Var::value(const Eigen::VectorXd& x) const
// {
//   assert(var_rep_->index > -1 && var_rep_->index < x.size());
//   assert(var_rep_->child_names.empty());
//   return x[var_rep_->index];
// }

// template<>
// Eigen::VectorXd Var::value(const Eigen::VectorXd& x) const
// {
//   assert(!var_rep_->child_names.empty());
//   assert(var_rep_->index > -1 && var_rep_->index < x.size());
//   assert(var_rep_->length > -1 && (var_rep_->index + var_rep_->length) < x.size());
//   return x.segment(var_rep_->index, var_rep_->length);
// }

// template<>
// std::string Var::name() const
// {
//   assert(var_rep_->child_names.empty());
//   return var_rep_->name;
// }

// template<>
// std::vector<std::string> Var::name() const
// {
//   assert(!var_rep_->child_names.empty());
//   return var_rep_->child_names;
// }

struct VarRep
{
  VarRep(Eigen::Index _index, std::string _name);
  VarRep(Eigen::Index _index, Eigen::Index _length, std::string _identifier, std::vector<std::string> _names);

  Eigen::Index index{ -1 };
  Eigen::Index length{ -1 };
  std::string identifier;
  std::vector<std::string> names;
  Eigen::VectorXd values;
};

/**
 * @brief This is the class which the constraints should be storing
 * @details This class contains all informtion necessary for filling the jacobian
 */
class Var
{
public:
  Var() = default;
  ~Var() = default;
  Var(std::unique_ptr<VarRep> var_rep);
  Var(const Var& other) = default;
  Var& operator=(const Var&) = default;
  Var(Var&&) = default;
  Var& operator=(Var&&) = default;

  Eigen::Index getIndex() const { return var_rep_->index; }
  Eigen::Index getLength() const { return var_rep_->length; }

  template <typename T>
  T value() const
  {
    throw std::runtime_error("This should never be used");
  }

  template <typename T>
  T name() const
  {
    throw std::runtime_error("This should never be used");
  }

  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x)
  {
    assert(var_rep_->index > -1 && var_rep_->index < x.size());
    assert(var_rep_->length > 0 && (var_rep_->index + var_rep_->length) < x.size());
    var_rep_->values = x.segment(var_rep_->index, var_rep_->length);
  }

private:
  friend class Node;
  std::shared_ptr<VarRep> var_rep_{ nullptr };
};

template <>
inline double Var::value() const
{
  assert(var_rep_->values.size() == 1);
  assert(var_rep_->index > -1);
  assert(var_rep_->length == 1);
  return var_rep_->values[0];
}

template <>
inline Eigen::VectorXd Var::value() const
{
  assert(var_rep_->index > -1);
  assert(var_rep_->length > 1);
  assert(var_rep_->names.size() > 1);
  return var_rep_->values;
}

template <>
inline std::string Var::name() const
{
  assert(var_rep_->names.size() == 1);
  return var_rep_->names[0];
}

template <>
inline std::vector<std::string> Var::name() const
{
  assert(var_rep_->names.size() > 1);
  return var_rep_->names;
}
}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_TRAJOPT_VAR_H
