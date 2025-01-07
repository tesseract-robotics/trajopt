#ifndef TRAJOPT_IFOPT_VAR_H
#define TRAJOPT_IFOPT_VAR_H

#include <Eigen/Core>
#include <vector>
#include <memory>

namespace trajopt_ifopt
{
/**
 * @brief This is the class which the constraints should be storing
 * @details This class contains all information necessary for filling the jacobian
 * This was copied from the trajopt implementation of Var and VarRep
 */
class Var
{
public:
  Var() = default;
  ~Var() = default;
  Var(Eigen::Index index, std::string name);
  Var(Eigen::Index index, Eigen::Index length, std::string identifier, std::vector<std::string> names);
  Var(const Var& other) = default;
  Var& operator=(const Var&) = default;
  Var(Var&&) = default;
  Var& operator=(Var&&) = default;

  /**
   * @brief Set the variables. This is the full vector of variables and it will extract its variables from the full
   * list.
   * @param x The full vector of variables
   */
  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x);

  /**
   * @brief Get the variable size
   * @return The size
   */
  Eigen::Index size() const;

  template <typename T>
  const T& value() const
  {
    throw std::runtime_error("This should never be used");
  }

  template <typename T>
  const T& name() const
  {
    throw std::runtime_error("This should never be used");
  }

private:
  friend class Node;
  Eigen::Index index_{ -1 };
  Eigen::Index length_{ -1 };
  std::string identifier_;
  std::vector<std::string> names_;
  Eigen::VectorXd values_;

  /**
   * @brief Get the index in the full set of variables that these are stored
   * @return The start index
   */
  Eigen::Index getIndex() const;

  /**
   * @brief Increment the start index by the provided value
   * @param value The value to increment the start index by
   */
  void incrementIndex(Eigen::Index value);
};

template <>
inline const double& Var::value() const
{
  assert(values_.size() == 1);
  assert(index_ > -1);
  assert(length_ == 1);
  return values_[0];
}

template <>
inline const Eigen::VectorXd& Var::value() const
{
  assert(index_ > -1);
  assert(length_ > 1);
  assert(names_.size() > 1);
  return values_;
}

template <>
inline const std::string& Var::name() const
{
  assert(names_.size() == 1);
  return names_[0];
}

template <>
inline const std::vector<std::string>& Var::name() const
{
  assert(names_.size() > 1);
  return names_;
}
}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_VAR_H
