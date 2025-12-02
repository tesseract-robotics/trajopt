#ifndef TRAJOPT_IFOPT_VAR_H
#define TRAJOPT_IFOPT_VAR_H

#include <Eigen/Core>
#include <vector>

namespace trajopt_ifopt
{
class Node;
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
  Var(Eigen::Index index, std::string name, Node* parent = nullptr);
  Var(Eigen::Index index,
      Eigen::Index length,
      std::string identifier,
      std::vector<std::string> names,
      Node* parent = nullptr);
  Var(const Var& other) = default;
  Var& operator=(const Var&) = default;
  Var(Var&&) = default;
  Var& operator=(Var&&) = default;

  /** @brief Get the identifier for the variable */
  const std::string& getIdentifier() const;

  /** @brief Get the parent node */
  const Node* getParent() const;

  /**
   * @brief Set the variables. This is the full vector of variables and it will extract its variables from the full
   * list.
   * @param x The full vector of variables
   */
  void setVariables(const Eigen::Ref<const Eigen::VectorXd>& x);

  /**
   * @brief Get the index in the full set of variables that these are stored
   * @return The start index
   */
  Eigen::Index getIndex() const;

  /**
   * @brief Get the variable size
   * @return The size
   */
  Eigen::Index size() const;

  /**
   * @brief Get variable values
   * @return The values
   */
  const Eigen::VectorXd& value() const;

  /**
   * @brief Get teh variable names
   * @return The names
   */
  const std::vector<std::string>& name() const;

private:
  friend class Node;
  Eigen::Index index_{ -1 };
  Eigen::Index length_{ -1 };
  std::string identifier_;
  std::vector<std::string> names_;
  Eigen::VectorXd values_;
  Node* parent_{ nullptr };

  /**
   * @brief Increment the start index by the provided value
   * @param value The value to increment the start index by
   */
  void incrementIndex(Eigen::Index value);
};

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_VAR_H
