#ifndef TRAJOPT_IFOPT_JOINT_POSITION_VARIABLE_H
#define TRAJOPT_IFOPT_JOINT_POSITION_VARIABLE_H

#include <ifopt/variable_set.h>
#include <ifopt/bounds.h>

#include <Eigen/Eigen>

namespace trajopt
{
/**
 * @brief Represents a single joint position in the optimization. Values are of dimension 1 x n_dof
 */
class JointPosition : public ifopt::VariableSet
{
public:
  using Ptr = std::shared_ptr<JointPosition>;
  using ConstPtr = std::shared_ptr<const JointPosition>;

  JointPosition(const Eigen::Ref<const Eigen::VectorXd>& init_value, const std::string& name = "Joint_Position")
    : ifopt::VariableSet(static_cast<int>(init_value.size()), name)
  {
    // This needs to be set somehow
    ifopt::Bounds bounds(-M_PI, M_PI);
    bounds_ = std::vector<ifopt::Bounds>(static_cast<size_t>(init_value.size()), bounds);
    values_ = init_value;
  }

  /**
   * @brief Sets this variable to the given joint position
   * @param x Joint Position to which this variable will be set.
   */
  void SetVariables(const Eigen::VectorXd& x) override { values_ = x; }

  /**
   * @brief Gets the joint position associated with this variable
   * @return This variable's joint position
   */
  Eigen::VectorXd GetValues() const override { return values_; }

  /**
   * @brief Gets the bounds on this variable
   * @return Bounds on this variable
   */
  VecBound GetBounds() const override { return bounds_; }

  /**
   * @brief Sets the bounds for the joints in this variable
   * @param new_bounds New bounds for the joints
   */
  void SetBounds(VecBound& new_bounds) { bounds_ = new_bounds; }

private:
  VecBound bounds_;
  Eigen::VectorXd values_;
};

}  // namespace trajopt

#endif
