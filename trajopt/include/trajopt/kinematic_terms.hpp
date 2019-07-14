#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>

namespace trajopt
{
/**
 * @brief Used to calculate the error for CartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct DynamicCartPoseErrCalculator : public TrajOptVectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string target_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_target_;
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  Eigen::Isometry3d tcp_;
  DynamicCartPoseErrCalculator(const std::string& target,
                               tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                               tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                               Eigen::Isometry3d world_to_base,
                               std::string link,
                               Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : target_(target)
    , manip_(manip)
    , adjacency_map_(adjacency_map)
    , world_to_base_(world_to_base)
    , link_(link)
    , tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
    kin_target_ = adjacency_map_->getLinkMapping(target_);
  }

  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/**
 * @brief Used to calculate the error for StaticCartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartPoseErrCalculator : public TrajOptVectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Isometry3d pose_inv_;
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  Eigen::Isometry3d tcp_;
  CartPoseErrCalculator(const Eigen::Isometry3d& pose,
                        tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                        tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                        Eigen::Isometry3d world_to_base,
                        std::string link,
                        Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : pose_inv_(pose.inverse())
    , manip_(manip)
    , adjacency_map_(adjacency_map)
    , world_to_base_(world_to_base)
    , link_(link)
    , tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/**
 * @brief Used to calculate the jacobian for CartVelTermInfo
 *
 */
struct CartVelJacCalculator : sco::MatrixOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartVelJacCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                       Eigen::Isometry3d world_to_base,
                       std::string link,
                       double limit,
                       Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : manip_(manip), adjacency_map_(adjacency_map), world_to_base_(world_to_base), link_(link), limit_(limit), tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  Eigen::MatrixXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/**
 * @brief  Used to calculate the error for CartVelTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartVelErrCalculator : sco::VectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartVelErrCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                       Eigen::Isometry3d world_to_base,
                       std::string link,
                       double limit,
                       Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : manip_(manip), adjacency_map_(adjacency_map), world_to_base_(world_to_base), link_(link), limit_(limit), tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

struct JointVelErrCalculator : sco::VectorOfVector
{
  /** @brief Velocity target */
  double target_;
  /** @brief Upper tolerance */
  double upper_tol_;
  /** @brief Lower tolerance */
  double lower_tol_;
  JointVelErrCalculator() : target_(0.0), upper_tol_(0.0), lower_tol_(0.0) {}
  JointVelErrCalculator(double target, double upper_tol, double lower_tol)
    : target_(target), upper_tol_(upper_tol), lower_tol_(lower_tol)
  {
  }
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct JointVelJacCalculator : sco::MatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct JointAccErrCalculator : sco::VectorOfVector
{
  JointVelErrCalculator vel_calc;
  double limit_;
  JointAccErrCalculator() : limit_(0.0) {}
  JointAccErrCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct JointAccJacCalculator : sco::MatrixOfVector
{
  JointVelErrCalculator vel_calc;
  JointVelJacCalculator vel_jac_calc;
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct JointJerkErrCalculator : sco::VectorOfVector
{
  JointAccErrCalculator acc_calc;
  double limit_;
  JointJerkErrCalculator() : limit_(0.0) {}
  JointJerkErrCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct JointJerkJacCalculator : sco::MatrixOfVector
{
  JointAccErrCalculator acc_calc;
  JointAccJacCalculator acc_jac_calc;
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct TimeCostCalculator : sco::VectorOfVector
{
  /** @brief The time target (s). This is subtracted from the cost, so only set limit!=0 if the penalty type is a hinge
   * (or you could get negatives)*/
  double limit_;
  TimeCostCalculator() : limit_(0.0) {}
  TimeCostCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct TimeCostJacCalculator : sco::MatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const;
};

}  // namespace trajopt
