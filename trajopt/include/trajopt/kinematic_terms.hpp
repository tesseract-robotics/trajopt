#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <console_bridge/console.h>
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

  /** @brief Manipulator kinematics object */
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;

  /** @brief Adjacency map for kinematics object mapping rigid links to moving links */
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;

  /** @brief Transform from world (root of scene) to base of the kinematics object */
  Eigen::Isometry3d world_to_base_;

  /** @brief The name of the link to track relative to target_*/
  std::string link_;

  /** @brief The tcp transform to apply to link_ location */
  Eigen::Isometry3d tcp_;

  /** @brief This is a map containing the transform from link_ to its adjacent moving link in the kinematics object */
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;

  /** @brief The name of the target link to track relative to link_ */
  std::string target_;

  /** @brief A tcp tranform to be applied to target_ location */
  Eigen::Isometry3d target_tcp_;

  /** @brief This is a map containing the transform from target_ to its adjacent moving link in the kinematics object */
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_target_;

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  DynamicCartPoseErrCalculator(
      std::string target,
      tesseract_kinematics::ForwardKinematics::ConstPtr manip,
      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
      const Eigen::Isometry3d& world_to_base,
      std::string link,
      const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity(),
      const Eigen::Isometry3d& target_tcp = Eigen::Isometry3d::Identity(),
      Eigen::VectorXi indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()))
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , tcp_(tcp)
    , target_(std::move(target))
    , target_tcp_(target_tcp)
    , indices_(std::move(indices))
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
    if (kin_link_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Link name '%s' provided does not exist.", link_.c_str());
      assert(false);
    }

    kin_target_ = adjacency_map_->getLinkMapping(target_);
    if (kin_target_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Link name '%s' provided does not exist.", target_.c_str());
      assert(false);
    }
  }

  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/** @brief Used to calculate the jacobian for CartPoseTermInfo */
struct DynamicCartPoseJacCalculator : sco::MatrixOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Manipulator kinematics object */
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;

  /** @brief Adjacency map for kinematics object mapping rigid links to moving links */
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;

  /** @brief Transform from world (root of scene) to base of the kinematics object */
  Eigen::Isometry3d world_to_base_;

  /** @brief The name of the link to track relative to target_*/
  std::string link_;

  /** @brief The tcp transform to apply to link_ location */
  Eigen::Isometry3d tcp_;

  /** @brief This is a map containing the transform from link_ to its adjacent moving link in the kinematics object */
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;

  /** @brief The name of the target link to track relative to link_ */
  std::string target_;

  /** @brief A tcp tranform to be applied to target_ location */
  Eigen::Isometry3d target_tcp_;

  /** @brief This is a map containing the transform from target_ to its adjacent moving link in the kinematics object */
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_target_;

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  DynamicCartPoseJacCalculator(
      std::string target,
      tesseract_kinematics::ForwardKinematics::ConstPtr manip,
      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
      const Eigen::Isometry3d& world_to_base,
      std::string link,
      const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity(),
      const Eigen::Isometry3d& target_tcp = Eigen::Isometry3d::Identity(),
      Eigen::VectorXi indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()))
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , tcp_(tcp)
    , target_(std::move(target))
    , target_tcp_(target_tcp)
    , indices_(std::move(indices))
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
    if (kin_link_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Link name '%s' provided does not exist.", link_.c_str());
      assert(false);
    }

    kin_target_ = adjacency_map_->getLinkMapping(target_);
    if (kin_target_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Link name '%s' provided does not exist.", target_.c_str());
      assert(false);
    }
    assert(indices_.size() <= 6);
  }

  Eigen::MatrixXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/**
 * @brief Used to calculate the error for StaticCartPoseTermInfo in target coordinate system
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

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  CartPoseErrCalculator(
      const Eigen::Isometry3d& pose,
      tesseract_kinematics::ForwardKinematics::ConstPtr manip,
      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
      const Eigen::Isometry3d& world_to_base,
      std::string link,
      const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity(),
      Eigen::VectorXi indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()))
    : pose_inv_(pose.inverse())
    , manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , tcp_(tcp)
    , indices_(std::move(indices))
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
    if (kin_link_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Link name '%s' provided does not exist.", link_.c_str());
      assert(false);
    }
    assert(indices_.size() <= 6);
  }

  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/** @brief Used to calculate the jacobian for StaticCartPoseTermInfo */
struct CartPoseJacCalculator : sco::MatrixOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d pose_inv_;
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  Eigen::Isometry3d tcp_;

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  CartPoseJacCalculator(
      const Eigen::Isometry3d& pose,
      tesseract_kinematics::ForwardKinematics::ConstPtr manip,
      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
      const Eigen::Isometry3d& world_to_base,
      std::string link,
      const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity(),
      Eigen::VectorXi indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()))
    : pose_inv_(pose.inverse())
    , manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , tcp_(tcp)
    , indices_(std::move(indices))
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
    if (kin_link_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Link name '%s' provided does not exist.", link_.c_str());
      assert(false);
    }
    assert(indices_.size() <= 6);
  }

  Eigen::MatrixXd operator()(const Eigen::VectorXd& dof_vals) const override;
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
                       const Eigen::Isometry3d& world_to_base,
                       std::string link,
                       double limit,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity())
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , limit_(limit)
    , tcp_(tcp)
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
                       const Eigen::Isometry3d& world_to_base,
                       std::string link,
                       double limit,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity())
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , limit_(limit)
    , tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

struct JointVelErrCalculator : sco::VectorOfVector
{
  /** @brief Velocity target */
  double target_{ 0.0 };
  /** @brief Upper tolerance */
  double upper_tol_{ 0.0 };
  /** @brief Lower tolerance */
  double lower_tol_{ 0.0 };
  JointVelErrCalculator() = default;
  JointVelErrCalculator(double target, double upper_tol, double lower_tol)
    : target_(target), upper_tol_(upper_tol), lower_tol_(lower_tol)
  {
  }
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointVelJacCalculator : sco::MatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointAccErrCalculator : sco::VectorOfVector
{
  JointVelErrCalculator vel_calc;
  double limit_{ 0.0 };
  JointAccErrCalculator() = default;
  JointAccErrCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointAccJacCalculator : sco::MatrixOfVector
{
  JointVelErrCalculator vel_calc;
  JointVelJacCalculator vel_jac_calc;
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointJerkErrCalculator : sco::VectorOfVector
{
  JointAccErrCalculator acc_calc;
  double limit_{ 0.0 };
  JointJerkErrCalculator() = default;
  JointJerkErrCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointJerkJacCalculator : sco::MatrixOfVector
{
  JointAccErrCalculator acc_calc;
  JointAccJacCalculator acc_jac_calc;
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct TimeCostCalculator : sco::VectorOfVector
{
  /** @brief The time target (s). This is subtracted from the cost, so only set limit!=0 if the penalty type is a hinge
   * (or you could get negatives)*/
  double limit_{ 0.0 };
  TimeCostCalculator() = default;
  TimeCostCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct TimeCostJacCalculator : sco::MatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

/** @brief Error calculator for evaluating the cost of a singularity, in the form 1.0 / (smallest_sv + lambda) */
struct AvoidSingularityErrCalculator : sco::VectorOfVector
{
  /** @brief Forward kinematics (and robot jacobian) calculator */
  tesseract_kinematics::ForwardKinematics::ConstPtr fwd_kin_;
  /** @brief The name of the robot link for which to calculate the robot jacobian (required because of kinematic trees)
   */
  std::string link_name_;
  /** @brief Damping factor to prevent the cost from becoming infinite when the smallest singular value is very close or
   * equal to zero */
  double lambda_;
  AvoidSingularityErrCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr fwd_kin,
                                std::string link_name,
                                double lambda = 1.0e-3)
    : fwd_kin_(std::move(fwd_kin)), link_name_(std::move(link_name)), lambda_(lambda)
  {
  }
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

/** @brief Jacobian calculator for the singularity avoidance error */
struct AvoidSingularityJacCalculator : sco::MatrixOfVector
{
  /** @brief Forward kinematics (and robot jacobian) calculator */
  tesseract_kinematics::ForwardKinematics::ConstPtr fwd_kin_;
  /** @brief The name of the robot link for which to calculate the robot jacobian (required because of kinematic trees)
   */
  std::string link_name_;
  /** @brief Damping factor to prevent the cost from becoming infinite when the smallest singular value is very close or
   * equal to zero */
  double lambda_;
  /** @brief Small number used to perturb each joint in the current state to calculate the partial derivative of the
   * robot jacobian */
  double eps_;
  AvoidSingularityJacCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr fwd_kin,
                                std::string link_name,
                                double lambda = 1.0e-3,
                                double eps = 1.0e-6)
    : fwd_kin_(std::move(fwd_kin)), link_name_(std::move(link_name)), lambda_(lambda), eps_(eps)
  {
  }
  /** @brief Helper function for numerically calculating the partial derivative of the jacobian */
  Eigen::MatrixXd jacobianPartialDerivative(const Eigen::VectorXd& state,
                                            const Eigen::MatrixXd& jacobian,
                                            Eigen::Index jntIdx) const;
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

/** @brief Error calculator for evaluating the cost of a singularity of a subset of the optimization problem joints.
 * The use case of this cost calculator would be to help a kinematic sub-chain avoid singularity (i.e. a robot in a
 * system with an integrated positioner) */
struct AvoidSingularitySubsetErrCalculator : AvoidSingularityErrCalculator
{
  /** @brief Forward kinematics (and robot jacobian) calculator for the optimization problem's full set of joints */
  tesseract_kinematics::ForwardKinematics::ConstPtr superset_kin_;
  AvoidSingularitySubsetErrCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr subset_kin,
                                      tesseract_kinematics::ForwardKinematics::ConstPtr superset_kin,
                                      std::string link_name,
                                      double lambda = 1.0e-3)
    : AvoidSingularityErrCalculator(std::move(subset_kin), std::move(link_name), lambda)
    , superset_kin_(std::move(superset_kin))
  {
  }
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

/** @brief Jacobian calculator for the subset singularity avoidance error */
struct AvoidSingularitySubsetJacCalculator : AvoidSingularityJacCalculator
{
  tesseract_kinematics::ForwardKinematics::ConstPtr superset_kin_;
  AvoidSingularitySubsetJacCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr subset_kin,
                                      tesseract_kinematics::ForwardKinematics::ConstPtr superset_kin,
                                      std::string link_name,
                                      double lambda = 1.0e-3,
                                      double eps = 1.0e-6)
    : AvoidSingularityJacCalculator(std::move(subset_kin), std::move(link_name), lambda, eps)
    , superset_kin_(std::move(superset_kin))
  {
  }
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

}  // namespace trajopt
