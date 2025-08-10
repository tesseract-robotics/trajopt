#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <functional>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_visualization/fwd.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/typedefs.hpp>

namespace trajopt
{
const double DEFAULT_EPSILON = 1e-5;

using ErrorFunctionType = std::function<Eigen::VectorXd(const Eigen::Isometry3d&, const Eigen::Isometry3d&)>;
using ErrorDiffFunctionType = std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                                            const Eigen::Isometry3d&,
                                                            const Eigen::Isometry3d&,
                                                            tesseract_common::TransformMap&)>;

/**
 * @brief Used to calculate the error for CartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct DynamicCartPoseErrCalculator : public TrajOptVectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Manipulator kinematics object */
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;

  /** @brief The name of the link to track relative to target_frame_*/
  std::string source_frame_;

  /** @brief The name of the target frame to track relative to source_frame_ */
  std::string target_frame_;

  /** @brief The offset transform to apply to source_frame_ location */
  Eigen::Isometry3d source_frame_offset_;

  /** @brief A offset transform to be applied to target_frame_ location */
  Eigen::Isometry3d target_frame_offset_;

  /** @brief Error function for calculating the error in the position given the source and target positions
   * this defaults to tesseract_common::calcTransformError if unset*/
  ErrorFunctionType error_function{ nullptr };

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  DynamicCartPoseErrCalculator(
      std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
      std::string source_frame,
      std::string target_frame,
      const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::VectorXi& indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()),
      const Eigen::VectorXd& lower_tolerance = {},
      const Eigen::VectorXd& upper_tolerance = {});

  void Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter,
            const Eigen::VectorXd& dof_vals) override;

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/** @brief Used to calculate the jacobian for CartPoseTermInfo */
struct DynamicCartPoseJacCalculator : TrajOptMatrixOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Manipulator kinematics object */
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;

  /** @brief The name of the link to track relative to target_frame_*/
  std::string source_frame_;

  /** @brief The offset transform to apply to source_frame_ location */
  Eigen::Isometry3d source_frame_offset_;

  /** @brief The name of the target frame to track relative to source_frame_ */
  std::string target_frame_;

  /** @brief A offset transform to be applied to target_frame_ location */
  Eigen::Isometry3d target_frame_offset_;

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  /** @brief perturbation amount for calculating Jacobian */
  double epsilon_;

  DynamicCartPoseJacCalculator(
      std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
      std::string source_frame,
      std::string target_frame,
      const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::VectorXi& indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()));

  Eigen::MatrixXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/**
 * @brief Used to calculate the error for StaticCartPoseTermInfo in target coordinate system
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartPoseErrCalculator : public TrajOptVectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;

  /** @brief The name of the link to track relative to target_frame_*/
  std::string source_frame_;

  /** @brief The offset transform to apply to source_frame_ location */
  Eigen::Isometry3d source_frame_offset_;

  /** @brief The name of the target link to track relative to source_frame_ */
  std::string target_frame_;

  /** @brief A offset transform to be applied to target_frame_ location */
  Eigen::Isometry3d target_frame_offset_;

  /** @brief indicates which link is active */
  bool is_target_active_{ true };

  /** @brief Error function for calculating the error in the position given the source and target positions
   * this defaults to tesseract_common::calcTransformError if unset*/
  ErrorFunctionType error_function_{ nullptr };

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  CartPoseErrCalculator(
      std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
      std::string source_frame,
      std::string target_frame,
      const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::VectorXi& indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()),
      const Eigen::VectorXd& lower_tolerance = {},
      const Eigen::VectorXd& upper_tolerance = {});

  void Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter,
            const Eigen::VectorXd& dof_vals) override;

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/** @brief Used to calculate the jacobian for StaticCartPoseTermInfo */
struct CartPoseJacCalculator : TrajOptMatrixOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;

  /** @brief The name of the link to track relative to target_frame_*/
  std::string source_frame_;

  /** @brief The offset transform to apply to source_frame_ location */
  Eigen::Isometry3d source_frame_offset_;

  /** @brief The name of the target link to track relative to source_frame_ */
  std::string target_frame_;

  /** @brief A offset transform to be applied to target_frame_ location */
  Eigen::Isometry3d target_frame_offset_;

  /** @brief indicates which link is active */
  bool is_target_active_{ true };

  /** @brief The error function to calculate the error difference used for jacobian calculations */
  ErrorDiffFunctionType error_diff_function_;

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  /** @brief perturbation amount for calculating Jacobian */
  double epsilon_;

  CartPoseJacCalculator(
      std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
      std::string source_frame,
      std::string target_frame,
      const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::VectorXi& indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()));

  Eigen::MatrixXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/**
 * @brief Used to calculate the jacobian for CartVelTermInfo
 *
 */
struct CartVelJacCalculator : TrajOptMatrixOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  double limit_;
  std::string link_;
  Eigen::Isometry3d tcp_;
  CartVelJacCalculator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                       std::string link,
                       double limit,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity());

  Eigen::MatrixXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

/**
 * @brief  Used to calculate the error for CartVelTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartVelErrCalculator : TrajOptVectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  std::string link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartVelErrCalculator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                       std::string link,
                       double limit,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity());

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

struct JointVelErrCalculator : TrajOptVectorOfVector
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

struct JointVelJacCalculator : TrajOptMatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointAccErrCalculator : TrajOptVectorOfVector
{
  JointVelErrCalculator vel_calc;
  double limit_{ 0.0 };
  JointAccErrCalculator() = default;
  JointAccErrCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointAccJacCalculator : TrajOptMatrixOfVector
{
  JointVelErrCalculator vel_calc;
  JointVelJacCalculator vel_jac_calc;
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointJerkErrCalculator : TrajOptVectorOfVector
{
  JointAccErrCalculator acc_calc;
  double limit_{ 0.0 };
  JointJerkErrCalculator() = default;
  JointJerkErrCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointJerkJacCalculator : TrajOptMatrixOfVector
{
  JointAccErrCalculator acc_calc;
  JointAccJacCalculator acc_jac_calc;
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct TimeCostCalculator : TrajOptVectorOfVector
{
  /** @brief The time target (s). This is subtracted from the cost, so only set limit!=0 if the penalty type is a hinge
   * (or you could get negatives)*/
  double limit_{ 0.0 };
  TimeCostCalculator() = default;
  TimeCostCalculator(double limit) : limit_(limit) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct TimeCostJacCalculator : TrajOptMatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

/** @brief Error calculator for evaluating the cost of a singularity, in the form 1.0 / (smallest_sv + lambda) */
struct AvoidSingularityErrCalculator : TrajOptVectorOfVector
{
  /** @brief Forward kinematics (and robot jacobian) calculator */
  std::shared_ptr<const tesseract_kinematics::JointGroup> fwd_kin_;
  /** @brief The name of the robot link for which to calculate the robot jacobian (required because of kinematic trees)
   */
  std::string link_name_;
  /** @brief Damping factor to prevent the cost from becoming infinite when the smallest singular value is very close or
   * equal to zero */
  double lambda_;
  AvoidSingularityErrCalculator(std::shared_ptr<const tesseract_kinematics::JointGroup> fwd_kin,
                                std::string link_name,
                                double lambda = 1.0e-3)
    : fwd_kin_(std::move(fwd_kin)), link_name_(std::move(link_name)), lambda_(lambda)
  {
  }
  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

/** @brief Jacobian calculator for the singularity avoidance error */
struct AvoidSingularityJacCalculator : TrajOptMatrixOfVector
{
  /** @brief Forward kinematics (and robot jacobian) calculator */
  std::shared_ptr<const tesseract_kinematics::JointGroup> fwd_kin_;
  /** @brief The name of the robot link for which to calculate the robot jacobian (required because of kinematic trees)
   */
  std::string link_name_;
  /** @brief Damping factor to prevent the cost from becoming infinite when the smallest singular value is very close or
   * equal to zero */
  double lambda_;
  /** @brief Small number used to perturb each joint in the current state to calculate the partial derivative of the
   * robot jacobian */
  double eps_;
  AvoidSingularityJacCalculator(std::shared_ptr<const tesseract_kinematics::JointGroup> fwd_kin,
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
  std::shared_ptr<const tesseract_kinematics::JointGroup> superset_kin_;
  AvoidSingularitySubsetErrCalculator(std::shared_ptr<const tesseract_kinematics::JointGroup> subset_kin,
                                      std::shared_ptr<const tesseract_kinematics::JointGroup> superset_kin,
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
  std::shared_ptr<const tesseract_kinematics::JointGroup> superset_kin_;
  AvoidSingularitySubsetJacCalculator(std::shared_ptr<const tesseract_kinematics::JointGroup> subset_kin,
                                      std::shared_ptr<const tesseract_kinematics::JointGroup> superset_kin,
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
