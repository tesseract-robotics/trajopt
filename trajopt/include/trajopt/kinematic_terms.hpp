#pragma once
#include <Eigen/Core>
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_fwd.hpp>

namespace trajopt
{
using namespace sco;
typedef BasicArray<Var> VarArray;

struct CartPoseErrCalculator : public VectorOfVector
{
  std::string target_;
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  Eigen::Affine3d tcp_;
  CartPoseErrCalculator(const std::string& target,
                        tesseract::BasicKinConstPtr manip,
                        tesseract::BasicEnvConstPtr env,
                        std::string link,
                        Eigen::Affine3d tcp = Eigen::Affine3d::Identity())
    : target_(target), manip_(manip), env_(env), link_(link), tcp_(tcp)
  {
  }

  VectorXd operator()(const VectorXd& dof_vals) const;
};

struct CartPoseErrorPlotter : public Plotter
{
  std::shared_ptr<void> m_calc;  // actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  CartPoseErrorPlotter(std::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x);
};

struct StaticCartPoseErrCalculator : public VectorOfVector
{
  Eigen::Affine3d pose_inv_;
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  Eigen::Affine3d tcp_;
  StaticCartPoseErrCalculator(const Eigen::Affine3d& pose,
                              tesseract::BasicKinConstPtr manip,
                              tesseract::BasicEnvConstPtr env,
                              std::string link,
                              Eigen::Affine3d tcp = Eigen::Affine3d::Identity())
    : pose_inv_(pose.inverse()), manip_(manip), env_(env), link_(link), tcp_(tcp)
  {
  }

  VectorXd operator()(const VectorXd& dof_vals) const;
};

struct StaticCartPoseErrorPlotter : public Plotter
{
  std::shared_ptr<void> m_calc;  // actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  StaticCartPoseErrorPlotter(std::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x);
};

struct CartVelJacCalculator : MatrixOfVector
{
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  double limit_;
  Eigen::Affine3d tcp_;
  CartVelJacCalculator(tesseract::BasicKinConstPtr manip,
                       tesseract::BasicEnvConstPtr env,
                       std::string link,
                       double limit,
                       Eigen::Affine3d tcp = Eigen::Affine3d::Identity())
    : manip_(manip), env_(env), link_(link), limit_(limit), tcp_(tcp)
  {
  }

  MatrixXd operator()(const VectorXd& dof_vals) const;
};

struct CartVelCalculator : VectorOfVector
{
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  double limit_;
  Eigen::Affine3d tcp_;
  CartVelCalculator(tesseract::BasicKinConstPtr manip,
                    tesseract::BasicEnvConstPtr env,
                    std::string link,
                    double limit,
                    Eigen::Affine3d tcp = Eigen::Affine3d::Identity())
    : manip_(manip), env_(env), link_(link), limit_(limit), tcp_(tcp)
  {
  }

  VectorXd operator()(const VectorXd& dof_vals) const;
};

/**
 * @brief The AlignedAxisErrCalculator is a struct whose operator() calculates error of a given
 * pose with respect to the rotation along the defined axis.
 */
struct AlignedAxisErrCalculator : public VectorOfVector
{
  Eigen::Matrix3d orientation_inv_;   /**< @brief param The inverse of the desired orientation */
  tesseract::BasicKinConstPtr manip_; /**< @brief Kinematics object */
  tesseract::BasicEnvConstPtr env_;   /**< @brief Environment object */
  std::string link_;                  /**< @brief Link of the robot referred to */
  Eigen::Matrix3d tcp_orientation_;   /**< @brief Tool center point orientation */

  Vector3d axis_; /**< @brief Axis of rotation to align with */
  double tol_;    /**< @brief Tolerance angle in radians */

  /**
   * @brief AlignedAxisErrCalculator Constructor
   * @param pose Desired pose
   * @param manip
   * @param env
   * @param link
   * @param axis Axis of rotation
   * @param tol_angle Tolerance in radians
   * @param tcp Tool center point
   */
  AlignedAxisErrCalculator(const Eigen::Matrix3d& orientation,
                           tesseract::BasicKinConstPtr manip,
                           tesseract::BasicEnvConstPtr env,
                           std::string link,
                           Vector3d axis,
                           double tol,
                           Eigen::Matrix3d tcp_orientation = Eigen::Matrix3d::Identity())
    : orientation_inv_(orientation.inverse())
    , manip_(manip)
    , env_(env)
    , link_(link)
    , tcp_orientation_(tcp_orientation)
    , axis_(axis)
    , tol_(tol)
  {
  }

  /**
   * @brief operator () Calculates error
   * @param dof_vals Values of the joints for forward kinematics
   * @return 1D vector of error beyond the allowed rotation
   */
  VectorXd operator()(const VectorXd& dof_vals) const;
};

/**
 * @brief The ConicalAxisErrCalculator is a struct whose operator() returns the error of the
 * given pose with respect to the conical constraint defined
 *
 */
struct ConicalAxisErrCalculator : public VectorOfVector
{
  Eigen::Matrix3d orientation_inv_;   /**< @brief Inverse of the desired orientation */
  tesseract::BasicKinConstPtr manip_; /**< @brief Kinematics object */
  tesseract::BasicEnvConstPtr env_;   /**< @brief Environment object */
  std::string link_;                  /**< @brief The link of the robot referred to */
  Eigen::Matrix3d tcp_orientation_;   /**< @brief Tool center point orientation */

  Vector3d axis_; /**< @brief Axis the conical tolerance is applied to */
  double tol_;    /**< @brief Tolerance angle in radians */

  /**
   * @brief ConicalAxisErrCalculator
   * @param pose Desired pose
   * @param manip Kinematics Object
   * @param env Environment object
   * @param link Link of the robot the term applies to
   * @param axis Axis to define the conical tolerance around
   * @param tol_angle Tolerance angle in degrees
   * @param tcp Tool center point
   */
  ConicalAxisErrCalculator(const Eigen::Matrix3d& orientation,
                           tesseract::BasicKinConstPtr manip,
                           tesseract::BasicEnvConstPtr env,
                           std::string link,
                           Vector3d axis,
                           double tol,
                           Eigen::Matrix3d tcp_orientation = Eigen::Matrix3d::Identity())
    : orientation_inv_(orientation.inverse())
    , manip_(manip)
    , env_(env)
    , link_(link)
    , tcp_orientation_(tcp_orientation)
    , axis_(axis)
    , tol_(tol)
  {
  }

  /**
   * @brief operator ()
   * @param dof_vals
   * @return
   */
  VectorXd operator()(const VectorXd& dof_vals) const;
};
}
