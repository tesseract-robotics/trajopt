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

enum Axis {
  X_AXIS,
  Y_AXIS,
  Z_AXIS
};

/**
 * @brief The ConfinedAxisErrCalculator is a truct whose operator() calculates error of a given
 * pose with respect ot the confined rotation along the defined axis.
 */
struct ConfinedAxisErrCalculator : public VectorOfVector {

  Eigen::Affine3d pose_inv_; /**< param The inverse of the desired pose */
  tesseract::BasicKinConstPtr manip_; /**< Kinematics object */
  tesseract::BasicEnvConstPtr env_; /**< Environment object */
  std::string link_; /**< Link of the robot referred to */
  Eigen::Affine3d tcp_; /**< Tool center point */

  Axis axis_; /**< Axis of rotation being provided with a tolerance */
  double tol_; /**< Tolerance angle in radians */

  /**
   * @brief ConfinedAxisErrCalculator Constructor
   * @param pose Desired pose
   * @param manip
   * @param env
   * @param link
   * @param axis Axis of rotation
   * @param tol_angle Tolerance in radians
   * @param tcp Tool center point
   */
  ConfinedAxisErrCalculator(const Eigen::Affine3d& pose, tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env,
                            std::string link, Axis axis, double tol_angle, Eigen::Affine3d tcp = Eigen::Affine3d::Identity()) :
    pose_inv_(pose.inverse()),
    manip_(manip),
    env_(env),
    link_(link),
    tcp_(tcp),
    axis_(axis),
    tol_(tol_angle * M_PI / 180.0)
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
struct ConicalAxisErrCalculator : public VectorOfVector {
  Eigen::Affine3d pose_inv_; /**< Inverse of the desired pose */
  tesseract::BasicKinConstPtr manip_; /**< Kinematics object */
  tesseract::BasicEnvConstPtr env_; /**< Environment object */
  std::string link_; /**< The link of the robot reggered to */
  Eigen::Affine3d tcp_; /**< Tool center point */

  Axis axis_; /**< Axis the conical tolreance is applied to */
  double tol_; /**< Tolerance angle in radians */

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
  ConicalAxisErrCalculator(const Eigen::Affine3d& pose, tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env,
                           std::string link, Axis axis, double tol_angle, Eigen::Affine3d tcp = Eigen::Affine3d::Identity()) :
    pose_inv_(pose.inverse()),
    manip_(manip),
    env_(env),
    link_(link),
    tcp_(tcp),
    axis_(axis),
    tol_(tol_angle * M_PI / 180.0)
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
