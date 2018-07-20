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

struct JointVelCalculator : VectorOfVector
{
  double limit_;
  JointVelCalculator() : limit_(0.0) {}
  JointVelCalculator(double limit) : limit_(limit) {}
  VectorXd operator()(const VectorXd& var_vals) const;
};

struct JointVelJacCalculator : MatrixOfVector
{
  MatrixXd operator()(const VectorXd& var_vals) const;
};

struct JointAccCalculator : VectorOfVector
{
  JointVelCalculator vel_calc;
  double limit_;
  JointAccCalculator() : limit_(0.0) {}
  JointAccCalculator(double limit) : limit_(limit) {}
  VectorXd operator()(const VectorXd& var_vals) const;
};

struct JointAccJacCalculator : MatrixOfVector
{
  JointVelCalculator vel_calc;
  JointVelJacCalculator vel_jac_calc;
  MatrixXd operator()(const VectorXd& var_vals) const;
};

struct JointJerkCalculator : VectorOfVector
{
  JointAccCalculator acc_calc;
  double limit_;
  JointJerkCalculator() : limit_(0.0) {}
  JointJerkCalculator(double limit) : limit_(limit) {}
  VectorXd operator()(const VectorXd& var_vals) const;
};

struct JointJerkJacCalculator : MatrixOfVector
{
  JointAccCalculator acc_calc;
  JointAccJacCalculator acc_jac_calc;
  MatrixXd operator()(const VectorXd& var_vals) const;
};

struct TimeCostCalculator : VectorOfVector
{
  double limit_;
  TimeCostCalculator(double limit) : limit_(limit) {}
  VectorXd operator()(const VectorXd& var_vals) const;
};

struct TimeCostJacCalculator : MatrixOfVector
{
  MatrixXd operator()(const VectorXd& var_vals) const;
};
}
