#pragma once

#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_fwd.hpp>
#include <trajopt/common.hpp>
#include <Eigen/Core>
#include <trajopt_scene/basic_kin.h>
#include <trajopt_scene/basic_env.h>

namespace trajopt {

using namespace sco;
typedef BasicArray<Var> VarArray;

struct CartPoseErrCalculator : public VectorOfVector
{
  Eigen::Affine3d pose_inv_;
  trajopt_scene::BasicKinConstPtr manip_;
  trajopt_scene::BasicEnvPtr env_;
  std::string link_;
  Eigen::Affine3d tcp_;
  CartPoseErrCalculator(const Eigen::Affine3d& pose, trajopt_scene::BasicKinConstPtr manip, trajopt_scene::BasicEnvPtr env, std::string link, Eigen::Affine3d tcp = Eigen::Affine3d::Identity()) :
    pose_inv_(pose.inverse()),
    manip_(manip),
    env_(env),
    link_(link),
    tcp_(tcp) {}

  VectorXd operator()(const VectorXd& dof_vals) const;
};

struct CartPoseErrorPlotter : public Plotter
{
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  CartPoseErrorPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x);
};


struct CartVelJacCalculator : MatrixOfVector
{
  trajopt_scene::BasicKinConstPtr manip_;
  trajopt_scene::BasicEnvPtr env_;
  std::string link_;
  double limit_;
  Eigen::Affine3d tcp_;
  CartVelJacCalculator(trajopt_scene::BasicKinConstPtr manip, trajopt_scene::BasicEnvPtr env, std::string link, double limit, Eigen::Affine3d tcp = Eigen::Affine3d::Identity()) :
    manip_(manip),
    env_(env),
    link_(link),
    limit_(limit),
    tcp_(tcp) {}

  MatrixXd operator()(const VectorXd& dof_vals) const;
};

struct CartVelCalculator : VectorOfVector
{
  trajopt_scene::BasicKinConstPtr manip_;
  trajopt_scene::BasicEnvPtr env_;
  std::string link_;
  double limit_;
  Eigen::Affine3d tcp_;
  CartVelCalculator(trajopt_scene::BasicKinConstPtr manip, trajopt_scene::BasicEnvPtr env, std::string link, double limit, Eigen::Affine3d tcp = Eigen::Affine3d::Identity()) :
    manip_(manip),
    env_(env),
    link_(link),
    limit_(limit),
    tcp_(tcp) {}

  VectorXd operator()(const VectorXd& dof_vals) const;
};

}
