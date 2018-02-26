#pragma once

#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_fwd.hpp>
#include <trajopt/common.hpp>
#include <Eigen/Core>
#include <trajopt/basic_kin.h>
#include <trajopt/basic_env.h>

namespace trajopt {

using namespace sco;
typedef BasicArray<Var> VarArray;

struct CartPoseErrCalculator : public VectorOfVector {
  Eigen::Affine3d pose_inv_;
  BasicKinPtr manip_;
  BasicEnvPtr env_;
  std::string link_;
  CartPoseErrCalculator(const Eigen::Affine3d& pose, BasicKinPtr manip, BasicEnvPtr env, std::string link) :
    pose_inv_(pose.inverse()),
    manip_(manip),
    env_(env),
    link_(link) {}

  VectorXd operator()(const VectorXd& dof_vals) const;
};

struct CartPoseErrorPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  CartPoseErrorPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x);
};


struct CartVelJacCalculator : MatrixOfVector {
  BasicKinPtr manip_;
  BasicEnvPtr env_;
  std::string link_;
  double limit_;
  CartVelJacCalculator(BasicKinPtr manip, BasicEnvPtr env, std::string link, double limit) :
    manip_(manip),
    env_(env),
    link_(link),
    limit_(limit) {}

  MatrixXd operator()(const VectorXd& dof_vals) const;
};

struct CartVelCalculator : VectorOfVector {
  BasicKinPtr manip_;
  BasicEnvPtr env_;
  std::string link_;
  double limit_;
  CartVelCalculator(BasicKinPtr manip, BasicEnvPtr env, std::string link, double limit) :
    manip_(manip),
    env_(env),
    link_(link),
    limit_(limit) {}

  VectorXd operator()(const VectorXd& dof_vals) const;
};

}
