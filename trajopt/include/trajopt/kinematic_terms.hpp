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

#if 0
void makeTrajVariablesAndBounds(int n_steps, const RobotAndDOF& manip, OptProb& prob_out, VarArray& vars_out);

class FKFunc {
public:
  virtual OpenRAVE::Transform operator()(const VectorXd& x) const = 0;
  virtual ~FKFunc() {}
};

class FKPositionJacobian {
public:
  virtual Eigen::MatrixXd operator()(const VectorXd& x) const = 0;
  virtual ~FKPositionJacobian() {}
};
#endif


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

#if 0
class CartPoseCost : public CostFromErrFunc {
public:
  CartPoseCost(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs);
};

class CartPoseConstraint : public ConstraintFromFunc {
public:
  CartPoseConstraint(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs);
};

class CartVelConstraint : public ConstraintFromFunc {
public:
  CartVelConstraint(const VarVector& step0vars, const VarVector& step1vars, RobotAndDOFPtr manip, KinBody::LinkPtr link, double distlimit);
};
#endif



}
