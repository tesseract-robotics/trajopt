#pragma once

/**
Simple quadratic costs on trajectory
*/

#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_utils/macros.h>

namespace trajopt
{
class TRAJOPT_API JointPosCost : public sco::Cost
{
public:
  JointPosCost(const sco::VarVector& vars, const Eigen::VectorXd& vals, const Eigen::VectorXd& coeffs);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  virtual double value(const DblVec &);

private:
  sco::VarVector vars_;
  Eigen::VectorXd vals_, coeffs_;
  sco::QuadExpr expr_;
};

class TRAJOPT_API JointVelCost : public sco::Cost
{
public:
  JointVelCost(const VarArray& traj, const Eigen::VectorXd& coeffs);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  virtual double value(const DblVec&);

private:
  VarArray vars_;
  Eigen::VectorXd coeffs_;
  sco::QuadExpr expr_;
};

class TRAJOPT_API JointAccCost : public sco::Cost
{
public:
  JointAccCost(const VarArray& traj, const Eigen::VectorXd& coeffs);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  virtual double value(const DblVec&);

private:
  VarArray vars_;
  Eigen::VectorXd coeffs_;
  sco::QuadExpr expr_;
};

class TRAJOPT_API JointJerkCost : public sco::Cost
{
public:
  JointJerkCost(const VarArray& traj, const Eigen::VectorXd& coeffs);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  virtual double value(const DblVec&);

private:
  VarArray vars_;
  Eigen::VectorXd coeffs_;
  sco::QuadExpr expr_;
};
}
