#pragma once

/**
Simple quadratic costs on trajectory
*/

#include <trajopt_utils/macros.h>
#include <trajopt_sco/modeling.hpp>
#include <trajopt/common.hpp>

namespace trajopt {

class TRAJOPT_API JointPosCost : public Cost {
public:
  JointPosCost(const VarVector& vars, const VectorXd& vals, const VectorXd& coeffs);
  virtual ConvexObjectivePtr convex(const vector<double>& /*x*/, Model* model);
  virtual double value(const vector<double>&);
private:
  VarVector vars_;
  VectorXd vals_, coeffs_;
  QuadExpr expr_;
};

class TRAJOPT_API JointVelCost : public Cost {
public:
  JointVelCost(const VarArray& traj, const VectorXd& coeffs);
  virtual ConvexObjectivePtr convex(const vector<double>& /*x*/, Model* model);
  virtual double value(const vector<double>&);
private:
  VarArray vars_;
  VectorXd coeffs_;
  QuadExpr expr_;
};

class TRAJOPT_API JointAccCost : public Cost {
public:
  JointAccCost(const VarArray& traj, const VectorXd& coeffs);
  virtual ConvexObjectivePtr convex(const vector<double>& /*x*/, Model* model);
  virtual double value(const vector<double>&);
private:
  VarArray vars_;
  VectorXd coeffs_;
  QuadExpr expr_;
};

class TRAJOPT_API JointJerkCost : public Cost {
public:
  JointJerkCost(const VarArray& traj, const VectorXd& coeffs);
  virtual ConvexObjectivePtr convex(const vector<double>& /*x*/, Model* model);
  virtual double value(const vector<double>&);
private:
  VarArray vars_;
  VectorXd coeffs_;
  QuadExpr expr_;
};

}

