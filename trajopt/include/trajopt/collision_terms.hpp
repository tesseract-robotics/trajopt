#pragma once
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/sco_fwd.hpp>
#include <trajopt/cache.hxx>
#include <trajopt/basic_env.h>
#include <trajopt/basic_kin.h>

namespace trajopt
{


struct CollisionEvaluator
{
  CollisionEvaluator(BasicKinPtr manip, BasicEnvPtr env) : manip_(manip), env_(env) {}
  virtual ~CollisionEvaluator() {}
  virtual void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, std::vector<BasicEnv::DistanceResult> &dist_results) = 0;
  void GetCollisionsCached(const DblVec& x, std::vector<BasicEnv::DistanceResult> &);
  void Plot(const DblVec& x);
  virtual VarVector GetVars()=0;

  Cache<size_t, std::vector<BasicEnv::DistanceResult>, 10> m_cache;

protected:
  BasicEnvPtr env_;
  BasicKinPtr manip_;

private:
  CollisionEvaluator() {}

};

typedef boost::shared_ptr<CollisionEvaluator> CollisionEvaluatorPtr;

struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  SingleTimestepCollisionEvaluator(BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars);
  /**
  @brief linearize all contact distances in terms of robot dofs
  ;
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance function
  */
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& exprs);
  void CalcCollisions(const DblVec& x, std::vector<BasicEnv::DistanceResult> &dist_results);
  VarVector GetVars() {return m_vars;}

private:
  VarVector m_vars;
};

struct CastCollisionEvaluator : public CollisionEvaluator {
public:
  CastCollisionEvaluator(BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1);
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  void CalcDists(const DblVec& x, DblVec& exprs);
  void CalcCollisions(const DblVec& x);
  VarVector GetVars() {return concat(m_vars0, m_vars1);}
  
private:
  VarVector m_vars0;
  VarVector m_vars1;
};

class TRAJOPT_API CollisionCost : public Cost, public Plotter {
public:
  /* constructor for single timestep */
  CollisionCost(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars);
  /* constructor for cast cost */
  CollisionCost(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1);
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  virtual double value(const vector<double>&);
  void Plot(const DblVec& x);
  VarVector getVars() {return m_calc->GetVars();}
private:
  CollisionEvaluatorPtr m_calc;
  double m_dist_pen;
  double m_coeff;
};

class TRAJOPT_API CollisionConstraint : public IneqConstraint {
public:
  /* constructor for single timestep */
  CollisionConstraint(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars);
  /* constructor for cast cost */
  CollisionConstraint(double dist_pen, double coeff,BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1);
  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  virtual DblVec value(const vector<double>&);
  void Plot(const DblVec& x);
  VarVector getVars() {return m_calc->GetVars();}
private:
  CollisionEvaluatorPtr m_calc;
  double m_dist_pen;
  double m_coeff;
};

}
