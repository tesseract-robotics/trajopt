#pragma once
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/sco_fwd.hpp>
#include <trajopt/cache.hxx>
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>

namespace trajopt
{


struct CollisionEvaluator
{
  CollisionEvaluator(tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env, SafetyMarginDataConstPtr safety_margin_data) : manip_(manip), env_(env), safety_margin_data_(safety_margin_data) {}
  virtual ~CollisionEvaluator() {}
  virtual void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, tesseract::ContactResultVector &dist_results) = 0;
  void GetCollisionsCached(const DblVec& x, tesseract::ContactResultVector &);
  void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x);
  virtual VarVector GetVars()=0;

  const SafetyMarginDataConstPtr getSafetyMarginData() const { return safety_margin_data_; }

  Cache<size_t, tesseract::ContactResultVector, 10> m_cache;

protected:
  tesseract::BasicEnvConstPtr env_;
  tesseract::BasicKinConstPtr manip_;
  SafetyMarginDataConstPtr safety_margin_data_;

private:
  CollisionEvaluator() {}

};

typedef std::shared_ptr<CollisionEvaluator> CollisionEvaluatorPtr;

struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  SingleTimestepCollisionEvaluator(tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env, SafetyMarginDataConstPtr safety_margin_data, const VarVector& vars);
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
  void CalcCollisions(const DblVec& x, tesseract::ContactResultVector &dist_results);
  VarVector GetVars() {return m_vars;}

private:
  VarVector m_vars;
};

struct CastCollisionEvaluator : public CollisionEvaluator {
public:
  CastCollisionEvaluator(tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env, SafetyMarginDataConstPtr safety_margin_data, const VarVector& vars0, const VarVector& vars1);
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  void CalcDists(const DblVec& x, DblVec& exprs);
  void CalcCollisions(const DblVec& x, tesseract::ContactResultVector &dist_results);
  VarVector GetVars() {return concat(m_vars0, m_vars1);}
  
private:
  VarVector m_vars0;
  VarVector m_vars1;
};

class TRAJOPT_API CollisionCost : public Cost, public Plotter {
public:
  /* constructor for single timestep */
  CollisionCost(tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env, SafetyMarginDataConstPtr safety_margin_data, const VarVector& vars);
  /* constructor for cast cost */
  CollisionCost(tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env, SafetyMarginDataConstPtr safety_margin_data, const VarVector& vars0, const VarVector& vars1);
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  virtual double value(const vector<double>&);
  void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x);
  VarVector getVars() {return m_calc->GetVars();}
private:
  CollisionEvaluatorPtr m_calc;
};

class TRAJOPT_API CollisionConstraint : public IneqConstraint {
public:
  /* constructor for single timestep */
  CollisionConstraint(tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env, SafetyMarginDataConstPtr safety_margin_data, const VarVector& vars);
  /* constructor for cast cost */
  CollisionConstraint(tesseract::BasicKinConstPtr manip, tesseract::BasicEnvConstPtr env, SafetyMarginDataConstPtr safety_margin_data, const VarVector& vars0, const VarVector& vars1);
  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  virtual DblVec value(const vector<double>&);
  void Plot(const DblVec& x);
  VarVector getVars() {return m_calc->GetVars();}
private:
  CollisionEvaluatorPtr m_calc;
};

}
