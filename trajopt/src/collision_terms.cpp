#include <trajopt/collision_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_vec_ops.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt_utils/logging.hpp>
#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>

using namespace sco;
using namespace util;
using namespace std;

namespace trajopt {


void CollisionsToDistances(const std::vector<BasicEnv::DistanceResult> &dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());
  for (auto i = 0; i < dist_results.size(); ++i)
  {
    if (dist_results[i].valid)
    {
      dists.push_back(dist_results[i].distance);
    }
  }
}

void DebugPrintInfo(const BasicEnv::DistanceResult &res, bool header = false)
{
  if (header)
  {
    std::printf("\n");
    std::printf("DistanceResult| %30s | %30s | %6s | %6s, %6s, %6s | %6s, %6s, %6s | %6s, %6s, %6s | %7s | %6s, %6s, %6s | %6s, %6s, %6s |\n", "LINK A", "LINK B", "DIST", "Nx", "Ny", "Nz", "PAx", "PAy", "PAz", "PBx", "PBy", "PBz", "CC TIME", "CPAx", "CPAy", "CPAz", "CPBx", "CPBy", "CPBz");
  }
  std::printf("DistanceResult| %30s | %30s | %6.3f | %6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %7.3f | %6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f |\n", res.link_names[0].c_str(), res.link_names[1].c_str(), res.distance, res.normal(0), res.normal(1), res.normal(2), res.nearest_points[0](0), res.nearest_points[0](1), res.nearest_points[0](2), res.nearest_points[1](0), res.nearest_points[1](1), res.nearest_points[1](2), res.cc_time, res.cc_nearest_points[0](0), res.cc_nearest_points[0](1), res.cc_nearest_points[0](2), res.cc_nearest_points[1](0), res.cc_nearest_points[1](1), res.cc_nearest_points[1](2));
}

void CollisionsToDistanceExpressions(const std::vector<BasicEnv::DistanceResult> &dist_results,
                                     const BasicEnvPtr env, const BasicKinPtr manip,
                                     const VarVector& vars, const DblVec& x,
                                     vector<AffExpr>& exprs, bool isTimestep1)
{
  VectorXd dofvals = getVec(x, vars);
  std::vector<std::string> link_names;
  manip->getLinkNamesWithGeometry(link_names);

  // All collision data is in world corrdinate system. This provides the transfrom
  // for converting data between world frame and manipulator frame.
  Eigen::Affine3d change_base = env->getLinkTransform(manip->getBaseLinkName());

  exprs.clear();
  exprs.reserve(dist_results.size());
  for (auto i = 0; i < dist_results.size(); ++i)
  {
    const BasicEnv::DistanceResult &res = dist_results[i];
    if (!res.valid)
    {
      continue;
    }

    AffExpr dist(res.distance);

    DebugPrintInfo(res, i==0);

    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      MatrixXd jac;
      VectorXd dist_grad;
      manip->calcJacobian(jac, change_base, dofvals, res.link_names[0], res.nearest_points[0]);
      dist_grad = -res.normal.transpose() * jac.topRows(3);

      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(dofvals));
    }

    std::vector<std::string>::const_iterator itB = std::find(link_names.begin(), link_names.end(), res.link_names[1]);
    if (itB != link_names.end())
    {
      MatrixXd jac;
      VectorXd dist_grad;
      manip->calcJacobian(jac, change_base, dofvals, res.link_names[1], (isTimestep1 && (res.cc_type == collision_detection::CCType_Between)) ? res.cc_nearest_points[1] : res.nearest_points[1]);
      dist_grad = res.normal.transpose() * jac.topRows(3);
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(dofvals));
    }

    if (itA != link_names.end() || itB != link_names.end())
    {
      exprs.push_back(dist);
    }
  }
}

void CollisionsToDistanceExpressions(const std::vector<BasicEnv::DistanceResult> &dist_results,
                                     const BasicEnvPtr env, const BasicKinPtr manip,
                                     const VarVector& vars0, const VarVector& vars1, const DblVec& x,
                                     vector<AffExpr>& exprs)
{
  vector<AffExpr> exprs0, exprs1;
  CollisionsToDistanceExpressions(dist_results, env, manip, vars0, x, exprs0, false);
  CollisionsToDistanceExpressions(dist_results, env, manip, vars1, x, exprs1, true);

  exprs.resize(exprs0.size());
  for (int i=0; i < exprs0.size(); ++i)
  {
    exprScale(exprs0[i], (1-dist_results[i].cc_time));
    exprScale(exprs1[i], dist_results[i].cc_time);
    exprs[i] = AffExpr(0);
    exprInc(exprs[i], exprs0[i]);
    exprInc(exprs[i], exprs1[i]);
    cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x)
{
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, std::vector<BasicEnv::DistanceResult> &dist_results)
{
  double key = hash(getDblVec(x, GetVars()));
  std::vector<BasicEnv::DistanceResult>* it = m_cache.get(key);
  if (it != NULL)
  {
    LOG_DEBUG("using cached collision check\n");
    dist_results = *it;
  }
  else
  {
    LOG_DEBUG("not using cached collision check\n");
    CalcCollisions(x, dist_results);
    m_cache.put(key, dist_results);
  }
}

void CollisionEvaluator::Plot(const DblVec& x)
{
  std::vector<BasicEnv::DistanceResult> dist_results;
  GetCollisionsCached(x, dist_results);
  std::vector<std::string> link_names;
  manip_->getLinkNamesWithGeometry(link_names);
  env_->plotCollisions(link_names, dist_results, contact_distance_);
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(BasicKinPtr manip, BasicEnvPtr env, double contact_distance, const VarVector& vars) :
  CollisionEvaluator(manip, env, contact_distance), m_vars(vars) {}


void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, std::vector<BasicEnv::DistanceResult> &dist_results)
{
  BasicEnv::DistanceRequest req;

  req.joint_angles1 = getVec(x, m_vars);
  manip_->getLinkNamesWithGeometry(req.link_names);
  manip_->getJointNames(req.joint_names);
  req.contact_distance = contact_distance_ + 0.04; // The original implementation added a margin of 0.04

  env_->calcDistancesDiscrete(req, dist_results);
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  std::vector<BasicEnv::DistanceResult> dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs)
{
  std::vector<BasicEnv::DistanceResult> dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars, x, exprs, false);

  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(BasicKinPtr manip, BasicEnvPtr env, double contact_distance, const VarVector& vars0, const VarVector& vars1) :
  CollisionEvaluator(manip, env, contact_distance), m_vars0(vars0),m_vars1(vars1)
{}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, std::vector<BasicEnv::DistanceResult> &dist_results)
{
  BasicEnv::DistanceRequest req;
  manip_->getLinkNamesWithGeometry(req.link_names);
  manip_->getJointNames(req.joint_names);
  req.joint_angles1 = getVec(x, m_vars0);
  req.joint_angles2 = getVec(x, m_vars1);
  req.contact_distance = contact_distance_ + 0.04; // The original implementation added a margin of 0.04

  env_->calcDistancesContinuous(req, dist_results);
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  std::vector<BasicEnv::DistanceResult> dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars0, m_vars1, x, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  std::vector<BasicEnv::DistanceResult> dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}


//////////////////////////////////////////


CollisionCost::CollisionCost(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars) :
    Cost("collision"),
    m_calc(new SingleTimestepCollisionEvaluator(manip, env, dist_pen, vars)), m_coeff(coeff)
{}

CollisionCost::CollisionCost(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1) :
    Cost("cast_collision"),
    m_calc(new CastCollisionEvaluator(manip, env, dist_pen, vars0, vars1)), m_coeff(coeff)
{}

ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_calc->getContactDistance()), exprs[i]);
    out->addHinge(viol, m_coeff);
  }
  return out;
}

double CollisionCost::value(const vector<double>& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);
  double out = 0;
  for (int i=0; i < dists.size(); ++i) {
    out += pospart(m_calc->getContactDistance() - dists[i]) * m_coeff;
  }
  return out;
}

void CollisionCost::Plot(const DblVec& x)
{
  m_calc->Plot(x);
}

// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars) :
    m_calc(new SingleTimestepCollisionEvaluator(manip, env, dist_pen, vars)), m_coeff(coeff)
{
  name_="collision";
}

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1) :
  m_calc(new CastCollisionEvaluator(manip, env, dist_pen, vars0, vars1)), m_coeff(coeff)
{
  name_="collision";
  ROS_ERROR("CastCollisionEvaluator is not currently implemented within ros");
}

ConvexConstraintsPtr CollisionConstraint::convex(const vector<double>& x, Model* model) {
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_calc->getContactDistance()), exprs[i]);
    out->addIneqCnt(exprMult(viol,m_coeff));
  }
  return out;
}

DblVec CollisionConstraint::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  DblVec out(dists.size());
  for (int i=0; i < dists.size(); ++i) {
    out[i] = pospart(m_calc->getContactDistance() - dists[i]) * m_coeff;
  }
  return out;
}



}
