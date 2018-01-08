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


//void CollisionsToDistances(const vector<Collision>& collisions, const Link2Int& m_link2ind,
//    DblVec& dists) {
//  // Note: this checking (that the links are in the list we care about) is probably unnecessary
//  // since we're using LinksVsAll
//  dists.clear();
//  dists.reserve(collisions.size());
//  BOOST_FOREACH(const Collision& col, collisions) {
//    Link2Int::const_iterator itA = m_link2ind.find(col.linkA);
//    Link2Int::const_iterator itB = m_link2ind.find(col.linkB);
//    if (itA != m_link2ind.end() || itB != m_link2ind.end()) {
//      dists.push_back(col.distance);
//    }
//  }
//}

//void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad,
//    const Link2Int& link2ind, const VarVector& vars, const DblVec& dofvals, vector<AffExpr>& exprs, bool isTimestep1) {

//  exprs.clear();
//  exprs.reserve(collisions.size());
//  rad.SetDOFValues(dofvals); // since we'll be calculating jacobians
//  BOOST_FOREACH(const Collision& col, collisions) {
//    AffExpr dist(col.distance);
//    Link2Int::const_iterator itA = link2ind.find(col.linkA);
//    if (itA != link2ind.end()) {
//      VectorXd dist_grad = toVector3d(col.normalB2A).transpose()*rad.PositionJacobian(itA->second, col.ptA);
//      exprInc(dist, varDot(dist_grad, vars));
//      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
//    }
//    Link2Int::const_iterator itB = link2ind.find(col.linkB);
//    if (itB != link2ind.end()) {
//      VectorXd dist_grad = -toVector3d(col.normalB2A).transpose()*rad.PositionJacobian(itB->second, (isTimestep1 && (col.cctype == CCType_Between)) ? col.ptB1 : col.ptB);
//      exprInc(dist, varDot(dist_grad, vars));
//      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
//    }
//    if (itA != link2ind.end() || itB != link2ind.end()) {
//      exprs.push_back(dist);
//    }
//  }
//  LOG_DEBUG("%ld distance expressions\n", exprs.size());
//}

//void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad, const Link2Int& link2ind,
//    const VarVector& vars0, const VarVector& vars1, const DblVec& vals0, const DblVec& vals1,
//    vector<AffExpr>& exprs) {
//  vector<AffExpr> exprs0, exprs1;
//  CollisionsToDistanceExpressions(collisions, rad, link2ind, vars0, vals0, exprs0, false);
//  CollisionsToDistanceExpressions(collisions, rad, link2ind, vars1, vals1, exprs1,true);

//  exprs.resize(exprs0.size());
//  for (int i=0; i < exprs0.size(); ++i) {
//    exprScale(exprs0[i], (1-collisions[i].time));
//    exprScale(exprs1[i], collisions[i].time);
//    exprs[i] = AffExpr(0);
//    exprInc(exprs[i], exprs0[i]);
//    exprInc(exprs[i], exprs1[i]);
//    cleanupAff(exprs[i]);
//  }
//}

inline size_t hash(const DblVec& x) {
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, std::vector<BasicEnv::DistanceResult> &dist_results) {
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
  env_->plotCollisions(link_names, dist_results);
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars) :
  CollisionEvaluator(manip, env), m_vars(vars) {}


void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, std::vector<BasicEnv::DistanceResult> &dist_results)
{
  VectorXd dofvals = getVec(x, m_vars);
  std::vector<std::string> link_names, joint_names;
  manip_->getLinkNamesWithGeometry(link_names);
  manip_->getJointNames(joint_names);

  env_->calcDistances(joint_names, dofvals, link_names, dist_results);
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  std::vector<BasicEnv::DistanceResult> dist_results;
  GetCollisionsCached(x, dist_results);

  dists.reserve(dist_results.size());
  for (auto i = 0; i < dist_results.size(); ++i)
  {
    if (dist_results[i].valid)
    {
      dists.push_back(dist_results[i].distance);
    }
  }
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs)
{
  VectorXd dofvals = getVec(x, m_vars);
  std::vector<std::string> link_names;
  manip_->getLinkNamesWithGeometry(link_names);

  std::vector<BasicEnv::DistanceResult> dist_results;
  GetCollisionsCached(x, dist_results);
  // All collision data is in world corrdinate system. This provides the transfrom
  // for converting data between world frame and manipulator frame.
  Eigen::Affine3d change_base = env_->getLinkTransform(manip_->getBaseLinkName()).inverse();

  exprs.clear();
  exprs.reserve(dist_results.size());
  for (auto i = 0; i < dist_results.size(); ++i)
  {
    BasicEnv::DistanceResult &res = dist_results[i];
    if (!res.valid)
    {
      continue;
    }

    AffExpr dist(res.distance);

    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      MatrixXd jac;
      VectorXd dist_grad;
      manip_->calcJacobian(jac, change_base, dofvals, res.link_names[0], res.nearest_points[0]);
      dist_grad = -res.normal.transpose() * jac.topRows(3);

      exprInc(dist, varDot(dist_grad, m_vars));
      exprInc(dist, -dist_grad.dot(dofvals));
    }

    std::vector<std::string>::const_iterator itB = std::find(link_names.begin(), link_names.end(), res.link_names[1]);
    if (itB != link_names.end())
    {
      MatrixXd jac;
      VectorXd dist_grad;
      manip_->calcJacobian(jac, change_base, dofvals, res.link_names[1], res.nearest_points[1]);
      dist_grad = res.normal.transpose() * jac.topRows(3);
      exprInc(dist, varDot(dist_grad, m_vars));
      exprInc(dist, -dist_grad.dot(dofvals));
    }

    if (itA != link_names.end() || itB != link_names.end())
    {
      exprs.push_back(dist);
    }
  }

 LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

////////////////////////////////////////

//CastCollisionEvaluator::CastCollisionEvaluator(BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1) :
//  CollisionEvaluator(manip, env), m_vars0(vars0),m_vars1(vars1)
//{
//  vector<KinBody::LinkPtr> links;
//  vector<int> inds;
//  rad->GetAffectedLinks(m_links, true, inds);
//  for (int i=0; i < m_links.size(); ++i) {
//    m_link2ind[m_links[i].get()] = inds[i];
//  }
//}

//void CastCollisionEvaluator::CalcCollisions(const DblVec& x) {
//  DblVec dofvals0 = getDblVec(x, m_vars0);
//  DblVec dofvals1 = getDblVec(x, m_vars1);
//  m_rad->SetDOFValues(dofvals0);
//  m_cc->CastVsAll(*m_rad, m_links, dofvals0, dofvals1, collisions);
//}
//void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
//  vector<Collision> collisions;
//  GetCollisionsCached(x, collisions);
//  DblVec dofvals0 = getDblVec(x, m_vars0);
//  DblVec dofvals1 = getDblVec(x, m_vars1);
//  CollisionsToDistanceExpressions(collisions, *m_rad, m_link2ind, m_vars0, m_vars1, dofvals0, dofvals1, exprs);
//}
//void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
//  vector<Collision> collisions;
//  GetCollisionsCached(x, collisions);
//  CollisionsToDistances(collisions, m_link2ind, dists);
//}


//////////////////////////////////////////


CollisionCost::CollisionCost(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars) :
    Cost("collision"),
    m_calc(new SingleTimestepCollisionEvaluator(manip, env, vars)), m_dist_pen(dist_pen), m_coeff(coeff)
{}

CollisionCost::CollisionCost(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1) :
    Cost("cast_collision") //, m_calc(new CastCollisionEvaluator(manip, env, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff)
{
  ROS_ERROR("CastCollisionEvaluator is not currently implemented within ros");
}

ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
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
    out += pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}

void CollisionCost::Plot(const DblVec& x)
{
  m_calc->Plot(x);
}

// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars) :
    m_calc(new SingleTimestepCollisionEvaluator(manip, env, vars)), m_dist_pen(dist_pen), m_coeff(coeff)
{
  name_="collision";
}

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, BasicKinPtr manip, BasicEnvPtr env, const VarVector& vars0, const VarVector& vars1) //: m_calc(new CastCollisionEvaluator(manip, env, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff)
{
  name_="collision";
  ROS_ERROR("CastCollisionEvaluator is not currently implemented within ros");
}

ConvexConstraintsPtr CollisionConstraint::convex(const vector<double>& x, Model* model) {
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addIneqCnt(exprMult(viol,m_coeff));
  }
  return out;
}

DblVec CollisionConstraint::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  DblVec out(dists.size());
  for (int i=0; i < dists.size(); ++i) {
    out[i] = pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}



}
