#include <boost/functional/hash.hpp>
#include <trajopt/collision_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/expr_vec_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace sco;
using namespace util;
using namespace std;

namespace trajopt
{
void CollisionsToDistances(const tesseract::ContactResultVector& dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
    dists.push_back(dist_results[i].distance);
}

void DebugPrintInfo(const tesseract::ContactResult& res, bool header = false)
{
  if (header)
  {
    std::printf("\n");
    std::printf("DistanceResult| %30s | %30s | %6s | %6s, %6s, %6s | %6s, %6s, "
                "%6s | %6s, %6s, %6s | %7s | %6s, %6s, %6s | %6s, %6s, %6s |\n",
                "LINK A",
                "LINK B",
                "DIST",
                "Nx",
                "Ny",
                "Nz",
                "PAx",
                "PAy",
                "PAz",
                "PBx",
                "PBy",
                "PBz",
                "CC TIME",
                "CPAx",
                "CPAy",
                "CPAz",
                "CPBx",
                "CPBy",
                "CPBz");
  }
  std::printf("DistanceResult| %30s | %30s | %6.3f | %6.3f, %6.3f, %6.3f | "
              "%6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %7.3f | %6.3f, "
              "%6.3f, %6.3f | %6.3f, %6.3f, %6.3f |\n",
              res.link_names[0].c_str(),
              res.link_names[1].c_str(),
              res.distance,
              res.normal(0),
              res.normal(1),
              res.normal(2),
              res.nearest_points[0](0),
              res.nearest_points[0](1),
              res.nearest_points[0](2),
              res.nearest_points[1](0),
              res.nearest_points[1](1),
              res.nearest_points[1](2),
              res.cc_time,
              res.cc_nearest_points[0](0),
              res.cc_nearest_points[0](1),
              res.cc_nearest_points[0](2),
              res.cc_nearest_points[1](0),
              res.cc_nearest_points[1](1),
              res.cc_nearest_points[1](2));
}

void CollisionsToDistanceExpressions(const tesseract::ContactResultVector& dist_results,
                                     const tesseract::BasicEnvConstPtr env,
                                     const tesseract::BasicKinConstPtr manip,
                                     const VarVector& vars,
                                     const DblVec& x,
                                     vector<AffExpr>& exprs,
                                     bool isTimestep1)
{
  VectorXd dofvals = getVec(x, vars);
  const std::vector<std::string>& link_names = manip->getLinkNames();

  // All collision data is in world corrdinate system. This provides the
  // transfrom for converting data between world frame and manipulator
  // frame.
  tesseract::EnvStateConstPtr state = env->getState();
  Eigen::Isometry3d change_base = state->transforms.at(manip->getBaseLinkName());
  assert(change_base.isApprox(env->getState(manip->getJointNames(), dofvals)->transforms.at(manip->getBaseLinkName())));

  exprs.clear();
  exprs.reserve(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    const tesseract::ContactResult& res = dist_results[i];

    AffExpr dist(res.distance);

    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      MatrixXd jac;
      VectorXd dist_grad;
      jac.resize(6, manip->numJoints());
      manip->calcJacobian(jac, change_base, dofvals, res.link_names[0], *state, res.nearest_points[0]);
      dist_grad = -res.normal.transpose() * jac.topRows(3);

      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(dofvals));
    }

    std::vector<std::string>::const_iterator itB = std::find(link_names.begin(), link_names.end(), res.link_names[1]);
    if (itB != link_names.end())
    {
      MatrixXd jac;
      VectorXd dist_grad;
      jac.resize(6, manip->numJoints());
      manip->calcJacobian(jac,
                          change_base,
                          dofvals,
                          res.link_names[1],
                          *state,
                          (isTimestep1 && (res.cc_type == tesseract::ContinouseCollisionType::CCType_Between)) ?
                              res.cc_nearest_points[1] :
                              res.nearest_points[1]);
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

void CollisionsToDistanceExpressions(const tesseract::ContactResultVector& dist_results,
                                     const tesseract::BasicEnvConstPtr env,
                                     const tesseract::BasicKinConstPtr manip,
                                     const VarVector& vars0,
                                     const VarVector& vars1,
                                     const DblVec& x,
                                     vector<AffExpr>& exprs)
{
  vector<AffExpr> exprs0, exprs1;
  CollisionsToDistanceExpressions(dist_results, env, manip, vars0, x, exprs0, false);
  CollisionsToDistanceExpressions(dist_results, env, manip, vars1, x, exprs1, true);

  exprs.resize(exprs0.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    exprScale(exprs0[i], (1 - dist_results[i].cc_time));
    exprScale(exprs1[i], dist_results[i].cc_time);
    exprs[i] = AffExpr(0);
    exprInc(exprs[i], exprs0[i]);
    exprInc(exprs[i], exprs1[i]);
    cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x) { return boost::hash_range(x.begin(), x.end()); }
void CollisionEvaluator::GetCollisionsCached(const DblVec& x, tesseract::ContactResultVector& dist_results)
{
  size_t key = hash(getDblVec(x, GetVars()));
  tesseract::ContactResultVector* it = m_cache.get(key);
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

void CollisionEvaluator::Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  const std::vector<std::string>& link_names = manip_->getLinkNames();

  Eigen::VectorXd safety_distance(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    const Eigen::Vector2d& data =
        getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0], dist_results[i].link_names[1]);
    safety_distance[i] = data[0];
  }

  plotter->plotContactResults(link_names, dist_results, safety_distance);
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(tesseract::BasicKinConstPtr manip,
                                                                   tesseract::BasicEnvConstPtr env,
                                                                   SafetyMarginDataConstPtr safety_margin_data,
                                                                   const VarVector& vars)
  : CollisionEvaluator(manip, env, safety_margin_data), m_vars(vars)
{
  tesseract::ContactRequest req;

  req.link_names = manip_->getLinkNames();
  req.contact_distance =
      safety_margin_data_->getMaxSafetyMargin() + 0.04;  // The original implementation added a margin of 0.04
  req.isContactAllowed = env_->getIsContactAllowedFn();
  req.type = tesseract::ContactRequestTypes::ALL;

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setContactRequest(req);
}

void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results)
{
  tesseract::ContactResultMap contacts;
  tesseract::EnvStatePtr state = env_->getState(manip_->getJointNames(), getVec(x, m_vars));

  for (const auto& link_name : manip_->getLinkNames())
    contact_manager_->setCollisionObjectsTransform(link_name, state->transforms[link_name]);

  contact_manager_->contactTest(contacts);
  tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars, x, exprs, false);

  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(tesseract::BasicKinConstPtr manip,
                                               tesseract::BasicEnvConstPtr env,
                                               SafetyMarginDataConstPtr safety_margin_data,
                                               const VarVector& vars0,
                                               const VarVector& vars1)
  : CollisionEvaluator(manip, env, safety_margin_data), m_vars0(vars0), m_vars1(vars1)
{
  tesseract::ContactRequest req;
  req.link_names = manip_->getLinkNames();
  req.contact_distance =
      safety_margin_data_->getMaxSafetyMargin() + 0.04;  // The original implementation added a margin of 0.04
  req.isContactAllowed = env_->getIsContactAllowedFn();
  req.type = tesseract::ContactRequestTypes::ALL;

  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setContactRequest(req);
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results)
{
  tesseract::ContactResultMap contacts;
  tesseract::EnvStatePtr state0 = env_->getState(manip_->getJointNames(), getVec(x, m_vars0));
  tesseract::EnvStatePtr state1 = env_->getState(manip_->getJointNames(), getVec(x, m_vars1));
  for (const auto& link_name : manip_->getLinkNames())
    contact_manager_->setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

  contact_manager_->contactTest(contacts);
  tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars0, m_vars1, x, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

//////////////////////////////////////////

CollisionCost::CollisionCost(tesseract::BasicKinConstPtr manip,
                             tesseract::BasicEnvConstPtr env,
                             SafetyMarginDataConstPtr safety_margin_data,
                             const VarVector& vars)
  : Cost("collision"), m_calc(new SingleTimestepCollisionEvaluator(manip, env, safety_margin_data, vars))
{
}

CollisionCost::CollisionCost(tesseract::BasicKinConstPtr manip,
                             tesseract::BasicEnvConstPtr env,
                             SafetyMarginDataConstPtr safety_margin_data,
                             const VarVector& vars0,
                             const VarVector& vars1)
  : Cost("cast_collision"), m_calc(new CastCollisionEvaluator(manip, env, safety_margin_data, vars0, vars1))
{
}

ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);

  tesseract::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    AffExpr viol = exprSub(AffExpr(data[0]), exprs[i]);
    out->addHinge(viol, data[1]);
  }
  return out;
}

double CollisionCost::value(const vector<double>& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  tesseract::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  double out = 0;
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);
    out += pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}

void CollisionCost::Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x) { m_calc->Plot(plotter, x); }
// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(tesseract::BasicKinConstPtr manip,
                                         tesseract::BasicEnvConstPtr env,
                                         SafetyMarginDataConstPtr safety_margin_data,
                                         const VarVector& vars)
  : m_calc(new SingleTimestepCollisionEvaluator(manip, env, safety_margin_data, vars))
{
  name_ = "collision";
}

CollisionConstraint::CollisionConstraint(tesseract::BasicKinConstPtr manip,
                                         tesseract::BasicEnvConstPtr env,
                                         SafetyMarginDataConstPtr safety_margin_data,
                                         const VarVector& vars0,
                                         const VarVector& vars1)
  : m_calc(new CastCollisionEvaluator(manip, env, safety_margin_data, vars0, vars1))
{
  name_ = "collision";
}

ConvexConstraintsPtr CollisionConstraint::convex(const vector<double>& x, Model* model)
{
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);

  tesseract::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    AffExpr viol = exprSub(AffExpr(data[0]), exprs[i]);
    out->addIneqCnt(exprMult(viol, data[1]));
  }
  return out;
}

DblVec CollisionConstraint::value(const vector<double>& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  tesseract::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  DblVec out(dists.size());
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    out[i] = pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}
}
