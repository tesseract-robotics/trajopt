#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/expr_vec_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

namespace trajopt
{
void CollisionsToDistances(const tesseract::ContactResultVector& dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
    dists.push_back(dist_results[i].distance);
}

void DebugPrintInfo(const tesseract::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals,
                    bool header = false)
{
  if (header)
  {
    std::printf("\n");
    std::printf("DistanceResult| %30s | %30s | %6s | %6s, %6s, %6s | %6s, %6s, "
                "%6s | %6s, %6s, %6s | %7s | %6s, %6s, %6s | %6s, %6s, %6s |",
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

    for (auto i = 0; i < dist_grad_A.size(); ++i)
    {
      if (i == dist_grad_A.size() - 1)
      {
        std::printf(" %6s |", ("dA" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("dA" + std::to_string(i)).c_str());
      }
    }

    for (auto i = 0; i < dist_grad_B.size(); ++i)
    {
      if (i == dist_grad_B.size() - 1)
      {
        std::printf(" %6s |", ("dB" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("dB" + std::to_string(i)).c_str());
      }
    }

    for (auto i = 0; i < dof_vals.size(); ++i)
    {
      if (i == dof_vals.size() - 1)
      {
        std::printf(" %6s |", ("J" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("J" + std::to_string(i)).c_str());
      }
    }

    std::printf("\n");
  }

  std::printf("DistanceResult| %30s | %30s | %6.3f | %6.3f, %6.3f, %6.3f | "
              "%6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %7.3f | %6.3f, "
              "%6.3f, %6.3f | %6.3f, %6.3f, %6.3f |",
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

  for (auto i = 0; i < dist_grad_A.size(); ++i)
  {
    if (i == dist_grad_A.size() - 1)
    {
      std::printf(" %6.3f |", dist_grad_A(i));
    }
    else
    {
      std::printf(" %6.3f,", dist_grad_A(i));
    }
  }

  for (auto i = 0; i < dist_grad_B.size(); ++i)
  {
    if (i == dist_grad_B.size() - 1)
    {
      std::printf(" %6.3f |", dist_grad_B(i));
    }
    else
    {
      std::printf(" %6.3f,", dist_grad_B(i));
    }
  }

  for (auto i = 0; i < dof_vals.size(); ++i)
  {
    if (i == dof_vals.size() - 1)
    {
      std::printf(" %6.3f |", dof_vals(i));
    }
    else
    {
      std::printf(" %6.3f,", dof_vals(i));
    }
  }

  std::printf("\n");
}

void CollisionsToDistanceExpressions(const tesseract::ContactResultVector& dist_results,
                                     const tesseract::BasicEnvConstPtr env,
                                     const tesseract::BasicKinConstPtr manip,
                                     const sco::VarVector& vars,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs,
                                     bool isTimestep1)
{
  Eigen::VectorXd dofvals = sco::getVec(x, vars);
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

    sco::AffExpr dist(res.distance);

    Eigen::VectorXd dist_grad_a, dist_grad_b;
    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      Eigen::MatrixXd jac;
      jac.resize(6, manip->numJoints());
      manip->calcJacobian(jac, change_base, dofvals, res.link_names[0], *state, res.nearest_points[0]);
      dist_grad_a = -res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_a, vars));
      sco::exprInc(dist, -dist_grad_a.dot(dofvals));
    }

    std::vector<std::string>::const_iterator itB = std::find(link_names.begin(), link_names.end(), res.link_names[1]);
    if (itB != link_names.end())
    {
      Eigen::MatrixXd jac;
      jac.resize(6, manip->numJoints());
      manip->calcJacobian(jac,
                          change_base,
                          dofvals,
                          res.link_names[1],
                          *state,
                          (isTimestep1 && (res.cc_type == tesseract::ContinouseCollisionType::CCType_Between)) ?
                              res.cc_nearest_points[1] :
                              res.nearest_points[1]);
      dist_grad_b = res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_b, vars));
      sco::exprInc(dist, -dist_grad_b.dot(dofvals));
    }
    // DebugPrintInfo(res, dist_grad_a, dist_grad_b, dofvals, i == 0);

    if (itA != link_names.end() || itB != link_names.end())
    {
      exprs.push_back(dist);
    }
  }
}

void CollisionsToDistanceExpressions(const tesseract::ContactResultVector& dist_results,
                                     const tesseract::BasicEnvConstPtr env,
                                     const tesseract::BasicKinConstPtr manip,
                                     const sco::VarVector& vars0,
                                     const sco::VarVector& vars1,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs)
{
  sco::AffExprVector exprs0, exprs1;
  CollisionsToDistanceExpressions(dist_results, env, manip, vars0, x, exprs0, false);
  CollisionsToDistanceExpressions(dist_results, env, manip, vars1, x, exprs1, true);

  exprs.resize(exprs0.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    assert(dist_results[i].cc_time >= 0.0 && dist_results[i].cc_time <= 1.0);
    sco::exprScale(exprs0[i], (1 - dist_results[i].cc_time));
    sco::exprScale(exprs1[i], dist_results[i].cc_time);
    exprs[i] = sco::AffExpr(0);
    sco::exprInc(exprs[i], exprs0[i]);
    sco::exprInc(exprs[i], exprs1[i]);
    sco::cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x) { return boost::hash_range(x.begin(), x.end()); }
void CollisionEvaluator::GetCollisionsCached(const DblVec& x, tesseract::ContactResultVector& dist_results)
{
  size_t key = hash(sco::getDblVec(x, GetVars()));
  tesseract::ContactResultVector* it = m_cache.get(key);
  if (it != nullptr)
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

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(tesseract::BasicKinConstPtr manip,
                                                                   tesseract::BasicEnvConstPtr env,
                                                                   SafetyMarginDataConstPtr safety_margin_data,
                                                                   const sco::VarVector& vars)
  : CollisionEvaluator(manip, env, safety_margin_data), m_vars(vars)
{
  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                                0.04);  // The original implementation added a margin of 0.04;
}

void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results)
{
  tesseract::ContactResultMap contacts;
  tesseract::EnvStatePtr state = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars));

  for (const auto& link_name : manip_->getLinkNames())
    contact_manager_->setCollisionObjectsTransform(link_name, state->transforms[link_name]);

  contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, env_, manip_, m_vars, x, exprs, false);

  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

void SingleTimestepCollisionEvaluator::Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x)
{
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  const std::vector<std::string>& link_names = manip_->getLinkNames();
  tesseract::EnvStateConstPtr state = env_->getState();
  Eigen::Isometry3d change_base = state->transforms.at(manip_->getBaseLinkName());
  Eigen::VectorXd dofvals = sco::getVec(x, m_vars);

  Eigen::VectorXd safety_distance(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    const tesseract::ContactResult& res = dist_results[i];
    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0], res.link_names[1]);
    safety_distance[i] = data[0];

    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      Eigen::MatrixXd jac;
      Eigen::VectorXd dist_grad;
      Eigen::Isometry3d pose, pose2;
      jac.resize(6, manip_->numJoints());
      manip_->calcFwdKin(pose, change_base, dofvals, res.link_names[0], *state);
      manip_->calcJacobian(jac, change_base, dofvals, res.link_names[0], *state, res.nearest_points[0]);
      dist_grad = -res.normal.transpose() * jac.topRows(3);

      Eigen::Vector3d local_link_point = pose.inverse() * res.nearest_points[0];
      manip_->calcFwdKin(pose2, change_base, dofvals + dist_grad, res.link_names[0], *state);
      plotter->plotArrow(res.nearest_points[0], pose2 * local_link_point, Eigen::Vector4d(1, 1, 1, 1), 0.005);
    }
  }

  plotter->plotContactResults(link_names, dist_results, safety_distance);
}

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(tesseract::BasicKinConstPtr manip,
                                               tesseract::BasicEnvConstPtr env,
                                               SafetyMarginDataConstPtr safety_margin_data,
                                               const sco::VarVector& vars0,
                                               const sco::VarVector& vars1)
  : CollisionEvaluator(manip, env, safety_margin_data), m_vars0(vars0), m_vars1(vars1)
{
  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                                0.04);  // The original implementation added a margin of 0.04;
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract::ContactResultVector& dist_results)
{
  tesseract::ContactResultMap contacts;
  tesseract::EnvStatePtr state0 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars0));
  tesseract::EnvStatePtr state1 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars1));
  for (const auto& link_name : manip_->getLinkNames())
    contact_manager_->setCollisionObjectsTransform(
        link_name, state0->transforms[link_name], state1->transforms[link_name]);

  contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
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

void CastCollisionEvaluator::Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x)
{
  // TODO LEVI: Need to improve this to match casted object
  tesseract::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  const std::vector<std::string>& link_names = manip_->getLinkNames();
  tesseract::EnvStateConstPtr state = env_->getState();
  Eigen::Isometry3d change_base = state->transforms.at(manip_->getBaseLinkName());
  Eigen::VectorXd dofvals = sco::getVec(x, m_vars0);

  Eigen::VectorXd safety_distance(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    const tesseract::ContactResult& res = dist_results[i];
    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0], res.link_names[1]);
    safety_distance[i] = data[0];

    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      Eigen::MatrixXd jac;
      Eigen::VectorXd dist_grad;
      Eigen::Isometry3d pose, pose2;
      jac.resize(6, manip_->numJoints());
      manip_->calcFwdKin(pose, change_base, dofvals, res.link_names[0], *state);
      manip_->calcJacobian(jac, change_base, dofvals, res.link_names[0], *state, res.nearest_points[0]);
      dist_grad = -res.normal.transpose() * jac.topRows(3);

      Eigen::Vector3d local_link_point = pose.inverse() * res.nearest_points[0];
      manip_->calcFwdKin(pose2, change_base, dofvals + dist_grad, res.link_names[0], *state);
      plotter->plotArrow(res.nearest_points[0], pose2 * local_link_point, Eigen::Vector4d(1, 1, 1, 1), 0.005);
    }
  }

  plotter->plotContactResults(link_names, dist_results, safety_distance);
}

//////////////////////////////////////////

CollisionCost::CollisionCost(tesseract::BasicKinConstPtr manip,
                             tesseract::BasicEnvConstPtr env,
                             SafetyMarginDataConstPtr safety_margin_data,
                             const sco::VarVector& vars)
  : Cost("collision"), m_calc(new SingleTimestepCollisionEvaluator(manip, env, safety_margin_data, vars))
{
}

CollisionCost::CollisionCost(tesseract::BasicKinConstPtr manip,
                             tesseract::BasicEnvConstPtr env,
                             SafetyMarginDataConstPtr safety_margin_data,
                             const sco::VarVector& vars0,
                             const sco::VarVector& vars1)
  : Cost("cast_collision"), m_calc(new CastCollisionEvaluator(manip, env, safety_margin_data, vars0, vars1))
{
}

sco::ConvexObjectivePtr CollisionCost::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  tesseract::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
    out->addHinge(viol, data[1]);
  }
  return out;
}

double CollisionCost::value(const sco::DblVec& x)
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
    out += sco::pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}

void CollisionCost::Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) { m_calc->Plot(plotter, x); }
// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(tesseract::BasicKinConstPtr manip,
                                         tesseract::BasicEnvConstPtr env,
                                         SafetyMarginDataConstPtr safety_margin_data,
                                         const sco::VarVector& vars)
  : m_calc(new SingleTimestepCollisionEvaluator(manip, env, safety_margin_data, vars))
{
  name_ = "collision";
}

CollisionConstraint::CollisionConstraint(tesseract::BasicKinConstPtr manip,
                                         tesseract::BasicEnvConstPtr env,
                                         SafetyMarginDataConstPtr safety_margin_data,
                                         const sco::VarVector& vars0,
                                         const sco::VarVector& vars1)
  : m_calc(new CastCollisionEvaluator(manip, env, safety_margin_data, vars0, vars1))
{
  name_ = "collision";
}

sco::ConvexConstraintsPtr CollisionConstraint::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  tesseract::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
    out->addIneqCnt(sco::exprMult(viol, data[1]));
  }
  return out;
}

DblVec CollisionConstraint::value(const sco::DblVec& x)
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

    out[i] = sco::pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}
}
