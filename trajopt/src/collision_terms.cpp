#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/utils.h>
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
void CollisionsToDistances(const tesseract_collision::ContactResultVector& dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());
  for (const auto& dist_result : dist_results)
    dists.push_back(dist_result.distance);
}

void DebugPrintInfo(const tesseract_collision::ContactResult& res,
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

void CollisionsToDistanceExpressions(const tesseract_collision::ContactResultVector& dist_results,
                                     const tesseract_kinematics::ForwardKinematics::ConstPtr& manip,
                                     const tesseract_environment::AdjacencyMap::ConstPtr& adjacency_map,
                                     const Eigen::Isometry3d& world_to_base,
                                     const sco::VarVector& vars,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs,
                                     bool isTimestep1)
{
  Eigen::VectorXd dofvals = sco::getVec(x, vars);

  // All collision data is in world corrdinate system. This provides the
  // transfrom for converting data between world frame and manipulator
  // frame.

  exprs.clear();
  exprs.reserve(dist_results.size());
  for (const auto& res : dist_results)
  {
    sco::AffExpr dist(res.distance);

    Eigen::VectorXd dist_grad_a, dist_grad_b;
    tesseract_environment::AdjacencyMapPair::ConstPtr itA = adjacency_map->getLinkMapping(res.link_names[0]);
    if (itA != nullptr)
    {
      Eigen::MatrixXd jac;
      jac.resize(6, manip->numJoints());

      // Calculate Jacobian
      manip->calcJacobian(jac, dofvals, itA->link_name);

      // Need to change the base and ref point of the jacobian.
      // When changing ref point you must provide a vector from the current ref
      // point to the new ref point.
      Eigen::Isometry3d link_transform;
      manip->calcFwdKin(link_transform, dofvals, itA->link_name);
      tesseract_kinematics::jacobianChangeBase(jac, world_to_base);
      tesseract_kinematics::jacobianChangeRefPoint(
          jac,
          (world_to_base * link_transform).linear() *
              ((world_to_base * link_transform).inverse() * res.nearest_points[0]));

      //      Eigen::MatrixXd jac_test;
      //      jac_test.resize(6, manip->numJoints());
      //      tesseract_kinematics::numericalJacobian(jac_test, world_to_base, *manip, dofvals, itA->link_name,
      //      (world_to_base * link_transform).inverse() * res.nearest_points[0]); bool check = jac.isApprox(jac_test,
      //      1e-3);

      dist_grad_a = -res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_a, vars));
      sco::exprInc(dist, -dist_grad_a.dot(dofvals));
    }

    tesseract_environment::AdjacencyMapPair::ConstPtr itB = adjacency_map->getLinkMapping(res.link_names[1]);
    if (itB != nullptr)
    {
      Eigen::MatrixXd jac;
      jac.resize(6, manip->numJoints());

      // Calculate Jacobian
      manip->calcJacobian(jac, dofvals, itB->link_name);

      // Need to change the base and ref point of the jacobian.
      // When changing ref point you must provide a vector from the current ref
      // point to the new ref point.
      Eigen::Isometry3d link_transform;
      manip->calcFwdKin(link_transform, dofvals, itB->link_name);
      tesseract_kinematics::jacobianChangeBase(jac, world_to_base);
      Eigen::Vector3d link_point =
          (isTimestep1 && (res.cc_type == tesseract_collision::ContinouseCollisionType::CCType_Between)) ?
              res.cc_nearest_points[1] :
              res.nearest_points[1];
      tesseract_kinematics::jacobianChangeRefPoint(
          jac, (world_to_base * link_transform).linear() * ((world_to_base * link_transform).inverse() * link_point));

      //      Eigen::MatrixXd jac_test;
      //      jac_test.resize(6, manip->numJoints());
      //      tesseract_kinematics::numericalJacobian(jac_test, world_to_base, *manip, dofvals, itB->link_name,
      //      (world_to_base * link_transform).inverse() * link_point); bool check = jac.isApprox(jac_test, 1e-3);

      dist_grad_b = res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_b, vars));
      sco::exprInc(dist, -dist_grad_b.dot(dofvals));
    }
    // DebugPrintInfo(res, dist_grad_a, dist_grad_b, dofvals, i == 0);

    if (itA != nullptr || itB != nullptr)
    {
      exprs.push_back(dist);
    }
  }
}

void CollisionsToDistanceExpressions(const tesseract_collision::ContactResultVector& dist_results,
                                     const tesseract_kinematics::ForwardKinematics::ConstPtr& manip,
                                     const tesseract_environment::AdjacencyMap::ConstPtr& adjacency_map,
                                     const Eigen::Isometry3d& world_to_base,
                                     const sco::VarVector& vars0,
                                     const sco::VarVector& vars1,
                                     const DblVec& x,
                                     sco::AffExprVector& exprs)
{
  sco::AffExprVector exprs0, exprs1;
  CollisionsToDistanceExpressions(dist_results, manip, adjacency_map, world_to_base, vars0, x, exprs0, false);
  CollisionsToDistanceExpressions(dist_results, manip, adjacency_map, world_to_base, vars1, x, exprs1, true);

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
void CollisionEvaluator::GetCollisionsCached(const DblVec& x, tesseract_collision::ContactResultVector& dist_results)
{
  size_t key = hash(sco::getDblVec(x, GetVars()));
  tesseract_collision::ContactResultVector* it = m_cache.get(key);
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

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(
    tesseract_kinematics::ForwardKinematics::ConstPtr manip,
    tesseract_environment::Environment::ConstPtr env,
    tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
    const Eigen::Isometry3d& world_to_base,
    SafetyMarginData::ConstPtr safety_margin_data,
    sco::VarVector vars)
  : CollisionEvaluator(std::move(manip),
                       std::move(env),
                       std::move(adjacency_map),
                       world_to_base,
                       std::move(safety_margin_data))
  , m_vars(std::move(vars))
{
  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                                0.04);  // The original implementation added a margin of 0.04;
}

void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x,
                                                      tesseract_collision::ContactResultVector& dist_results)
{
  tesseract_collision::ContactResultMap contacts;
  tesseract_environment::EnvState::Ptr state = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars));

  for (const auto& link_name : adjacency_map_->getActiveLinkNames())
    contact_manager_->setCollisionObjectsTransform(link_name, state->transforms[link_name]);

  contact_manager_->contactTest(contacts, tesseract_collision::ContactTestType::ALL);

  tesseract_collision::ContactResultVector temp;
  tesseract_collision::flattenResults(std::move(contacts), temp);

  // Because each pair can have its own contact distance we need to
  // filter out collision pairs that are outside the pairs threshold
  dist_results.reserve(temp.size());
  for (auto& res : temp)
  {
    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0], res.link_names[1]);
    if (data[0] > res.distance)
      dist_results.emplace_back(res);
  }
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  tesseract_collision::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  tesseract_collision::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, manip_, adjacency_map_, world_to_base_, m_vars, x, exprs, false);

  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

void SingleTimestepCollisionEvaluator::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x)
{
  tesseract_collision::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  Eigen::VectorXd dofvals = sco::getVec(x, m_vars);

  Eigen::VectorXd safety_distance(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    const tesseract_collision::ContactResult& res = dist_results[i];
    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0], res.link_names[1]);
    safety_distance[i] = data[0];

    tesseract_environment::AdjacencyMapPair::ConstPtr itA = adjacency_map_->getLinkMapping(res.link_names[0]);
    if (itA != nullptr)
    {
      Eigen::MatrixXd jac;
      Eigen::VectorXd dist_grad;
      Eigen::Isometry3d pose, pose2;
      jac.resize(6, manip_->numJoints());
      manip_->calcFwdKin(pose, dofvals, itA->link_name);
      pose = world_to_base_ * pose;

      Eigen::Vector3d local_link_point = pose.inverse() * res.nearest_points[0];

      manip_->calcJacobian(jac, dofvals, itA->link_name);
      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
      tesseract_kinematics::jacobianChangeRefPoint(jac, pose.linear() * local_link_point);

      dist_grad = -res.normal.transpose() * jac.topRows(3);

      manip_->calcFwdKin(pose2, dofvals + dist_grad, itA->link_name);
      pose2 = world_to_base_ * pose2 * itA->transform;
      plotter->plotArrow(res.nearest_points[0], pose2 * local_link_point, Eigen::Vector4d(1, 1, 1, 1), 0.005);
    }
  }

  plotter->plotContactResults(adjacency_map_->getActiveLinkNames(), dist_results, safety_distance);
}

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                               tesseract_environment::Environment::ConstPtr env,
                                               tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                               const Eigen::Isometry3d& world_to_base,
                                               SafetyMarginData::ConstPtr safety_margin_data,
                                               sco::VarVector vars0,
                                               sco::VarVector vars1)
  : CollisionEvaluator(std::move(manip),
                       std::move(env),
                       std::move(adjacency_map),
                       world_to_base,
                       std::move(safety_margin_data))
  , m_vars0(std::move(vars0))
  , m_vars1(std::move(vars1))
{
  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
  contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
                                                0.04);  // The original implementation added a margin of 0.04;
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results)
{
  tesseract_collision::ContactResultMap contacts;
  tesseract_environment::EnvState::Ptr state0 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars0));
  tesseract_environment::EnvState::Ptr state1 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars1));
  for (const auto& link_name : adjacency_map_->getActiveLinkNames())
    contact_manager_->setCollisionObjectsTransform(
        link_name, state0->transforms[link_name], state1->transforms[link_name]);

  contact_manager_->contactTest(contacts, tesseract_collision::ContactTestType::ALL);

  tesseract_collision::ContactResultVector temp;
  tesseract_collision::flattenResults(std::move(contacts), temp);

  // Because each pair can have its own contact distance we need to
  // filter out collision pairs that are outside the pairs threshold
  dist_results.reserve(temp.size());
  for (auto& res : temp)
  {
    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0], res.link_names[1]);
    if (data[0] > res.distance)
      dist_results.emplace_back(res);
  }
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  tesseract_collision::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, manip_, adjacency_map_, world_to_base_, m_vars0, m_vars1, x, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& exprs)
{
  tesseract_collision::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, exprs);
}

void CastCollisionEvaluator::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x)
{
  // TODO LEVI: Need to improve this to match casted object
  tesseract_collision::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  Eigen::VectorXd dofvals = sco::getVec(x, m_vars0);

  Eigen::VectorXd safety_distance(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    const tesseract_collision::ContactResult& res = dist_results[i];
    const Eigen::Vector2d& data = getSafetyMarginData()->getPairSafetyMarginData(res.link_names[0], res.link_names[1]);
    safety_distance[i] = data[0];

    tesseract_environment::AdjacencyMapPair::ConstPtr itA = adjacency_map_->getLinkMapping(res.link_names[0]);
    if (itA != nullptr)
    {
      Eigen::MatrixXd jac;
      Eigen::VectorXd dist_grad;
      Eigen::Isometry3d pose, pose2;
      jac.resize(6, manip_->numJoints());
      manip_->calcFwdKin(pose, dofvals, itA->link_name);
      pose = world_to_base_ * pose;

      Eigen::Vector3d local_link_point = pose.inverse() * res.nearest_points[0];

      manip_->calcJacobian(jac, dofvals, itA->link_name);
      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
      tesseract_kinematics::jacobianChangeRefPoint(jac, pose.linear() * local_link_point);

      //      Eigen::MatrixXd jac_test;
      //      jac_test.resize(6, manip_->numJoints());
      //      tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itA->link_name,
      //      local_link_point); bool check = jac.isApprox(jac_test, 1e-3);

      dist_grad = -res.normal.transpose() * jac.topRows(3);

      manip_->calcFwdKin(pose2, dofvals + dist_grad, itA->link_name);
      pose2 = world_to_base_ * pose2 * itA->transform;
      plotter->plotArrow(res.nearest_points[0], pose2 * local_link_point, Eigen::Vector4d(1, 1, 1, 1), 0.005);
    }

    tesseract_environment::AdjacencyMapPair::ConstPtr itB = adjacency_map_->getLinkMapping(res.link_names[1]);
    if (itB != nullptr)
    {
      Eigen::MatrixXd jac;
      Eigen::VectorXd dist_grad;
      Eigen::Isometry3d pose, pose2;
      jac.resize(6, manip_->numJoints());
      manip_->calcFwdKin(pose, dofvals, itB->link_name);
      pose = world_to_base_ * pose;

      // Calculate Jacobian
      manip_->calcJacobian(jac, dofvals, itB->link_name);

      tesseract_kinematics::jacobianChangeBase(jac, world_to_base_);
      bool isTimestep1 = false;
      Eigen::Vector3d link_point =
          (isTimestep1 && (res.cc_type == tesseract_collision::ContinouseCollisionType::CCType_Between)) ?
              res.cc_nearest_points[1] :
              res.nearest_points[1];
      Eigen::Vector3d local_link_point = pose.inverse() * link_point;
      tesseract_kinematics::jacobianChangeRefPoint(jac, pose.linear() * local_link_point);

      //      Eigen::MatrixXd jac_test;
      //      jac_test.resize(6, manip_->numJoints());
      //      tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itB->link_name,
      //      local_link_point); bool check = jac.isApprox(jac_test, 1e-3);

      dist_grad = res.normal.transpose() * jac.topRows(3);

      manip_->calcFwdKin(pose2, dofvals + dist_grad, itB->link_name);
      pose2 = world_to_base_ * pose2 * itB->transform;
      plotter->plotArrow(link_point, pose2 * local_link_point, Eigen::Vector4d(1, 1, 1, 1), 0.005);
    }
  }

  plotter->plotContactResults(adjacency_map_->getActiveLinkNames(), dist_results, safety_distance);
}

//////////////////////////////////////////

CollisionCost::CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                             tesseract_environment::Environment::ConstPtr env,
                             tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                             const Eigen::Isometry3d& world_to_base,
                             SafetyMarginData::ConstPtr safety_margin_data,
                             sco::VarVector vars)
  : Cost("collision")
  , m_calc(new SingleTimestepCollisionEvaluator(std::move(manip),
                                                std::move(env),
                                                std::move(adjacency_map),
                                                world_to_base,
                                                std::move(safety_margin_data),
                                                std::move(vars)))
{
}

CollisionCost::CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                             tesseract_environment::Environment::ConstPtr env,
                             tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                             const Eigen::Isometry3d& world_to_base,
                             SafetyMarginData::ConstPtr safety_margin_data,
                             sco::VarVector vars0,
                             sco::VarVector vars1)
  : Cost("cast_collision")
  , m_calc(new CastCollisionEvaluator(std::move(manip),
                                      std::move(env),
                                      std::move(adjacency_map),
                                      world_to_base,
                                      std::move(safety_margin_data),
                                      std::move(vars0),
                                      std::move(vars1)))
{
}

sco::ConvexObjective::Ptr CollisionCost::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexObjective::Ptr out(new sco::ConvexObjective(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  tesseract_collision::ContactResultVector dist_results;
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

  tesseract_collision::ContactResultVector dist_results;
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

void CollisionCost::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x)
{
  m_calc->Plot(plotter, x);
}
// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                         tesseract_environment::Environment::ConstPtr env,
                                         tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                         const Eigen::Isometry3d& world_to_base,
                                         SafetyMarginData::ConstPtr safety_margin_data,
                                         sco::VarVector vars)
  : m_calc(new SingleTimestepCollisionEvaluator(std::move(manip),
                                                std::move(env),
                                                std::move(adjacency_map),
                                                world_to_base,
                                                std::move(safety_margin_data),
                                                std::move(vars)))
{
  name_ = "collision";
}

CollisionConstraint::CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                         tesseract_environment::Environment::ConstPtr env,
                                         tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                         const Eigen::Isometry3d& world_to_base,
                                         SafetyMarginData::ConstPtr safety_margin_data,
                                         sco::VarVector vars0,
                                         sco::VarVector vars1)
  : m_calc(new CastCollisionEvaluator(std::move(manip),
                                      std::move(env),
                                      std::move(adjacency_map),
                                      world_to_base,
                                      std::move(safety_margin_data),
                                      std::move(vars0),
                                      std::move(vars1)))
{
  name_ = "collision";
}

sco::ConvexConstraints::Ptr CollisionConstraint::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexConstraints::Ptr out(new sco::ConvexConstraints(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  tesseract_collision::ContactResultVector dist_results;
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

  tesseract_collision::ContactResultVector dist_results;
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
}  // namespace trajopt
