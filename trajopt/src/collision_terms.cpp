#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/arrow_marker.h>
#include <tesseract_visualization/markers/contact_results_marker.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/expr_vec_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/collision_utils.h>
#include <trajopt_common/utils.hpp>

namespace trajopt
{
namespace
{
void CollisionsToDistances(const ContactResultVectorWrapper& dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());
  for (const auto& dist_result : dist_results)
    dists.push_back(dist_result.get().distance);
}

void DebugPrintInfoHeader(Eigen::Index dof)
{
  std::printf("\n");
  std::printf("DistanceResult| %30s | %30s | %6s | %6s, %6s, %6s | %6s, %6s, "
              "%6s | %6s, %6s, %6s | %6s, %6s, %6s | %6s, %6s, %6s | %10s %10s |",
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
              "LPAx",
              "LPAy",
              "LPAz",
              "LPBx",
              "LPBy",
              "LPBz",
              "CC TIME A",
              "CC TIME B");

  for (auto i = 0; i < dof; ++i)
  {
    if (i == dof - 1)
    {
      std::printf(" %6s |", ("dA" + std::to_string(i)).c_str());
    }
    else
    {
      std::printf(" %6s,", ("dA" + std::to_string(i)).c_str());
    }
  }

  for (auto i = 0; i < dof; ++i)
  {
    if (i == dof - 1)
    {
      std::printf(" %6s |", ("dB" + std::to_string(i)).c_str());
    }
    else
    {
      std::printf(" %6s,", ("dB" + std::to_string(i)).c_str());
    }
  }

  for (auto i = 0; i < dof; ++i)
  {
    if (i == dof - 1)
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

void DebugPrintInfo(const tesseract_collision::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals)
{
  std::printf("DistanceResult| %30s | %30s | %6.3f | %6.3f, %6.3f, %6.3f | "
              "%6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %6.3f, "
              "%6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %10.3f %10.3f |",
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
              res.nearest_points_local[0](0),
              res.nearest_points_local[0](1),
              res.nearest_points_local[0](2),
              res.nearest_points_local[1](0),
              res.nearest_points_local[1](1),
              res.nearest_points_local[1](2),
              res.cc_time[0],
              res.cc_time[1]);

  for (auto i = 0; i < dof_vals.size(); ++i)
  {
    if (dist_grad_A.size() == dof_vals.size())
    {
      if (i == dof_vals.size() - 1)
      {
        std::printf(" %6.3f |", dist_grad_A(i));
      }
      else
      {
        std::printf(" %6.3f,", dist_grad_A(i));
      }
    }
    else
    {
      if (i == dof_vals.size() - 1)
      {
        std::printf(" %6s |", "NA");
      }
      else
      {
        std::printf(" %6s,", "NA");
      }
    }
  }

  for (auto i = 0; i < dist_grad_B.size(); ++i)
  {
    if (dist_grad_B.size() == dof_vals.size())
    {
      if (i == dof_vals.size() - 1)
      {
        std::printf(" %6.3f |", dist_grad_B(i));
      }
      else
      {
        std::printf(" %6.3f,", dist_grad_B(i));
      }
    }
    else
    {
      if (i == dof_vals.size() - 1)
      {
        std::printf(" %6s |", "NA");
      }
      else
      {
        std::printf(" %6s,", "NA");
      }
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

}  // namespace

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals,
                                                const tesseract_collision::ContactResult& contact_result,
                                                double margin,
                                                double coeff,
                                                bool isTimestep1)
{
  GradientResults results(margin, coeff);
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (manip_->isActiveLinkName(contact_result.link_names[i]))
    {
      results.gradients[i].has_gradient = true;

      // Calculate Jacobian
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvals, contact_result.link_names[i]);

      // Need to change the base and ref point of the jacobian.
      // When changing ref point you must provide a vector from the current ref
      // point to the new ref point.
      results.gradients[i].scale = 1;
      Eigen::Isometry3d link_transform = contact_result.transform[i];
      if (contact_result.cc_type[i] != tesseract_collision::ContinuousCollisionType::CCType_None)
      {
        assert(contact_result.cc_time[i] >= 0.0 && contact_result.cc_time[i] <= 1.0);
        results.gradients[i].scale = (isTimestep1) ? contact_result.cc_time[i] : (1 - contact_result.cc_time[i]);
        link_transform = (isTimestep1) ? contact_result.cc_transform[i] : contact_result.transform[i];
      }
      // Since the link transform is known do not call calcJacobian with link point
      tesseract_common::jacobianChangeRefPoint(jac, link_transform.linear() * contact_result.nearest_points_local[i]);

#ifndef NDEBUG
//      Eigen::Isometry3d test_link_transform = manip_->calcFwdKin(dofvals).at(contact_result.link_names[i]);
//      assert(test_link_transform.isApprox(link_transform, 0.0001));

//      Eigen::MatrixXd jac_test;
//      jac_test.resize(6, manip_->numJoints());
//      tesseract_kinematics::numericalJacobian(jac_test, *manip_, dofvals, contact_result.link_names[i],
//      contact_result.nearest_points_local[i]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);
#endif

      results.gradients[i].gradient = ((i == 0) ? -1.0 : 1.0) * contact_result.normal.transpose() * jac.topRows(3);
    }
  }

  // DebugPrintInfo(contact_result, results.gradients[0].gradient, results.gradients[1].gradient, dofvals);

  return results;
}

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals,
                                                const tesseract_collision::ContactResult& contact_result,
                                                bool isTimestep1)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  const double margin = margin_data_.getCollisionMargin(contact_result.link_names[0], contact_result.link_names[1]);
  const double coeff = coeff_data_.getCollisionCoeff(contact_result.link_names[0], contact_result.link_names[1]);
  return GetGradient(dofvals, contact_result, margin, coeff, isTimestep1);
}

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals0,
                                                const Eigen::VectorXd& dofvals1,
                                                const tesseract_collision::ContactResult& contact_result,
                                                double margin,
                                                double coeff,
                                                bool isTimestep1)
{
  GradientResults results(margin, coeff);
  Eigen::VectorXd dofvalst = Eigen::VectorXd::Zero(dofvals0.size());
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (manip_->isActiveLinkName(contact_result.link_names[i]))
    {
      results.gradients[i].has_gradient = true;

      if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
        dofvalst = dofvals0;
      else if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
        dofvalst = dofvals1;
      else
        dofvalst = dofvals0 + (dofvals1 - dofvals0) * contact_result.cc_time[i];

      // Calculate Jacobian
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvalst, contact_result.link_names[i]);

      // Need to change the base and ref point of the jacobian.
      // When changing ref point you must provide a vector from the current ref
      // point to the new ref point.
      results.gradients[i].scale = 1;
      Eigen::Isometry3d link_transform = contact_result.transform[i];

      assert(contact_result.cc_time[i] >= 0.0 && contact_result.cc_time[i] <= 1.0);
      results.gradients[i].scale = (isTimestep1) ? contact_result.cc_time[i] : (1 - contact_result.cc_time[i]);
      link_transform = (isTimestep1) ? contact_result.cc_transform[i] : contact_result.transform[i];

      // Since the link transform is known do not call calcJacobian with link point
      tesseract_common::jacobianChangeRefPoint(jac, link_transform.linear() * contact_result.nearest_points_local[i]);

#ifndef NDEBUG
      const Eigen::Isometry3d test_link_transform = manip_->calcFwdKin(dofvalst)[contact_result.link_names[i]];
      assert(test_link_transform.isApprox(link_transform, 0.0001));

      Eigen::MatrixXd jac_test;
      jac_test.resize(6, manip_->numJoints());
      tesseract_kinematics::numericalJacobian(jac_test,
                                              Eigen::Isometry3d::Identity(),
                                              *manip_,
                                              dofvalst,
                                              contact_result.link_names[i],
                                              contact_result.nearest_points_local[i]);
      const bool check = jac.isApprox(jac_test, 1e-3);
      assert(check == true);
#endif

      results.gradients[i].gradient = ((i == 0) ? -1.0 : 1.0) * contact_result.normal.transpose() * jac.topRows(3);
    }
  }

  // DebugPrintInfo(contact_result, results.gradients[0].gradient, results.gradients[1].gradient, dofvalst);

  return results;
}

GradientResults CollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals0,
                                                const Eigen::VectorXd& dofvals1,
                                                const tesseract_collision::ContactResult& contact_result,
                                                bool isTimestep1)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  const double margin = margin_data_.getCollisionMargin(contact_result.link_names[0], contact_result.link_names[1]);
  const double coeff = coeff_data_.getCollisionCoeff(contact_result.link_names[0], contact_result.link_names[1]);
  return GetGradient(dofvals0, dofvals1, contact_result, margin, coeff, isTimestep1);
}

const tesseract_common::CollisionMarginData& CollisionEvaluator::getCollisionMarginData() const { return margin_data_; }

const trajopt_common::CollisionCoeffData& CollisionEvaluator::getCollisionCoeffData() const { return coeff_data_; }

void CollisionEvaluator::CollisionsToDistanceExpressions(sco::AffExprVector& exprs,
                                                         std::vector<double>& exprs_margin,
                                                         std::vector<double>& exprs_coeff,
                                                         const ContactResultVectorWrapper& dist_results,
                                                         const sco::VarVector& vars,
                                                         const DblVec& x,
                                                         bool isTimestep1)
{
  const Eigen::VectorXd dofvals = sco::getVec(x, vars);

  // All collision data is in world coordinate system. This provides the
  // transform for converting data between world frame and manipulator
  // frame.

  exprs.clear();
  exprs_margin.clear();
  exprs_coeff.clear();
  exprs.reserve(dist_results.size());
  exprs_margin.reserve(dist_results.size());
  exprs_coeff.reserve(dist_results.size());
  for (const auto& res : dist_results)
  {
    sco::AffExpr dist(0);
    GradientResults grad = GetGradient(dofvals, res.get(), isTimestep1);
    for (const auto& g : grad.gradients)
    {
      if (g.has_gradient)
      {
        sco::exprInc(dist, sco::varDot(g.scale * g.gradient, vars));
        sco::exprInc(dist, g.scale * -g.gradient.dot(dofvals));
      }
    }

    if (grad.gradients[0].has_gradient || grad.gradients[1].has_gradient)
    {
      exprs.push_back(dist);
      exprs_margin.push_back(grad.margin);
      exprs_coeff.push_back(grad.coeff);
    }
  }
}

void CollisionEvaluator::CollisionsToDistanceExpressionsW(sco::AffExprVector& exprs,
                                                          std::vector<double>& exprs_margin,
                                                          std::vector<double>& exprs_coeff,
                                                          const tesseract_collision::ContactResultMap& dist_results,
                                                          const sco::VarVector& vars,
                                                          const DblVec& x,
                                                          bool isTimestep1)
{
  const Eigen::VectorXd dofvals = sco::getVec(x, vars);

  // All collision data is in world coordinate system. This provides the
  // transform for converting data between world frame and manipulator
  // frame.

  exprs.clear();
  exprs_margin.clear();
  exprs_coeff.clear();
  exprs.reserve(dist_results.count());
  exprs_margin.reserve(dist_results.count());
  exprs_coeff.reserve(dist_results.count());
  for (const auto& pair : dist_results)
  {
    if (pair.second.empty())
      continue;

    double worst_dist{ std::numeric_limits<double>::max() };
    double total_weight[2];
    bool found[2];
    Eigen::VectorXd dist_grad[2];

    total_weight[0] = 0;
    total_weight[1] = 0;

    found[0] = false;
    found[1] = false;

    dist_grad[0] = Eigen::VectorXd::Zero(manip_->numJoints());
    dist_grad[1] = Eigen::VectorXd::Zero(manip_->numJoints());

    // Contains the contact distance threshold and coefficient for the given link pair
    const double margin = margin_data_.getCollisionMargin(pair.first.first, pair.first.second);
    const double coeff = coeff_data_.getCollisionCoeff(pair.first.first, pair.first.second);
    for (const auto& res : pair.second)
    {
      GradientResults grad = GetGradient(dofvals, res, margin, coeff, isTimestep1);

      for (std::size_t i = 0; i < 2; ++i)
      {
        // Changing the start state does not have an affect if the collision is at the end state so do not process
        // Changing the end state does not have an affect if the collision is at the start state so do not process
        if (grad.gradients[i].has_gradient &&
            !(!isTimestep1 && res.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time1) &&
            !(isTimestep1 && res.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time0))
        {
          found[i] = true;

          if (res.distance < worst_dist)
            worst_dist = res.distance;

          const double weight = 100.0 * sco::pospart(grad.margin + margin_buffer_ - res.distance);
          total_weight[i] += weight;
          dist_grad[i] += (weight * grad.gradients[i].gradient);
        }
      }
    }

    exprs_margin.push_back(margin);
    exprs_coeff.push_back(coeff);
    if (!found[0] && !found[1])
    {
      exprs.emplace_back(0);
    }
    else
    {
      sco::AffExpr dist(worst_dist);
      for (std::size_t i = 0; i < 2; ++i)
      {
        if (found[i])
        {
          assert(std::abs(total_weight[i]) > 1e-8);
          dist_grad[i] *= (1.0 / total_weight[i]);
          sco::exprInc(dist, sco::varDot(dist_grad[i], vars));
          sco::exprInc(dist, -dist_grad[i].dot(dofvals));
        }
      }
      exprs.push_back(dist);
    }
  }
}

void CollisionEvaluator::CollisionsToDistanceExpressionsContinuousW(
    sco::AffExprVector& exprs,
    std::vector<double>& exprs_margin,
    std::vector<double>& exprs_coeff,
    const tesseract_collision::ContactResultMap& dist_results,
    const sco::VarVector& vars0,
    const sco::VarVector& vars1,
    const DblVec& x,
    bool isTimestep1)
{
  const Eigen::VectorXd dofvals0 = sco::getVec(x, vars0);
  const Eigen::VectorXd dofvals1 = sco::getVec(x, vars1);
  const Eigen::VectorXd dofvalst = Eigen::VectorXd::Zero(dofvals0.size());

  // All collision data is in world coordinate system. This provides the
  // transform for converting data between world frame and manipulator
  // frame.

  exprs.clear();
  exprs_margin.clear();
  exprs_coeff.clear();
  exprs.reserve(dist_results.count());
  exprs_margin.reserve(dist_results.count());
  exprs_coeff.reserve(dist_results.count());
  for (const auto& pair : dist_results)
  {
    if (pair.second.empty())
      continue;

    double worst_dist{ std::numeric_limits<double>::max() };
    double total_weight[2];
    bool found[2];
    Eigen::VectorXd dist_grad[2];

    total_weight[0] = 0;
    total_weight[1] = 0;

    found[0] = false;
    found[1] = false;

    dist_grad[0] = Eigen::VectorXd::Zero(manip_->numJoints());
    dist_grad[1] = Eigen::VectorXd::Zero(manip_->numJoints());

    // Contains the contact distance threshold and coefficient for the given link pair
    const double margin = margin_data_.getCollisionMargin(pair.first.first, pair.first.second);
    const double coeff = coeff_data_.getCollisionCoeff(pair.first.first, pair.first.second);
    for (const auto& res : pair.second)
    {
      GradientResults grad = GetGradient(dofvals0, dofvals1, res, margin, coeff, isTimestep1);

      for (std::size_t i = 0; i < 2; ++i)
      {
        // Changing the start state does not have an affect if the collision is at the end state so do not process
        // Changing the end state does not have an affect if the collision is at the start state so do not process
        if (grad.gradients[i].has_gradient &&
            !(!isTimestep1 && res.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time1) &&
            !(isTimestep1 && res.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time0))
        {
          assert(res.cc_type[i] != tesseract_collision::ContinuousCollisionType::CCType_None);
          assert(res.cc_time[i] >= 0.0 && res.cc_time[i] <= 1.0);

          found[i] = true;

          if (res.distance < worst_dist)
            worst_dist = res.distance;

          const double weight = 100.0 * sco::pospart(grad.margin + margin_buffer_ - res.distance);
          total_weight[i] += weight;
          dist_grad[i] += (weight * grad.gradients[i].gradient);
        }
      }
    }

    exprs_margin.push_back(margin);
    exprs_coeff.push_back(coeff);
    if (!found[0] && !found[1])
    {
      exprs.emplace_back(0);
    }
    else
    {
      sco::AffExpr dist(worst_dist);
      for (std::size_t i = 0; i < 2; ++i)
      {
        if (found[i])
        {
          assert(std::abs(total_weight[i]) > 1e-8);
          dist_grad[i] *= (1.0 / total_weight[i]);

          if (i == 0)
          {
            sco::exprInc(dist, sco::varDot(dist_grad[i], vars0));
            sco::exprInc(dist, -dist_grad[i].dot(dofvals0));
          }
          else
          {
            sco::exprInc(dist, sco::varDot(dist_grad[i], vars1));
            sco::exprInc(dist, -dist_grad[i].dot(dofvals1));
          }
        }
      }
      exprs.push_back(dist);
    }
  }
}

CollisionEvaluator::CollisionEvaluator(tesseract_kinematics::JointGroup::ConstPtr manip,
                                       tesseract_environment::Environment::ConstPtr env,
                                       bool dynamic_environment)
  : manip_(std::move(manip)), env_(std::move(env)), dynamic_environment_(dynamic_environment)
{
  manip_active_link_names_ = manip_->getActiveLinkNames();

  // If the environment is not expected to change, then the cloned state solver may be used each time.
  if (dynamic_environment_)
  {
    get_state_fn_ = [&](const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return env_->getState(manip_->getJointNames(), joint_values).link_transforms;
    };
    env_active_link_names_ = env_->getActiveLinkNames();

    std::sort(manip_active_link_names_.begin(), manip_active_link_names_.end());
    std::sort(env_active_link_names_.begin(), env_active_link_names_.end());
    std::set_difference(env_active_link_names_.begin(),
                        env_active_link_names_.end(),
                        manip_active_link_names_.begin(),
                        manip_active_link_names_.end(),
                        std::inserter(diff_active_link_names_, diff_active_link_names_.begin()));
  }
  else
  {
    get_state_fn_ = [&](const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return manip_->calcFwdKin(joint_values);
    };
    env_active_link_names_ = manip_->getActiveLinkNames();
  }
}

void CollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  const ContactResultVectorConstPtr dist_results = GetContactResultVectorCached(x);
  CollisionsToDistances(*dist_results, dists);
}

inline std::size_t hash(const DblVec& x) { return boost::hash_range(x.begin(), x.end()); }
ContactResultVectorConstPtr CollisionEvaluator::GetContactResultVectorCached(const DblVec& x)
{
  const std::pair<ContactResultMapConstPtr, ContactResultVectorConstPtr> pair = GetContactResultCached(x);
  return pair.second;
}

ContactResultMapConstPtr CollisionEvaluator::GetContactResultMapCached(const DblVec& x)
{
  const std::pair<ContactResultMapConstPtr, ContactResultVectorConstPtr> pair = GetContactResultCached(x);
  return pair.first;
}

std::pair<ContactResultMapConstPtr, ContactResultVectorConstPtr>
CollisionEvaluator::GetContactResultCached(const DblVec& x)
{
  const std::size_t key = hash(sco::getDblVec(x, GetVars()));
  auto* it = m_cache.get(key);
  if (it != nullptr)
  {
    LOG_DEBUG("using cached collision check\n")
    return *it;
  }

  LOG_DEBUG("not using cached collision check\n")

  /**
   * We tried different combinations of how to store the dist_map and also passing in a sub_dist_map to CalcCollisions,
   * using member variable and thread_local. Just making the dist_map thread_local and making a copy for sub_dist_result
   * in CalcCollisions had the most significant impact on peformance and memory.
   */
  thread_local tesseract_collision::ContactResultMap dist_map;
  dist_map.clear();

  CalcCollisions(x, dist_map);

  auto dist_map_ptr = std::make_shared<tesseract_collision::ContactResultMap>(dist_map);
  auto dist_vec_ptr = std::make_shared<ContactResultVectorWrapper>();
  dist_map_ptr->flattenWrapperResults(*dist_vec_ptr);

  auto pair = std::make_pair(dist_map_ptr, dist_vec_ptr);
  m_cache.put(key, pair);
  return pair;
}

void CollisionEvaluator::CalcDistExpressionsStartFree(const DblVec& x,
                                                      sco::AffExprVector& exprs,
                                                      std::vector<double>& exprs_margin,
                                                      std::vector<double>& exprs_coeff)
{
  const ContactResultVectorConstPtr dist_vec = GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  sco::AffExprVector exprs0;
  CollisionsToDistanceExpressions(exprs0, exprs_margin, exprs_coeff, dist_results, vars0_, x, false);

  exprs.resize(exprs0.size());
  assert(exprs0.size() == dist_results.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    exprs[i] = sco::AffExpr(dist_results[i].get().distance);
    sco::exprInc(exprs[i], exprs0[i]);
    exprs[i] = sco::cleanupAff(exprs[i]);
  }
}

void CollisionEvaluator::CalcDistExpressionsEndFree(const DblVec& x,
                                                    sco::AffExprVector& exprs,
                                                    std::vector<double>& exprs_margin,
                                                    std::vector<double>& exprs_coeff)
{
  const ContactResultVectorConstPtr dist_vec = GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  sco::AffExprVector exprs1;
  CollisionsToDistanceExpressions(exprs1, exprs_margin, exprs_coeff, dist_results, vars1_, x, true);

  exprs.resize(exprs1.size());
  assert(exprs1.size() == dist_results.size());
  for (std::size_t i = 0; i < exprs1.size(); ++i)
  {
    exprs[i] = sco::AffExpr(dist_results[i].get().distance);
    sco::exprInc(exprs[i], exprs1[i]);
    exprs[i] = sco::cleanupAff(exprs[i]);
  }
}

void CollisionEvaluator::CalcDistExpressionsBothFree(const DblVec& x,
                                                     sco::AffExprVector& exprs,
                                                     std::vector<double>& exprs_margin,
                                                     std::vector<double>& exprs_coeff)
{
  const ContactResultVectorConstPtr dist_vec = GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  sco::AffExprVector exprs0, exprs1;
  std::vector<double> exprs_margin0, exprs_margin1;
  std::vector<double> exprs_coeff0, exprs_coeff1;
  CollisionsToDistanceExpressions(exprs0, exprs_margin0, exprs_coeff0, dist_results, vars0_, x, false);
  CollisionsToDistanceExpressions(exprs1, exprs_margin1, exprs_coeff1, dist_results, vars1_, x, true);

  exprs_margin = exprs_margin0;
  exprs_coeff = exprs_coeff0;
  exprs.resize(exprs0.size());
  assert(exprs0.size() == dist_results.size());
  assert(exprs0.size() == exprs1.size());
  assert(exprs0.size() == exprs_margin0.size());
  assert(exprs0.size() == exprs_coeff0.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    exprs[i] = sco::AffExpr(dist_results[i].get().distance);
    sco::exprInc(exprs[i], exprs0[i]);
    sco::exprInc(exprs[i], exprs1[i]);
    exprs[i] = sco::cleanupAff(exprs[i]);
  }
}

void CollisionEvaluator::CalcDistExpressionsStartFreeW(const DblVec& x,
                                                       sco::AffExprVector& exprs,
                                                       std::vector<double>& exprs_margin,
                                                       std::vector<double>& exprs_coeff)
{
  const ContactResultMapConstPtr dist_results = GetContactResultMapCached(x);

  sco::AffExprVector exprs0;
  CollisionsToDistanceExpressionsContinuousW(
      exprs0, exprs_margin, exprs_coeff, *dist_results, vars0_, vars1_, x, false);

  exprs.resize(exprs0.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    exprs[i] = sco::AffExpr(0);
    sco::exprInc(exprs[i], exprs0[i]);
    exprs[i] = sco::cleanupAff(exprs[i]);
  }
}

void CollisionEvaluator::CalcDistExpressionsEndFreeW(const DblVec& x,
                                                     sco::AffExprVector& exprs,
                                                     std::vector<double>& exprs_margin,
                                                     std::vector<double>& exprs_coeff)
{
  const ContactResultMapConstPtr dist_results = GetContactResultMapCached(x);

  sco::AffExprVector exprs1;
  CollisionsToDistanceExpressionsContinuousW(exprs1, exprs_margin, exprs_coeff, *dist_results, vars0_, vars1_, x, true);

  exprs.resize(exprs1.size());
  for (std::size_t i = 0; i < exprs1.size(); ++i)
  {
    exprs[i] = sco::AffExpr(0);
    sco::exprInc(exprs[i], exprs1[i]);
    exprs[i] = sco::cleanupAff(exprs[i]);
  }
}

void CollisionEvaluator::CalcDistExpressionsBothFreeW(const DblVec& x,
                                                      sco::AffExprVector& exprs,
                                                      std::vector<double>& exprs_margin,
                                                      std::vector<double>& exprs_coeff)
{
  const ContactResultMapConstPtr dist_results = GetContactResultMapCached(x);

  sco::AffExprVector exprs0, exprs1;
  std::vector<double> exprs_margin0, exprs_margin1;
  std::vector<double> exprs_coeff0, exprs_coeff1;
  CollisionsToDistanceExpressionsContinuousW(
      exprs0, exprs_margin0, exprs_coeff0, *dist_results, vars0_, vars1_, x, false);
  CollisionsToDistanceExpressionsContinuousW(
      exprs1, exprs_margin1, exprs_coeff1, *dist_results, vars0_, vars1_, x, true);

  exprs_margin = exprs_margin0;
  exprs_coeff = exprs_coeff0;
  exprs.resize(exprs0.size());
  assert(exprs0.size() == exprs1.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    exprs[i] = sco::AffExpr(0);
    sco::exprInc(exprs[i], exprs0[i]);
    sco::exprInc(exprs[i], exprs1[i]);
    exprs[i] = sco::cleanupAff(exprs[i]);
  }
}

void CollisionEvaluator::CalcDistExpressionsSingleTimeStep(const DblVec& x,
                                                           sco::AffExprVector& exprs,
                                                           std::vector<double>& exprs_margin,
                                                           std::vector<double>& exprs_coeff)
{
  const ContactResultVectorConstPtr dist_vec = GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  CollisionsToDistanceExpressions(exprs, exprs_margin, exprs_coeff, dist_results, vars0_, x, false);
  assert(dist_results.size() == exprs.size());

  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    sco::exprInc(exprs[i], dist_results[i].get().distance);
    sco::cleanupAff(exprs[i]);
  }
}

void CollisionEvaluator::CalcDistExpressionsSingleTimeStepW(const DblVec& x,
                                                            sco::AffExprVector& exprs,
                                                            std::vector<double>& exprs_margin,
                                                            std::vector<double>& exprs_coeff)
{
  const ContactResultMapConstPtr dist_results = GetContactResultMapCached(x);
  CollisionsToDistanceExpressionsW(exprs, exprs_margin, exprs_coeff, *dist_results, vars0_, x, false);
  assert(dist_results->size() == exprs.size());

  for (auto& expr : exprs)
    expr = sco::cleanupAff(expr);
}

void CollisionEvaluator::removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results,
                                                     double margin) const
{
  auto end = std::remove_if(
      contact_results.begin(), contact_results.end(), [this, margin](const tesseract_collision::ContactResult& r) {
        switch (evaluator_type_)
        {
          case CollisionExpressionEvaluatorType::START_FREE_END_FREE:
          {
            break;
          }
          case CollisionExpressionEvaluatorType::START_FIXED_END_FREE:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            break;
          }
          case CollisionExpressionEvaluatorType::START_FREE_END_FIXED:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            break;
          }
          case CollisionExpressionEvaluatorType::START_FREE_END_FREE_WEIGHTED_SUM:
          {
            break;
          }
          case CollisionExpressionEvaluatorType::START_FIXED_END_FREE_WEIGHTED_SUM:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            break;
          }
          case CollisionExpressionEvaluatorType::START_FREE_END_FIXED_WEIGHTED_SUM:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            break;
          }
          default:
          {
            PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for "
                            "CollisionEvaluator::removeInvalidContactResults!");
          }
        }

        return (r.distance >= (margin + margin_buffer_));
      });

  contact_results.erase(end, contact_results.end());
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(
    std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
    std::shared_ptr<const tesseract_environment::Environment> env,
    const trajopt_common::TrajOptCollisionConfig& collision_config,
    sco::VarVector vars,
    CollisionExpressionEvaluatorType type,
    bool dynamic_environment)
  : CollisionEvaluator(std::move(manip), std::move(env), dynamic_environment)
{
  collision_check_config_ = collision_config.collision_check_config;
  if (collision_check_config_.type != tesseract_collision::CollisionEvaluatorType::DISCRETE)
    throw std::runtime_error("SingleTimestepCollisionEvaluator, should be configured with DISCRETE");

  vars0_ = std::move(vars);
  evaluator_type_ = type;

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(manip_->getActiveLinkNames());
  contact_manager_->applyContactManagerConfig(collision_config.contact_manager_config);
  // Must make a copy after applying the config but before we increment the margin data
  margin_data_ = contact_manager_->getCollisionMarginData();
  coeff_data_ = collision_config.collision_coeff_data;
  margin_buffer_ = collision_config.collision_margin_buffer;

  // Increase the default by the buffer
  contact_manager_->incrementCollisionMargin(margin_buffer_);

  switch (evaluator_type_)
  {
    case CollisionExpressionEvaluatorType::SINGLE_TIME_STEP:
    {
      fn_ = std::bind(&SingleTimestepCollisionEvaluator::CalcDistExpressionsSingleTimeStep,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::SINGLE_TIME_STEP_WEIGHTED_SUM:
    {
      fn_ = std::bind(&SingleTimestepCollisionEvaluator::CalcDistExpressionsSingleTimeStepW,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    default:
    {
      PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for SingleTimestepCollisionEvaluator!");
    }
  }
}

void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x,
                                                      tesseract_collision::ContactResultMap& dist_results)
{
  const Eigen::VectorXd joint_vals = sco::getVec(x, vars0_);
  CalcCollisions(joint_vals, dist_results);
}

void SingleTimestepCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                                                      tesseract_collision::ContactResultMap& dist_results)
{
  tesseract_common::TransformMap state = get_state_fn_(dof_vals);

  // If not empty then there are links that are not part of the kinematics object that can move (dynamic environment)
  for (const auto& link_name : diff_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);

  for (const auto& link_name : manip_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);

  contact_manager_->contactTest(dist_results, collision_check_config_.contact_request);

  const auto& zero_coeff_pairs = coeff_data_.getPairsWithZeroCoeff();
  auto filter = [this, &zero_coeff_pairs](tesseract_collision::ContactResultMap::PairType& pair) {
    // Remove pairs with zero coeffs
    if (zero_coeff_pairs.find(pair.first) != zero_coeff_pairs.end())
    {
      pair.second.clear();
      return;
    }

    // Contains the contact distance threshold and coefficient for the given link pair
    const double margin = margin_data_.getCollisionMargin(pair.first.first, pair.first.second);

    auto end = std::remove_if(
        pair.second.begin(), pair.second.end(), [margin, this](const tesseract_collision::ContactResult& r) {
          return (r.distance >= (margin + margin_buffer_));
        });
    pair.second.erase(end, pair.second.end());
  };

  dist_results.filter(filter);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x,
                                                           sco::AffExprVector& exprs,
                                                           std::vector<double>& exprs_margin,
                                                           std::vector<double>& exprs_coeff)
{
  fn_(x, exprs, exprs_margin, exprs_coeff);
}

void SingleTimestepCollisionEvaluator::Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter,
                                            const DblVec& x)
{
  const ContactResultVectorConstPtr dist_vec = GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  const Eigen::VectorXd dofvals = sco::getVec(x, vars0_);

  tesseract_collision::ContactResultVector dist_results_copy(dist_results.size());
  Eigen::VectorXd collision_margins(dist_results.size());
  for (auto i = 0U; i < dist_results.size(); ++i)
  {
    const tesseract_collision::ContactResult& res = dist_results[i].get();
    dist_results_copy[i] = res;

    // Contains the contact distance threshold for the given link pair
    collision_margins[i] = margin_data_.getCollisionMargin(res.link_names[0], res.link_names[1]);

    if (manip_->isActiveLinkName(res.link_names[0]))
    {
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvals, res.link_names[0], res.nearest_points[0]);

      const Eigen::VectorXd dist_grad = -res.normal.transpose() * jac.topRows(3);

      const Eigen::Isometry3d pose2 = manip_->calcFwdKin(dofvals + dist_grad)[res.link_names[0]];

      tesseract_visualization::ArrowMarker am(res.nearest_points[0], pose2 * res.nearest_points[0]);
      am.material = std::make_shared<tesseract_scene_graph::Material>("collision_error_material");
      am.material->color << 1, 1, 1, 1;
      plotter->plotMarker(am);
    }
  }

  auto margin_fn = [this](const std::string& link1, const std::string& link2) {
    return margin_data_.getCollisionMargin(link1, link2);
  };

  const tesseract_visualization::ContactResultsMarker cm(manip_->getActiveLinkNames(), dist_results_copy, margin_fn);
  plotter->plotMarker(cm);
}

////////////////////////////////////////

DiscreteCollisionEvaluator::DiscreteCollisionEvaluator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                                       std::shared_ptr<const tesseract_environment::Environment> env,
                                                       const trajopt_common::TrajOptCollisionConfig& collision_config,
                                                       sco::VarVector vars0,
                                                       sco::VarVector vars1,
                                                       CollisionExpressionEvaluatorType type)
  : CollisionEvaluator(std::move(manip), std::move(env))
{
  collision_check_config_ = collision_config.collision_check_config;
  if (collision_check_config_.type != tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
    std::runtime_error("DiscreteCollisionEvaluator, should be configured with LVS_DISCRETE");

  vars0_ = std::move(vars0);
  vars1_ = std::move(vars1);
  evaluator_type_ = type;

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(manip_->getActiveLinkNames());
  contact_manager_->applyContactManagerConfig(collision_config.contact_manager_config);
  // Must make a copy after applying the config but before we increment the margin data
  margin_data_ = contact_manager_->getCollisionMarginData();
  coeff_data_ = collision_config.collision_coeff_data;
  margin_buffer_ = collision_config.collision_margin_buffer;

  // Increase the default by the buffer
  contact_manager_->incrementCollisionMargin(margin_buffer_);

  switch (evaluator_type_)
  {
    case CollisionExpressionEvaluatorType::START_FREE_END_FREE:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = false;
      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsBothFree,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE:
    {
      vars0_fixed_ = true;
      vars1_fixed_ = false;
      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsEndFree,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = true;
      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsStartFree,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FREE_END_FREE_WEIGHTED_SUM:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = false;
      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsBothFreeW,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE_WEIGHTED_SUM:
    {
      vars0_fixed_ = true;
      vars1_fixed_ = false;
      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsEndFreeW,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED_WEIGHTED_SUM:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = true;
      fn_ = std::bind(&DiscreteCollisionEvaluator::CalcDistExpressionsStartFreeW,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    default:
    {
      PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for DiscreteCollisionEvaluator!");
    }
  }
}

void DiscreteCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract_collision::ContactResultMap& dist_results)
{
  const Eigen::VectorXd s0 = sco::getVec(x, vars0_);
  const Eigen::VectorXd s1 = sco::getVec(x, vars1_);
  CalcCollisions(s0, s1, dist_results);
}

void DiscreteCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                                tesseract_collision::ContactResultMap& dist_results)
{
  assert(dist_results.empty());
  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
  // the collision checking is broken up into multiple casted collision checks such that each check is less then
  // the longest valid segment length.
  const double dist = (dof_vals1 - dof_vals0).norm();

  // If not empty then there are links that are not part of the kinematics object that can move (dynamic environment)
  if (!diff_active_link_names_.empty())
  {
    tesseract_common::TransformMap state = get_state_fn_(dof_vals0);
    for (const auto& link_name : diff_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);
  }

  long cnt = 2;
  if (dist > collision_check_config_.longest_valid_segment_length)
  {
    // Calculate the number state to interpolate
    cnt = static_cast<long>(std::ceil(dist / collision_check_config_.longest_valid_segment_length)) + 1;
  }

  // Create interpolated trajectory between two states that satisfies the longest valid segment length.
  tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
  for (long i = 0; i < dof_vals0.size(); ++i)
    subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

  // Define Filter
  const auto& zero_coeff_pairs = coeff_data_.getPairsWithZeroCoeff();
  auto filter = [this, &zero_coeff_pairs](tesseract_collision::ContactResultMap::PairType& pair) {
    // Remove pairs with zero coeffs
    if (zero_coeff_pairs.find(pair.first) != zero_coeff_pairs.end())
    {
      pair.second.clear();
      return;
    }

    // Contains the contact distance threshold and coefficient for the given link pair
    const double margin = margin_data_.getCollisionMargin(pair.first.first, pair.first.second);

    // Don't include contacts at the fixed state
#ifndef NDEBUG
    tesseract_collision::ContactResultVector copy{ pair.second };
#endif

    trajopt_common::removeInvalidContactResults(pair.second, margin, margin_buffer_, vars0_fixed_, vars1_fixed_);

#ifndef NDEBUG
    removeInvalidContactResults(copy, margin);
    assert(copy.size() == pair.second.size());
#endif
  };

  // Perform casted collision checking for sub trajectory and store results in contacts_vector
  const tesseract_common::TrajArray::Index last_state_idx{ subtraj.rows() - 1 };
  const double dt = 1.0 / double(last_state_idx);
  /** @todo require this to be passed in to reduce memory allocations */
  tesseract_collision::ContactResultMap contacts{ dist_results };
  for (int i = 0; i < subtraj.rows(); ++i)
  {
    tesseract_common::TransformMap state0 = get_state_fn_(subtraj.row(i));

    for (const auto& link_name : manip_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state0[link_name]);

    contact_manager_->contactTest(contacts, collision_check_config_.contact_request);

    if (!contacts.empty())
    {
      dist_results.addInterpolatedCollisionResults(
          contacts, i, last_state_idx, manip_active_link_names_, dt, true, filter);
    }
    contacts.clear();
  }
}

void DiscreteCollisionEvaluator::CalcDistExpressions(const DblVec& x,
                                                     sco::AffExprVector& exprs,
                                                     std::vector<double>& exprs_margin,
                                                     std::vector<double>& exprs_coeff)
{
  fn_(x, exprs, exprs_margin, exprs_coeff);
}

void DiscreteCollisionEvaluator::Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter,
                                      const DblVec& x)
{
  const ContactResultVectorConstPtr dist_vec = GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  const Eigen::VectorXd dofvals0 = sco::getVec(x, vars0_);
  const Eigen::VectorXd dofvals1 = sco::getVec(x, vars1_);

  const tesseract_common::TransformMap state0 = manip_->calcFwdKin(dofvals0);
  const tesseract_common::TransformMap state1 = manip_->calcFwdKin(dofvals1);

  tesseract_collision::ContactResultVector dist_results_copy(dist_results.size());
  Eigen::VectorXd collision_margins(dist_results.size());
  for (auto i = 0U; i < dist_results.size(); ++i)
  {
    const tesseract_collision::ContactResult& res = dist_results[i].get();
    dist_results_copy[i] = res;

    // Contains the contact distance threshold for the given link pair
    collision_margins[i] = margin_data_.getCollisionMargin(res.link_names[0], res.link_names[1]);

    if (manip_->isActiveLinkName(res.link_names[0]))
    {
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvals0, res.link_names[0], res.nearest_points_local[0]);

      // Eigen::MatrixXd jac_test;
      // jac_test.resize(6, manip_->numJoints());
      // tesseract_kinematics::numericalJacobian(jac_test, *manip_, dofvals0, res.link_names[0],
      // res.nearest_points_local[0]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);

      const Eigen::VectorXd dist_grad = -res.normal.transpose() * jac.topRows(3);

      const Eigen::Isometry3d pose2 = manip_->calcFwdKin(dofvals0 + dist_grad)[res.link_names[0]];

      tesseract_visualization::ArrowMarker am(res.nearest_points[0], pose2 * res.nearest_points_local[0]);
      am.material = std::make_shared<tesseract_scene_graph::Material>("collision_error_material");
      am.material->color << 1, 1, 1, 1;
      plotter->plotMarker(am);
    }

    if (manip_->isActiveLinkName(res.link_names[1]))
    {
      // Calculate Jacobian
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvals0, res.link_names[1], res.nearest_points_local[1]);

      // Eigen::MatrixXd jac_test;
      // jac_test.resize(6, manip_->numJoints());
      // tesseract_kinematics::numericalJacobian(jac_test, *manip_, dofvals0, res.link_names[1],
      // res.nearest_points_local[1]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);

      const Eigen::VectorXd dist_grad = res.normal.transpose() * jac.topRows(3);

      const Eigen::Isometry3d pose2 = manip_->calcFwdKin(dofvals0 + dist_grad)[res.link_names[1]];

      tesseract_visualization::ArrowMarker am(res.nearest_points[1], pose2 * res.nearest_points_local[1]);
      am.material = std::make_shared<tesseract_scene_graph::Material>("collision_error_material");
      am.material->color << 1, 1, 1, 1;
      plotter->plotMarker(am);
    }
  }

  auto margin_fn = [this](const std::string& link1, const std::string& link2) {
    return margin_data_.getCollisionMargin(link1, link2);
  };

  const tesseract_visualization::ContactResultsMarker cm(manip_->getActiveLinkNames(), dist_results_copy, margin_fn);
  plotter->plotMarker(cm);
}

sco::VarVector DiscreteCollisionEvaluator::GetVars() { return trajopt_common::concat(vars0_, vars1_); }

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                               std::shared_ptr<const tesseract_environment::Environment> env,
                                               const trajopt_common::TrajOptCollisionConfig& collision_config,
                                               sco::VarVector vars0,
                                               sco::VarVector vars1,
                                               CollisionExpressionEvaluatorType type)
  : CollisionEvaluator(std::move(manip), std::move(env))
{
  collision_check_config_ = collision_config.collision_check_config;
  if (collision_check_config_.type != tesseract_collision::CollisionEvaluatorType::CONTINUOUS &&
      collision_check_config_.type != tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    throw std::runtime_error("CastCollisionEvaluator, should be configured with CONTINUOUS or LVS_CONTINUOUS");
  }

  vars0_ = std::move(vars0);
  vars1_ = std::move(vars1);
  evaluator_type_ = type;

  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(manip_->getActiveLinkNames());
  contact_manager_->applyContactManagerConfig(collision_config.contact_manager_config);
  // Must make a copy after applying the config but before we increment the margin data
  margin_data_ = contact_manager_->getCollisionMarginData();
  coeff_data_ = collision_config.collision_coeff_data;
  margin_buffer_ = collision_config.collision_margin_buffer;

  // Increase the default by the buffer
  contact_manager_->incrementCollisionMargin(margin_buffer_);

  switch (evaluator_type_)
  {
    case CollisionExpressionEvaluatorType::START_FREE_END_FREE:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = false;
      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsBothFree,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE:
    {
      vars0_fixed_ = true;
      vars1_fixed_ = false;
      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsEndFree,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = true;
      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsStartFree,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FREE_END_FREE_WEIGHTED_SUM:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = false;
      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsBothFreeW,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FIXED_END_FREE_WEIGHTED_SUM:
    {
      vars0_fixed_ = true;
      vars1_fixed_ = false;
      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsEndFreeW,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    case CollisionExpressionEvaluatorType::START_FREE_END_FIXED_WEIGHTED_SUM:
    {
      vars0_fixed_ = false;
      vars1_fixed_ = true;
      fn_ = std::bind(&CastCollisionEvaluator::CalcDistExpressionsStartFreeW,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3,
                      std::placeholders::_4);
      break;
    }
    default:
    {
      PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for CastCollisionEvaluator!");
    }
  }
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, tesseract_collision::ContactResultMap& dist_results)
{
  const Eigen::VectorXd s0 = sco::getVec(x, vars0_);
  const Eigen::VectorXd s1 = sco::getVec(x, vars1_);
  CalcCollisions(s0, s1, dist_results);
}

void CastCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                            tesseract_collision::ContactResultMap& dist_results)
{
  assert(dist_results.empty());
  // The first step is to see if the distance between two states is larger than the longest valid segment. If larger
  // the collision checking is broken up into multiple casted collision checks such that each check is less then
  // the longest valid segment length.
  const double dist = (dof_vals1 - dof_vals0).norm();

  // If not empty then there are links that are not part of the kinematics object that can move (dynamic environment)
  if (!diff_active_link_names_.empty())
  {
    tesseract_common::TransformMap state = get_state_fn_(dof_vals0);
    for (const auto& link_name : diff_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);
  }

  // Define Filter
  const auto& zero_coeff_pairs = coeff_data_.getPairsWithZeroCoeff();
  auto filter = [this, &zero_coeff_pairs](tesseract_collision::ContactResultMap::PairType& pair) {
    // Remove pairs with zero coeffs
    if (zero_coeff_pairs.find(pair.first) != zero_coeff_pairs.end())
    {
      pair.second.clear();
      return;
    }

    // Contains the contact distance threshold and coefficient for the given link pair
    const double margin = margin_data_.getCollisionMargin(pair.first.first, pair.first.second);

    // Don't include contacts at the fixed state
#ifndef NDEBUG
    tesseract_collision::ContactResultVector copy{ pair.second };
#endif

    trajopt_common::removeInvalidContactResults(pair.second, margin, margin_buffer_, vars0_fixed_, vars1_fixed_);

#ifndef NDEBUG
    removeInvalidContactResults(copy, margin);
    assert(copy.size() == pair.second.size());
#endif
  };

  if (dist > collision_check_config_.longest_valid_segment_length)
  {
    // Calculate the number state to interpolate
    auto cnt = static_cast<long>(std::ceil(dist / collision_check_config_.longest_valid_segment_length)) + 1;

    // Create interpolated trajectory between two states that satisfies the longest valid segment length.
    tesseract_common::TrajArray subtraj(cnt, dof_vals0.size());
    for (long i = 0; i < dof_vals0.size(); ++i)
      subtraj.col(i) = Eigen::VectorXd::LinSpaced(cnt, dof_vals0(i), dof_vals1(i));

    // Perform casted collision checking for sub trajectory and store results in contacts_vector
    /** @todo require this to be passed in to reduce memory allocations */
    tesseract_collision::ContactResultMap contacts{ dist_results };
    const tesseract_common::TrajArray::Index last_state_idx{ subtraj.rows() - 1 };
    const double dt = 1.0 / double(last_state_idx);
    for (int i = 0; i < subtraj.rows() - 1; ++i)
    {
      tesseract_common::TransformMap state0 = manip_->calcFwdKin(subtraj.row(i));
      tesseract_common::TransformMap state1 = manip_->calcFwdKin(subtraj.row(i + 1));

      for (const auto& link_name : manip_active_link_names_)
        contact_manager_->setCollisionObjectsTransform(link_name, state0[link_name], state1[link_name]);

      contact_manager_->contactTest(contacts, collision_check_config_.contact_request);

      if (!contacts.empty())
      {
        dist_results.addInterpolatedCollisionResults(
            contacts, i, last_state_idx, manip_active_link_names_, dt, false, filter);
      }
      contacts.clear();
    }
  }
  else
  {
    tesseract_common::TransformMap state0 = manip_->calcFwdKin(dof_vals0);
    tesseract_common::TransformMap state1 = manip_->calcFwdKin(dof_vals1);
    for (const auto& link_name : manip_active_link_names_)
      contact_manager_->setCollisionObjectsTransform(link_name, state0[link_name], state1[link_name]);

    contact_manager_->contactTest(dist_results, collision_check_config_.contact_request);

    dist_results.filter(filter);
  }
}

void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x,
                                                 sco::AffExprVector& exprs,
                                                 std::vector<double>& exprs_margin,
                                                 std::vector<double>& exprs_coeff)
{
  fn_(x, exprs, exprs_margin, exprs_coeff);
}

void CastCollisionEvaluator::Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter,
                                  const DblVec& x)
{
  const ContactResultVectorConstPtr dist_vec = GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  const Eigen::VectorXd dofvals = sco::getVec(x, vars0_);

  tesseract_collision::ContactResultVector dist_results_copy(dist_results.size());
  Eigen::VectorXd collision_margins(dist_results.size());
  for (auto i = 0U; i < dist_results.size(); ++i)
  {
    const tesseract_collision::ContactResult& res = dist_results[i].get();
    dist_results_copy[i] = res;

    // Contains the contact distance threshold and coefficient for the given link pair
    collision_margins[i] = margin_data_.getCollisionMargin(res.link_names[0], res.link_names[1]);

    if (manip_->isActiveLinkName(res.link_names[0]))
    {
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvals, res.link_names[0], res.nearest_points_local[0]);

      // Eigen::MatrixXd jac_test;
      // jac_test.resize(6, manip_->numJoints());
      // tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itA->link_name,
      // res.nearest_points_local[0]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);

      const Eigen::VectorXd dist_grad = -res.normal.transpose() * jac.topRows(3);

      const Eigen::Isometry3d pose2 = manip_->calcFwdKin(dofvals + dist_grad)[res.link_names[0]];
      tesseract_visualization::ArrowMarker am(res.nearest_points[0], pose2 * res.nearest_points_local[0]);
      am.material = std::make_shared<tesseract_scene_graph::Material>("collision_error_material");
      am.material->color << 1, 1, 1, 1;
      plotter->plotMarker(am);
    }

    if (manip_->isActiveLinkName(res.link_names[1]))
    {
      // Calculate Jacobian
      Eigen::MatrixXd jac = manip_->calcJacobian(dofvals, res.link_names[1], res.nearest_points_local[1]);

      // Eigen::MatrixXd jac_test;
      // jac_test.resize(6, manip_->numJoints());
      // tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, itB->link_name,
      // res.nearest_points_local[1]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true)

      const Eigen::VectorXd dist_grad = res.normal.transpose() * jac.topRows(3);
      const Eigen::Isometry3d pose2 = manip_->calcFwdKin(dofvals + dist_grad)[res.link_names[1]];
      tesseract_visualization::ArrowMarker am(res.nearest_points[1], pose2 * res.nearest_points_local[1]);
      am.material = std::make_shared<tesseract_scene_graph::Material>("collision_error_material");
      am.material->color << 1, 1, 1, 1;
      plotter->plotMarker(am);
    }
  }

  auto margin_fn = [this](const std::string& link1, const std::string& link2) {
    return margin_data_.getCollisionMargin(link1, link2);
  };

  const tesseract_visualization::ContactResultsMarker cm(manip_->getActiveLinkNames(), dist_results_copy, margin_fn);
  plotter->plotMarker(cm);
}

sco::VarVector CastCollisionEvaluator::GetVars() { return trajopt_common::concat(vars0_, vars1_); }

//////////////////////////////////////////

CollisionCost::CollisionCost(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                             std::shared_ptr<const tesseract_environment::Environment> env,
                             const trajopt_common::TrajOptCollisionConfig& collision_config,
                             sco::VarVector vars,
                             CollisionExpressionEvaluatorType type)
  : Cost("collision")
{
  m_calc = std::make_shared<SingleTimestepCollisionEvaluator>(
      std::move(manip), std::move(env), collision_config, std::move(vars), type);
}

CollisionCost::CollisionCost(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                             std::shared_ptr<const tesseract_environment::Environment> env,
                             const trajopt_common::TrajOptCollisionConfig& collision_config,
                             sco::VarVector vars0,
                             sco::VarVector vars1,
                             CollisionExpressionEvaluatorType type,
                             bool discrete)
{
  if (discrete)
  {
    name_ = "discrete_continuous_collision";
    m_calc = std::make_shared<DiscreteCollisionEvaluator>(
        std::move(manip), std::move(env), collision_config, std::move(vars0), std::move(vars1), type);
  }
  else
  {
    name_ = "cast_continuous_collision";
    m_calc = std::make_shared<CastCollisionEvaluator>(
        std::move(manip), std::move(env), collision_config, std::move(vars0), std::move(vars1), type);
  }
}

sco::ConvexObjective::Ptr CollisionCost::convex(const sco::DblVec& x, sco::Model* model)
{
  auto out = std::make_shared<sco::ConvexObjective>(model);
  /** @todo might make exprs and exprs_data thread local */
  sco::AffExprVector exprs;
  std::vector<double> exprs_margin;
  std::vector<double> exprs_coeff;

  m_calc->CalcDistExpressions(x, exprs, exprs_margin, exprs_coeff);
  assert(exprs.size() == exprs_margin.size());
  assert(exprs.size() == exprs_coeff.size());

  auto dist_results = m_calc->GetContactResultVectorCached(x);
  UNUSED(dist_results);

  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const sco::AffExpr viol = sco::exprSub(sco::AffExpr(exprs_margin[i]), exprs[i]);
    out->addHinge(viol, exprs_coeff[i]);
  }
  return out;
}

double CollisionCost::value(const sco::DblVec& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  auto dist_vec = m_calc->GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  double out = 0;
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    const tesseract_collision::ContactResult& res = dist_results[i].get();
    // Contains the contact distance threshold and coefficient for the given link pair
    const auto& margin_data = m_calc->getCollisionMarginData();
    const auto& coeff_data = m_calc->getCollisionCoeffData();
    const double margin = margin_data.getCollisionMargin(res.link_names[0], res.link_names[1]);
    const double coeff = coeff_data.getCollisionCoeff(res.link_names[0], res.link_names[1]);
    // The margin_buffer is not leverage here because we want the actual error
    out += sco::pospart(margin - dists[i]) * coeff;
  }
  return out;
}

void CollisionCost::Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter, const DblVec& x)
{
  m_calc->Plot(plotter, x);
}
// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                         std::shared_ptr<const tesseract_environment::Environment> env,
                                         const trajopt_common::TrajOptCollisionConfig& collision_config,
                                         sco::VarVector vars,
                                         CollisionExpressionEvaluatorType type)
{
  name_ = "collision";
  m_calc = std::make_shared<SingleTimestepCollisionEvaluator>(
      std::move(manip), std::move(env), collision_config, std::move(vars), type);
}

CollisionConstraint::CollisionConstraint(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                         std::shared_ptr<const tesseract_environment::Environment> env,
                                         const trajopt_common::TrajOptCollisionConfig& collision_config,
                                         sco::VarVector vars0,
                                         sco::VarVector vars1,
                                         CollisionExpressionEvaluatorType type,
                                         bool discrete)
{
  if (discrete)
  {
    name_ = "discrete_continuous_collision";
    m_calc = std::make_shared<DiscreteCollisionEvaluator>(
        std::move(manip), std::move(env), collision_config, std::move(vars0), std::move(vars1), type);
  }
  else
  {
    name_ = "cast_continuous_collision";
    m_calc = std::make_shared<CastCollisionEvaluator>(
        std::move(manip), std::move(env), collision_config, std::move(vars0), std::move(vars1), type);
  }
}

sco::ConvexConstraints::Ptr CollisionConstraint::convex(const sco::DblVec& x, sco::Model* model)
{
  auto out = std::make_shared<sco::ConvexConstraints>(model);
  /** @todo Try using thread_local */
  sco::AffExprVector exprs;
  std::vector<double> exprs_margin;
  std::vector<double> exprs_coeff;

  m_calc->CalcDistExpressions(x, exprs, exprs_margin, exprs_coeff);
  assert(exprs.size() == exprs_margin.size());
  assert(exprs.size() == exprs_coeff.size());

  auto dist_results = m_calc->GetContactResultVectorCached(x);
  UNUSED(dist_results);

  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const sco::AffExpr viol = sco::exprSub(sco::AffExpr(exprs_margin[i]), exprs[i]);
    out->addIneqCnt(sco::exprMult(viol, exprs_coeff[i]));
  }
  return out;
}

DblVec CollisionConstraint::value(const sco::DblVec& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  auto dist_vec = m_calc->GetContactResultVectorCached(x);
  const ContactResultVectorWrapper& dist_results = *dist_vec;

  DblVec out(dists.size());
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    const tesseract_collision::ContactResult& res = dist_results[i].get();
    // Contains the contact distance threshold and coefficient for the given link pair
    const auto& margin_data = m_calc->getCollisionMarginData();
    const auto& coeff_data = m_calc->getCollisionCoeffData();
    const double margin = margin_data.getCollisionMargin(res.link_names[0], res.link_names[1]);
    const double coeff = coeff_data.getCollisionCoeff(res.link_names[0], res.link_names[1]);
    // The margin_buffer is not leverage here because we want the actual error
    out[i] = sco::pospart(margin - dists[i]) * coeff;
  }
  return out;
}
}  // namespace trajopt
