#pragma once
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <trajopt/cache.hxx>
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>

namespace trajopt
{
enum class CollisionEvaluatorType
{
  START_FREE_END_FREE = 0,  /**< @brief Both start and end state variables are free to be adjusted */
  START_FREE_END_FIXED = 1, /**< @brief Only start state variables are free to be adjusted */
  START_FIXED_END_FREE = 2  /**< @brief Only end state variables are free to be adjusted */
};

struct CollisionEvaluator
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionEvaluator>;

  CollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                     tesseract_environment::Environment::ConstPtr env,
                     tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                     const Eigen::Isometry3d& world_to_base,
                     SafetyMarginData::ConstPtr safety_margin_data,
                     tesseract_collision::ContactTestType contact_test_type,
                     double longest_valid_segment_length)
    : manip_(std::move(manip))
    , env_(std::move(env))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , safety_margin_data_(std::move(safety_margin_data))
    , contact_test_type_(contact_test_type)
    , longest_valid_segment_length_(longest_valid_segment_length)
  {
  }
  virtual ~CollisionEvaluator() = default;
  CollisionEvaluator(const CollisionEvaluator&) = default;
  CollisionEvaluator& operator=(const CollisionEvaluator&) = default;
  CollisionEvaluator(CollisionEvaluator&&) = default;
  CollisionEvaluator& operator=(CollisionEvaluator&&) = default;

  virtual void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) = 0;
  void GetCollisionsCached(const DblVec& x, tesseract_collision::ContactResultVector&);
  virtual void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) = 0;
  virtual sco::VarVector GetVars() = 0;

  const SafetyMarginData::ConstPtr getSafetyMarginData() const { return safety_margin_data_; }
  Cache<size_t, tesseract_collision::ContactResultVector, 10> m_cache;

protected:
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  SafetyMarginData::ConstPtr safety_margin_data_;
  tesseract_collision::ContactTestType contact_test_type_;
  double longest_valid_segment_length_;

private:
  CollisionEvaluator() = default;
};

struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  SingleTimestepCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                   tesseract_environment::Environment::ConstPtr env,
                                   tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                   const Eigen::Isometry3d& world_to_base,
                                   SafetyMarginData::ConstPtr safety_margin_data,
                                   tesseract_collision::ContactTestType contact_test_type,
                                   sco::VarVector vars);
  /**
  @brief linearize all contact distances in terms of robot dofs
  ;
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance
  function
  */
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& dists) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return m_vars; }

private:
  sco::VarVector m_vars;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
  tesseract_environment::StateSolver::Ptr state_solver_;
};

struct CastCollisionEvaluator : public CollisionEvaluator
{
public:
  CastCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                         tesseract_environment::Environment::ConstPtr env,
                         tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                         const Eigen::Isometry3d& world_to_base,
                         SafetyMarginData::ConstPtr safety_margin_data,
                         tesseract_collision::ContactTestType contact_test_type,
                         double longest_valid_segment_length,
                         sco::VarVector vars0,
                         sco::VarVector vars1,
                         CollisionEvaluatorType type);
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  void CalcDists(const DblVec& x, DblVec& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return concat(vars0_, vars1_); }

private:
  sco::VarVector vars0_;
  sco::VarVector vars1_;
  CollisionEvaluatorType type_;
  tesseract_collision::ContinuousContactManager::Ptr contact_manager_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  std::function<void(const DblVec&, sco::AffExprVector&)> fn_;

  /**
   * @brief Calculate the distance expressions when the start is free but the end is fixed
   * @param x The current values
   * @param exprs The returned expression
   */
  void CalcDistExpressionsStartFree(const DblVec& x, sco::AffExprVector& exprs);

  /**
   * @brief Calculate the distance expressions when the end is free but the start is fixed
   * @param x The current values
   * @param exprs The returned expression
   */
  void CalcDistExpressionsEndFree(const DblVec& x, sco::AffExprVector& exprs);

  /**
   * @brief Calculate the distance expressions when the start and end is free
   * @param x The current values
   * @param exprs The returned expression
   */
  void CalcDistExpressionsBothFree(const DblVec& x, sco::AffExprVector& exprs);
};

class TRAJOPT_API CollisionCost : public sco::Cost, public Plotter
{
public:
  /* constructor for single timestep */
  CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                tesseract_environment::Environment::ConstPtr env,
                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                const Eigen::Isometry3d& world_to_base,
                SafetyMarginData::ConstPtr safety_margin_data,
                tesseract_collision::ContactTestType contact_test_type,
                sco::VarVector vars);
  /* constructor for cast cost */
  CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                tesseract_environment::Environment::ConstPtr env,
                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                const Eigen::Isometry3d& world_to_base,
                SafetyMarginData::ConstPtr safety_margin_data,
                tesseract_collision::ContactTestType contact_test_type,
                double longest_valid_segment_length,
                sco::VarVector vars0,
                sco::VarVector vars1,
                CollisionEvaluatorType type);
  sco::ConvexObjective::Ptr convex(const DblVec& x, sco::Model* model) override;
  double value(const DblVec&) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};

class TRAJOPT_API CollisionConstraint : public sco::IneqConstraint
{
public:
  /* constructor for single timestep */
  CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                      tesseract_environment::Environment::ConstPtr env,
                      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                      const Eigen::Isometry3d& world_to_base,
                      SafetyMarginData::ConstPtr safety_margin_data,
                      tesseract_collision::ContactTestType contact_test_type,
                      sco::VarVector vars);
  /* constructor for cast cost */
  CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                      tesseract_environment::Environment::ConstPtr env,
                      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                      const Eigen::Isometry3d& world_to_base,
                      SafetyMarginData::ConstPtr safety_margin_data,
                      tesseract_collision::ContactTestType contact_test_type,
                      double longest_valid_segment_length,
                      sco::VarVector vars0,
                      sco::VarVector vars1,
                      CollisionEvaluatorType type);
  sco::ConvexConstraints::Ptr convex(const DblVec& x, sco::Model* model) override;
  DblVec value(const DblVec&) override;
  void Plot(const DblVec& x);
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};
}  // namespace trajopt
