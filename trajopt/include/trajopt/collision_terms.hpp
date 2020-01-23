#pragma once
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <trajopt/cache.hxx>
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>

namespace trajopt
{
/**
 * @brief This contains the different types of expression evaluators used when performing continuous collision checking.
 */
enum class CollisionExpressionEvaluatorType
{
  START_FREE_END_FREE = 0,  /**< @brief Both start and end state variables are free to be adjusted */
  START_FREE_END_FIXED = 1, /**< @brief Only start state variables are free to be adjusted */
  START_FIXED_END_FREE = 2  /**< @brief Only end state variables are free to be adjusted */
};

/**
 * @brief Base class for collision evaluators containing function that are commonly used between them.
 *
 * This class also facilitates the caching of the contact results to prevent collision checking from being called
 * multiple times throughout the optimization.
 *
 */
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
                     double longest_valid_segment_length,
                     double safety_margin_buffer);
  virtual ~CollisionEvaluator() = default;
  CollisionEvaluator(const CollisionEvaluator&) = default;
  CollisionEvaluator& operator=(const CollisionEvaluator&) = default;
  CollisionEvaluator(CollisionEvaluator&&) = default;
  CollisionEvaluator& operator=(CollisionEvaluator&&) = default;

  /**
   * @brief This function calls GetCollisionsCached and stores the distances in a vector
   * @param x Optimizer variables
   * @param dists Returned distance values
   */
  virtual void CalcDists(const DblVec& x, DblVec& dists);

  /**
   * @brief Convert the contact information into an affine expression
   * @param x Optimizer variables
   * @param exprs Returned affine expression representation of the contact information
   */
  virtual void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) = 0;

  /**
   * @brief Given optimizer parameters calculate the collision results for this evaluator
   * @param x Optimizer variables
   * @param dist_results Contact results
   */
  virtual void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) = 0;

  /**
   * @brief This function checks to see if results are cached for input variable x. If not it calls CalcCollisions and
   * caches the results with x as the key.
   * @param x Optimizer variables
   */
  void GetCollisionsCached(const DblVec& x, tesseract_collision::ContactResultVector&);

  /**
   * @brief Plot the collision evaluator results
   * @param plotter Plotter
   * @param x Optimizer variables
   */
  virtual void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) = 0;

  /**
   * @brief Get the specific optimizer variables associated with this evaluator.
   * @return Evaluators variables
   */
  virtual sco::VarVector GetVars() = 0;

  /**
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  const SafetyMarginData::ConstPtr getSafetyMarginData() const { return safety_margin_data_; }
  Cache<size_t, tesseract_collision::ContactResultVector, 10> m_cache;

protected:
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  SafetyMarginData::ConstPtr safety_margin_data_;
  double safety_margin_buffer_;
  tesseract_collision::ContactTestType contact_test_type_;
  double longest_valid_segment_length_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  sco::VarVector vars0_;
  sco::VarVector vars1_;
  CollisionExpressionEvaluatorType evaluator_type_;

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
   * @brief Calculate the distance expressions when the start and end are free
   * @param x The current values
   * @param exprs The returned expression
   */
  void CalcDistExpressionsBothFree(const DblVec& x, sco::AffExprVector& exprs);

  /**
   * @brief This takes contacts results at each interpolated timestep and creates a single contact results map.
   * This also updates the cc_time and cc_type for the contact results
   * @param contacts_vector Contact results map at each interpolated timestep
   * @param contact_results The merged contact results map
   */
  void processInterpolatedCollisionResults(std::vector<tesseract_collision::ContactResultMap>& contacts_vector,
                                           tesseract_collision::ContactResultMap& contact_results) const;

  /**
   * @brief Remove any results that are invalid.
   * Invalid state are contacts that occur at fixed states or have distances outside the threshold.
   * @param contact_results Contact results vector to process.
   */
  void removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results,
                                   const Eigen::Vector2d& pair_data) const;

private:
  CollisionEvaluator() = default;
};

/**
 * @brief This collision evaluator only operates on a single state in the trajectory and does not check for collisions
 * between states.
 */
struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  SingleTimestepCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                   tesseract_environment::Environment::ConstPtr env,
                                   tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                   const Eigen::Isometry3d& world_to_base,
                                   SafetyMarginData::ConstPtr safety_margin_data,
                                   tesseract_collision::ContactTestType contact_test_type,
                                   sco::VarVector vars,
                                   double safety_margin_buffer);
  /**
  @brief linearize all contact distances in terms of robot dofs
  ;
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance
  function
  */
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return vars0_; }

private:
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * casted collision objects between to intermediate interpolated states.
 */
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
                         CollisionExpressionEvaluatorType type,
                         double safety_margin_buffer);
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return concat(vars0_, vars1_); }

private:
  tesseract_collision::ContinuousContactManager::Ptr contact_manager_;
  std::function<void(const DblVec&, sco::AffExprVector&)> fn_;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * descrete collision objects at each intermediate interpolated states.
 */
struct DiscreteCollisionEvaluator : public CollisionEvaluator
{
public:
  DiscreteCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                             tesseract_environment::Environment::ConstPtr env,
                             tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                             const Eigen::Isometry3d& world_to_base,
                             SafetyMarginData::ConstPtr safety_margin_data,
                             tesseract_collision::ContactTestType contact_test_type,
                             double longest_valid_segment_length,
                             sco::VarVector vars0,
                             sco::VarVector vars1,
                             CollisionExpressionEvaluatorType type,
                             double safety_margin_buffer);
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return concat(vars0_, vars1_); }

private:
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
  std::function<void(const DblVec&, sco::AffExprVector&)> fn_;
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
                sco::VarVector vars,
                double safety_margin_buffer);
  /* constructor for discrete continuous and cast continuous cost */
  CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                tesseract_environment::Environment::ConstPtr env,
                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                const Eigen::Isometry3d& world_to_base,
                SafetyMarginData::ConstPtr safety_margin_data,
                tesseract_collision::ContactTestType contact_test_type,
                double longest_valid_segment_length,
                sco::VarVector vars0,
                sco::VarVector vars1,
                CollisionExpressionEvaluatorType type,
                bool discrete,
                double safety_margin_buffer);
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
                      sco::VarVector vars,
                      double safety_margin_buffer);
  /* constructor for discrete continuous and cast continuous cost */
  CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                      tesseract_environment::Environment::ConstPtr env,
                      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                      const Eigen::Isometry3d& world_to_base,
                      SafetyMarginData::ConstPtr safety_margin_data,
                      tesseract_collision::ContactTestType contact_test_type,
                      double longest_valid_segment_length,
                      sco::VarVector vars0,
                      sco::VarVector vars1,
                      CollisionExpressionEvaluatorType type,
                      bool discrete,
                      double safety_margin_buffer);
  sco::ConvexConstraints::Ptr convex(const DblVec& x, sco::Model* model) override;
  DblVec value(const DblVec&) override;
  void Plot(const DblVec& x);
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};
}  // namespace trajopt
