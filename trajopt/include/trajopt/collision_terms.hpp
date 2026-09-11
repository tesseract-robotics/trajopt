#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <memory>
#include <Eigen/Core>

#include <tesseract/collision/fwd.h>
#include <tesseract/collision/types.h>
#include <tesseract/kinematics/fwd.h>
#include <tesseract/environment/fwd.h>
#include <tesseract/visualization/fwd.h>

#include <trajopt_common/collision_types.h>
#include <trajopt_sco/sco_common.hpp>

#include <trajopt/cache.hxx>
#include <trajopt/typedefs.hpp>

namespace trajopt
{
using ContactResultMapConstPtr = std::shared_ptr<const tesseract::collision::ContactResultMap>;
using ContactResultVectorWrapper = std::vector<std::reference_wrapper<const tesseract::collision::ContactResult>>;
using ContactResultVectorConstPtr = std::shared_ptr<const ContactResultVectorWrapper>;

/**
 * @brief This contains the different types of expression evaluators used when performing continuous collision checking.
 */
enum class CollisionExpressionEvaluatorType : std::uint8_t
{
  START_FREE_END_FREE = 0,  /**< @brief Both start and end state variables are free to be adjusted */
  START_FREE_END_FIXED = 1, /**< @brief Only start state variables are free to be adjusted */
  START_FIXED_END_FREE = 2, /**< @brief Only end state variables are free to be adjusted */
  SINGLE_TIME_STEP = 3,     /**< @brief Expressions are only calculated at a single time step */
};

/** @brief A data structure to contain a links gradient results */
struct LinkGradientResults
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Indicates if gradient results are available */
  bool has_gradient{ false };

  /** @brief Gradient Results */
  Eigen::VectorXd gradient;

  /** @brief Gradient Scale */
  double scale{ 1.0 };
};

/** @brief A data structure to contain a link pair gradient results */
struct GradientResults
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct the GradientResults
   * @param data The link pair safety margin data
   */
  GradientResults(double margin, double coeff) : margin(margin), coeff(coeff) {}

  /** @brief The gradient results data for LinkA and LinkB */
  std::array<LinkGradientResults, 2> gradients;

  /** @brief The link pair contact margin */
  double margin{ 0 };

  /** @brief The link pair coefficient/weight */
  double coeff{ 0 };
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
  using ConstPtr = std::shared_ptr<const CollisionEvaluator>;

  // NOLINTNEXTLINE
  CollisionEvaluator(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                     std::shared_ptr<const tesseract::environment::Environment> env,
                     bool dynamic_environment = false);
  virtual ~CollisionEvaluator() = default;
  CollisionEvaluator(const CollisionEvaluator&) = default;
  CollisionEvaluator& operator=(const CollisionEvaluator&) = default;
  CollisionEvaluator(CollisionEvaluator&&) = default;
  CollisionEvaluator& operator=(CollisionEvaluator&&) = default;

  /**
   * @brief Convert the contact information into an affine expression
   * @param x Optimizer variables
   * @param exprs Returned affine expression representation of the contact information
   * @param exprs_data The safety margin pair associated with the expression
   */
  virtual void CalcDistExpressions(const DblVec& x,
                                   sco::AffExprVector& exprs,
                                   std::vector<double>& exprs_margin,
                                   std::vector<double>& exprs_coeff) = 0;

  /**
   * @brief Given optimizer parameters calculate the collision results for this evaluator
   * @param x Optimizer variables
   * @param dist_results Contact results map
   */
  virtual void CalcCollisions(const DblVec& x, tesseract::collision::ContactResultMap& dist_results) = 0;

  /**
   * @brief Plot the collision evaluator results
   * @param plotter Plotter
   * @param x Optimizer variables
   */
  virtual void Plot(const std::shared_ptr<tesseract::visualization::Visualization>& plotter, const DblVec& x) = 0;

  /**
   * @brief Get the specific optimizer variables associated with this evaluator.
   * @return Evaluators variables
   */
  virtual sco::VarVector GetVars() = 0;

  /**
   * @brief This function checks to see if results are cached for input variable x. If not it calls CalcCollisions and
   * caches the results vector with x as the key.
   * @param x Optimizer variables
   */
  ContactResultVectorConstPtr GetContactResultVectorCached(const DblVec& x);

  /**
   * @brief This function checks to see if results are cached for input variable x. If not it calls CalcCollisions and
   * caches the results with x as the key.
   * @param x Optimizer variables
   */
  ContactResultMapConstPtr GetContactResultMapCached(const DblVec& x);

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param margin The link pair the contact margin.
   * @param coeff The link pair the coefficient/weight.
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                              const tesseract::collision::ContactResult& contact_result,
                              double margin,
                              double coeff,
                              bool isTimestep1);

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                              const tesseract::collision::ContactResult& contact_result,
                              bool isTimestep1);

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals0 The joint values at the first timestep
   * @param dofvals1 The joint values at the second timestep
   * @param contact_result The contact results to compute the gradient
   * @param margin The link pair the contact margin.
   * @param coeff The link pair the coefficient/weight.
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              const tesseract::collision::ContactResult& contact_result,
                              double margin,
                              double coeff,
                              bool isTimestep1);

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals0 The joint values at the first timestep
   * @param dofvals1 The joint values at the second timestep
   * @param contact_result The contact results to compute the gradient
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              const tesseract::collision::ContactResult& contact_result,
                              bool isTimestep1);

  /**
   * @brief The number of equal casts this evaluator's check splits the segment from @p dofvals0 to @p dofvals1 into
   * @details The two-state gradients place each contact in the cast it was found in from this count, so an evaluator
   * whose check casts must return the count its check uses. The default suits a check at interpolated states.
   * @return The cast count, or 0 when the segment is checked at interpolated states rather than cast
   */
  virtual long GetCastCount(const Eigen::Ref<const Eigen::VectorXd>& dofvals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dofvals1) const;

  /**
   * @brief Get the collision margin information.
   * @return Collision margin information
   */
  const tesseract::common::CollisionMarginData& getCollisionMarginData() const;

  /**
   * @brief Get the collision coefficient information.
   * @return Collision coefficient information
   */
  const trajopt_common::CollisionCoeffData& getCollisionCoeffData() const;

  /** @brief The collision results cached results */

  Cache<std::size_t, std::pair<ContactResultMapConstPtr, ContactResultVectorConstPtr>> m_cache{ 2 };

protected:
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip_;
  std::shared_ptr<const tesseract::environment::Environment> env_;
  std::unordered_set<tesseract::common::LinkId> env_active_link_ids_;
  std::unordered_set<tesseract::common::LinkId> manip_active_link_ids_;
  std::unordered_set<tesseract::common::LinkId> diff_active_link_ids_;
  /** @brief The union of manip_active_link_ids_ and diff_active_link_ids_; filled in the constructor, never mutated */
  std::unordered_set<tesseract::common::LinkId> all_active_link_ids_;
  tesseract::common::CollisionMarginData margin_data_;
  trajopt_common::CollisionCoeffData coeff_data_;
  double margin_buffer_{ 0.0 };
  tesseract::collision::CollisionCheckConfig collision_check_config_;

  sco::VarVector vars0_;
  sco::VarVector vars1_;
  bool vars0_fixed_{ false };
  bool vars1_fixed_{ false };
  CollisionExpressionEvaluatorType evaluator_type_{ CollisionExpressionEvaluatorType::START_FREE_END_FREE };
  std::function<void(tesseract::common::LinkIdTransformMap& transforms,
                     const Eigen::Ref<const Eigen::VectorXd>& joint_values)>
      get_state_fn_;
  bool dynamic_environment_{ false };

  std::pair<ContactResultMapConstPtr, ContactResultVectorConstPtr> GetContactResultCached(const DblVec& x);

  /** @brief Scratch for the link transforms a gradient linearises about. Reused across calls so
   * the map retains its nodes; get_state_fn_ overwrites the entry of every link its state source
   * currently holds, which covers every link a gradient reads. It does not erase, so a link the
   * environment later drops leaves a pose behind - harmless, since a dropped link yields no contact
   * and so is never read, but it is why entries here must not be trusted as a scene inventory.
   * Distinct from transforms_cache0_/transforms_cache1_ on purpose: those hold the poses of the
   * last collision check, which on a contact-cache hit belong to a different state. */
  tesseract::common::LinkIdTransformMap transforms_gradient_;
  tesseract::common::LinkIdTransformMap transforms_cache0_;
  tesseract::common::LinkIdTransformMap transforms_cache1_;

  /**
   * @brief Build one distance expression per contact found by a check at the state @p x gives @p vars
   * @details The gradient rotates each contact's reference points by the contact's stored link poses, so those must be
   * the poses at that state, as a check at that state stores them.
   */
  void CollisionsToDistanceExpressions(sco::AffExprVector& exprs,
                                       std::vector<double>& exprs_margin,
                                       std::vector<double>& exprs_coeff,
                                       const ContactResultVectorWrapper& dist_results,
                                       const sco::VarVector& vars,
                                       const DblVec& x,
                                       bool isTimestep1);

  /**
   * @brief Build one distance expression per contact for a segment with interpolated contacts, in either or both
   * states' variables
   *
   * Each contact's gradient is evaluated at the states the check found it between (see GetCastCount). When both
   * states' expressions are wanted, each of those states is evaluated once and serves both.
   * @param exprs0 Expressions built in vars0_, or nullptr when they are not wanted
   * @param exprs1 Expressions built in vars1_, or nullptr when they are not wanted
   */
  void CollisionsToDistanceExpressionsTwoState(sco::AffExprVector* exprs0,
                                               sco::AffExprVector* exprs1,
                                               std::vector<double>& exprs_margin,
                                               std::vector<double>& exprs_coeff,
                                               const ContactResultVectorWrapper& dist_results,
                                               const DblVec& x);

  /**
   * @brief Calculate the distance expressions when the start is free but the end is fixed
   * This creates an expression for every contact results found.
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsStartFree(const DblVec& x,
                                    sco::AffExprVector& exprs,
                                    std::vector<double>& exprs_margin,
                                    std::vector<double>& exprs_coeff);

  /**
   * @brief Calculate the distance expressions when the end is free but the start is fixed
   * This creates an expression for every contact results found.
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsEndFree(const DblVec& x,
                                  sco::AffExprVector& exprs,
                                  std::vector<double>& exprs_margin,
                                  std::vector<double>& exprs_coeff);

  /**
   * @brief Calculate the distance expressions when the start and end are free
   * This creates an expression for every contact results found.
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsBothFree(const DblVec& x,
                                   sco::AffExprVector& exprs,
                                   std::vector<double>& exprs_margin,
                                   std::vector<double>& exprs_coeff);

  /**
   * @brief Calculate the distance expressions for single time step
   * This creates an expression for every contact results found.
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsSingleTimeStep(const DblVec& x,
                                         sco::AffExprVector& exprs,
                                         std::vector<double>& exprs_margin,
                                         std::vector<double>& exprs_coeff);

  /**
   * @brief Remove any results that are invalid.
   * Invalid state are contacts that occur at fixed states or have distances outside the threshold.
   * @param contact_results Contact results vector to process.
   * @param margin The contact margin
   */
  void removeInvalidContactResults(tesseract::collision::ContactResultVector& contact_results, double margin) const;

private:
  /**
   * @brief Extracts the gradient information based on the contact results, given the poses of the contact's links at
   * @p dofvals
   * @param dofvals The joint values
   * @param link_poses The poses of the contact's two links at @p dofvals, in the order of the contact's link ids. Only
   * an active link's pose is read.
   * @param contact_result The contact results to compute the gradient
   * @param margin The link pair the contact margin.
   * @param coeff The link pair the coefficient/weight.
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                              const std::array<Eigen::Isometry3d, 2>& link_poses,
                              const tesseract::collision::ContactResult& contact_result,
                              double margin,
                              double coeff,
                              bool isTimestep1);

  /**
   * @brief Gradient of one of a contact's links with respect to the joints, at a given state
   * @param dofvalst The configuration the jacobian and the reference-point rotation are taken at
   * @param link_transforms Scratch space the call overwrites with the poses at @p dofvalst; must
   * outlive the call
   * @param i Which of the contact's two links; the contact normal points away from link 0, so the
   * sign of the returned gradient follows from it
   */
  Eigen::VectorXd CalcLinkGradient(const Eigen::VectorXd& dofvalst,
                                   tesseract::common::LinkIdTransformMap& link_transforms,
                                   const tesseract::collision::ContactResult& contact_result,
                                   std::size_t i);

  /**
   * @brief One of a contact's links' gradient for either or both timesteps of a segment
   * @details A timed link's contact point is modelled as moving with the link at both ends of the interval the check
   * found it in, so each timestep's gradient is the weighted mean of the gradients at those two states and its scale
   * is the total weight (see trajopt_common::intervalWeights). A link pinned to a segment endpoint is a point in time
   * there. A link the check gave no interval carries full weight at the state it is linearised at.
   * @param link_transforms Scratch space the call overwrites with the poses at each state it evaluates
   * @param i Which of the contact's two links
   * @param cast_count The number of equal casts the check split the segment into, or 0 when it checked interpolated
   * states
   * @param start The segment start's result, or nullptr when it is not wanted
   * @param end The segment end's result, or nullptr when it is not wanted
   */
  void CalcLinkGradientTwoState(const Eigen::VectorXd& dofvals0,
                                const Eigen::VectorXd& dofvals1,
                                tesseract::common::LinkIdTransformMap& link_transforms,
                                const tesseract::collision::ContactResult& contact_result,
                                std::size_t i,
                                long cast_count,
                                LinkGradientResults* start,
                                LinkGradientResults* end);

  /**
   * @brief Gradient for a contact between two states, for either or both timesteps
   * @details Both timesteps blend the gradients at the same two ends of a link's contact interval, differing only in
   * weights, so when both are wanted each end is evaluated once and serves both.
   * @param link_transforms Scratch space the call overwrites with the poses at each state it evaluates; nothing
   * carries over between calls, and the map must outlive the call
   * @param cast_count The number of equal casts the check split the segment into, or 0 when it checked interpolated
   * states; see trajopt_common::contactInterval
   * @param start The segment start's results, or nullptr when they are not wanted
   * @param end The segment end's results, or nullptr when they are not wanted
   */
  void CalcGradientTwoState(const Eigen::VectorXd& dofvals0,
                            const Eigen::VectorXd& dofvals1,
                            tesseract::common::LinkIdTransformMap& link_transforms,
                            const tesseract::collision::ContactResult& contact_result,
                            long cast_count,
                            GradientResults* start,
                            GradientResults* end);

  CollisionEvaluator() = default;
};

/**
 * @brief This collision evaluator only operates on a single state in the trajectory and does not check for collisions
 * between states.
 */
struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  using Ptr = std::shared_ptr<SingleTimestepCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const SingleTimestepCollisionEvaluator>;

  SingleTimestepCollisionEvaluator(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                                   std::shared_ptr<const tesseract::environment::Environment> env,
                                   const trajopt_common::TrajOptCollisionConfig& collision_config,
                                   sco::VarVector vars,
                                   CollisionExpressionEvaluatorType type,
                                   bool dynamic_environment = false);
  /**
  @brief linearize all contact distances in terms of robot dofs
  ;
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance
  function
  */
  void CalcDistExpressions(const DblVec& x,
                           sco::AffExprVector& exprs,
                           std::vector<double>& exprs_margin,
                           std::vector<double>& exprs_coeff) override;
  void CalcCollisions(const DblVec& x, tesseract::collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals Joint values set prior to collision checking
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                      tesseract::collision::ContactResultMap& dist_results);
  void Plot(const std::shared_ptr<tesseract::visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return vars0_; }

private:
  std::shared_ptr<tesseract::collision::DiscreteContactManager> contact_manager_;
  std::function<void(const DblVec&, sco::AffExprVector&, std::vector<double>&, std::vector<double>&)> fn_;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between them using casted
 * collision objects. Under LVS_CONTINUOUS the segment is interpolated so that no cast is longer than the longest
 * valid segment length; under CONTINUOUS it is cast once.
 */
struct CastCollisionEvaluator : public CollisionEvaluator
{
public:
  using Ptr = std::shared_ptr<CastCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const CastCollisionEvaluator>;

  CastCollisionEvaluator(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                         std::shared_ptr<const tesseract::environment::Environment> env,
                         const trajopt_common::TrajOptCollisionConfig& collision_config,
                         sco::VarVector vars0,
                         sco::VarVector vars1,
                         CollisionExpressionEvaluatorType type);
  void CalcDistExpressions(const DblVec& x,
                           sco::AffExprVector& exprs,
                           std::vector<double>& exprs_margin,
                           std::vector<double>& exprs_coeff) override;
  void CalcCollisions(const DblVec& x, tesseract::collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract::collision::ContactResultMap& dist_results);
  void Plot(const std::shared_ptr<tesseract::visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override;
  long GetCastCount(const Eigen::Ref<const Eigen::VectorXd>& dofvals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dofvals1) const override;

private:
  std::shared_ptr<tesseract::collision::ContinuousContactManager> contact_manager_;
  std::function<void(const DblVec&, sco::AffExprVector&, std::vector<double>&, std::vector<double>&)> fn_;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * discrete collision objects at each intermediate interpolated states.
 */
struct DiscreteCollisionEvaluator : public CollisionEvaluator
{
public:
  using Ptr = std::shared_ptr<DiscreteCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const DiscreteCollisionEvaluator>;

  DiscreteCollisionEvaluator(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                             std::shared_ptr<const tesseract::environment::Environment> env,
                             const trajopt_common::TrajOptCollisionConfig& collision_config,
                             sco::VarVector vars0,
                             sco::VarVector vars1,
                             CollisionExpressionEvaluatorType type);
  void CalcDistExpressions(const DblVec& x,
                           sco::AffExprVector& exprs,
                           std::vector<double>& exprs_margin,
                           std::vector<double>& exprs_coeff) override;
  void CalcCollisions(const DblVec& x, tesseract::collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract::collision::ContactResultMap& dist_results);
  void Plot(const std::shared_ptr<tesseract::visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override;

private:
  std::shared_ptr<tesseract::collision::DiscreteContactManager> contact_manager_;
  std::function<void(const DblVec&, sco::AffExprVector&, std::vector<double>&, std::vector<double>&)> fn_;
};

class CollisionCost : public sco::Cost, public Plotter
{
public:
  /* constructor for single timestep */
  CollisionCost(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                std::shared_ptr<const tesseract::environment::Environment> env,
                const trajopt_common::TrajOptCollisionConfig& collision_config,
                sco::VarVector vars,
                CollisionExpressionEvaluatorType type);
  /* constructor for discrete continuous and cast continuous cost */
  CollisionCost(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                std::shared_ptr<const tesseract::environment::Environment> env,
                const trajopt_common::TrajOptCollisionConfig& collision_config,
                sco::VarVector vars0,
                sco::VarVector vars1,
                CollisionExpressionEvaluatorType type,
                bool discrete);
  sco::ConvexObjective::Ptr convex(const DblVec& x, sco::Model* model) override;
  double value(const DblVec&) override;
  void Plot(const std::shared_ptr<tesseract::visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};

class CollisionConstraint : public sco::IneqConstraint
{
public:
  /* constructor for single timestep */
  CollisionConstraint(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                      std::shared_ptr<const tesseract::environment::Environment> env,
                      const trajopt_common::TrajOptCollisionConfig& collision_config,
                      sco::VarVector vars,
                      CollisionExpressionEvaluatorType type);
  /* constructor for discrete continuous and cast continuous cost */
  CollisionConstraint(const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                      std::shared_ptr<const tesseract::environment::Environment> env,
                      const trajopt_common::TrajOptCollisionConfig& collision_config,
                      sco::VarVector vars0,
                      sco::VarVector vars1,
                      CollisionExpressionEvaluatorType type,
                      bool discrete);
  sco::ConvexConstraints::Ptr convex(const DblVec& x, sco::Model* model) override;
  DblVec value(const DblVec&) override;
  void Plot(const DblVec& x);
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};
}  // namespace trajopt
