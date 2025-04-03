#pragma once
#include <cstdint>
#include <vector>
#include <memory>
#include <Eigen/Core>

#include <tesseract_collision/core/fwd.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_visualization/fwd.h>

#include <trajopt_common/collision_types.h>
#include <trajopt_sco/sco_common.hpp>

#include <trajopt/cache.hxx>
#include <trajopt/typedefs.hpp>

namespace trajopt
{
using ContactResultMapConstPtr = std::shared_ptr<const tesseract_collision::ContactResultMap>;
using ContactResultVectorWrapper = std::vector<std::reference_wrapper<const tesseract_collision::ContactResult>>;
using ContactResultVectorConstPtr = std::shared_ptr<const ContactResultVectorWrapper>;

/**
 * @brief This contains the different types of expression evaluators used when performing continuous collision checking.
 */
enum class CollisionExpressionEvaluatorType : std::uint8_t
{
  START_FREE_END_FREE = 0,  /**< @brief Both start and end state variables are free to be adjusted */
  START_FREE_END_FIXED = 1, /**< @brief Only start state variables are free to be adjusted */
  START_FIXED_END_FREE = 2, /**< @brief Only end state variables are free to be adjusted */

  /**
   * @brief Both start and end state variables are free to be adjusted
   * The jacobian is calculated using a weighted sum over the interpolated states for a given link pair.
   */
  START_FREE_END_FREE_WEIGHTED_SUM = 3,
  /**
   * @brief Only start state variables are free to be adjusted
   * The jacobian is calculated using a weighted sum over the interpolated states for a given link pair.
   */
  START_FREE_END_FIXED_WEIGHTED_SUM = 4,
  /**
   * @brief Only end state variables are free to be adjusted
   * The jacobian is calculated using a weighted sum over the interpolated states for a given link pair.
   */
  START_FIXED_END_FREE_WEIGHTED_SUM = 5,

  SINGLE_TIME_STEP = 6, /**< @brief Expressions are only calculated at a single time step */
  /**
   * @brief Expressions are only calculated at a single time step
   * The jacobian is calculated using a weighted sum for a given link pair.
   */
  SINGLE_TIME_STEP_WEIGHTED_SUM = 7
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
  CollisionEvaluator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                     std::shared_ptr<const tesseract_environment::Environment> env,
                     bool dynamic_environment = false);
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
  virtual void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultMap& dist_results) = 0;

  /**
   * @brief Plot the collision evaluator results
   * @param plotter Plotter
   * @param x Optimizer variables
   */
  virtual void Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter, const DblVec& x) = 0;

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
                              const tesseract_collision::ContactResult& contact_result,
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
                              const tesseract_collision::ContactResult& contact_result,
                              bool isTimestep1);

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param margin The link pair the contact margin.
   * @param coeff The link pair the coefficient/weight.
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              const tesseract_collision::ContactResult& contact_result,
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
  GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              const tesseract_collision::ContactResult& contact_result,
                              bool isTimestep1);

  /**
   * @brief Get the collision margin information.
   * @return Collision margin information
   */
  const tesseract_common::CollisionMarginData& getCollisionMarginData() const;

  /**
   * @brief Get the collision coefficient information.
   * @return Collision coefficient information
   */
  const trajopt_common::CollisionCoeffData& getCollisionCoeffData() const;

  /** @brief The collision results cached results */

  Cache<std::size_t, std::pair<ContactResultMapConstPtr, ContactResultVectorConstPtr>> m_cache{ 2 };

protected:
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  std::shared_ptr<const tesseract_environment::Environment> env_;
  std::vector<std::string> env_active_link_names_;
  std::vector<std::string> manip_active_link_names_;
  std::vector<std::string> diff_active_link_names_;
  tesseract_collision::ContactRequest contact_request_;
  tesseract_common::CollisionMarginData margin_data_;
  trajopt_common::CollisionCoeffData coeff_data_;
  double margin_buffer_{ 0.0 };
  double longest_valid_segment_length_{ 0.005 };

  sco::VarVector vars0_;
  sco::VarVector vars1_;
  bool vars0_fixed_{ false };
  bool vars1_fixed_{ false };
  CollisionExpressionEvaluatorType evaluator_type_{ CollisionExpressionEvaluatorType::START_FREE_END_FREE };
  std::function<tesseract_common::TransformMap(const Eigen::Ref<const Eigen::VectorXd>& joint_values)> get_state_fn_;
  bool dynamic_environment_{ false };

  std::pair<ContactResultMapConstPtr, ContactResultVectorConstPtr> GetContactResultCached(const DblVec& x);

  void CollisionsToDistanceExpressions(sco::AffExprVector& exprs,
                                       std::vector<double>& exprs_margin,
                                       std::vector<double>& exprs_coeff,
                                       const ContactResultVectorWrapper& dist_results,
                                       const sco::VarVector& vars,
                                       const DblVec& x,
                                       bool isTimestep1);

  void CollisionsToDistanceExpressionsW(sco::AffExprVector& exprs,
                                        std::vector<double>& exprs_margin,
                                        std::vector<double>& exprs_coeff,
                                        const tesseract_collision::ContactResultMap& dist_results,
                                        const sco::VarVector& vars,
                                        const DblVec& x,
                                        bool isTimestep1);

  void CollisionsToDistanceExpressionsContinuousW(sco::AffExprVector& exprs,
                                                  std::vector<double>& exprs_margin,
                                                  std::vector<double>& exprs_coeff,
                                                  const tesseract_collision::ContactResultMap& dist_results,
                                                  const sco::VarVector& vars0,
                                                  const sco::VarVector& vars1,
                                                  const DblVec& x,
                                                  bool isTimestep1);

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
   * @brief Calculate the distance expressions when the start is free but the end is fixed
   * This creates the expression based on the weighted sum for given link pair
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsStartFreeW(const DblVec& x,
                                     sco::AffExprVector& exprs,
                                     std::vector<double>& exprs_margin,
                                     std::vector<double>& exprs_coeff);

  /**
   * @brief Calculate the distance expressions when the end is free but the start is fixed
   * This creates the expression based on the weighted sum for given link pair
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsEndFreeW(const DblVec& x,
                                   sco::AffExprVector& exprs,
                                   std::vector<double>& exprs_margin,
                                   std::vector<double>& exprs_coeff);

  /**
   * @brief Calculate the distance expressions when the start and end are free
   * This creates the expression based on the weighted sum for given link pair
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsBothFreeW(const DblVec& x,
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
   * @brief Calculate the distance expressions for single time step
   * This creates the expression based on the weighted sum for given link pair
   * @param x The current values
   * @param exprs The returned expression
   * @param exprs_data The safety margin pair associated with the expression
   */
  void CalcDistExpressionsSingleTimeStepW(const DblVec& x,
                                          sco::AffExprVector& exprs,
                                          std::vector<double>& exprs_margin,
                                          std::vector<double>& exprs_coeff);

  /**
   * @brief Remove any results that are invalid.
   * Invalid state are contacts that occur at fixed states or have distances outside the threshold.
   * @param contact_results Contact results vector to process.
   * @param margin The contact margin
   */
  void removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results, double margin) const;

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
  using Ptr = std::shared_ptr<SingleTimestepCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const SingleTimestepCollisionEvaluator>;

  SingleTimestepCollisionEvaluator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                   std::shared_ptr<const tesseract_environment::Environment> env,
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
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals Joint values set prior to collision checking
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                      tesseract_collision::ContactResultMap& dist_results);
  void Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return vars0_; }

private:
  std::shared_ptr<tesseract_collision::DiscreteContactManager> contact_manager_;
  std::function<void(const DblVec&, sco::AffExprVector&, std::vector<double>&, std::vector<double>&)> fn_;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * casted collision objects between to intermediate interpolated states.
 */
struct CastCollisionEvaluator : public CollisionEvaluator
{
public:
  using Ptr = std::shared_ptr<CastCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const CastCollisionEvaluator>;

  CastCollisionEvaluator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                         std::shared_ptr<const tesseract_environment::Environment> env,
                         const trajopt_common::TrajOptCollisionConfig& collision_config,
                         sco::VarVector vars0,
                         sco::VarVector vars1,
                         CollisionExpressionEvaluatorType type);
  void CalcDistExpressions(const DblVec& x,
                           sco::AffExprVector& exprs,
                           std::vector<double>& exprs_margin,
                           std::vector<double>& exprs_coeff) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultMap& dist_results);
  void Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override;

private:
  std::shared_ptr<tesseract_collision::ContinuousContactManager> contact_manager_;
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

  DiscreteCollisionEvaluator(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                             std::shared_ptr<const tesseract_environment::Environment> env,
                             const trajopt_common::TrajOptCollisionConfig& collision_config,
                             sco::VarVector vars0,
                             sco::VarVector vars1,
                             CollisionExpressionEvaluatorType type);
  void CalcDistExpressions(const DblVec& x,
                           sco::AffExprVector& exprs,
                           std::vector<double>& exprs_margin,
                           std::vector<double>& exprs_coeff) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultMap& dist_results);
  void Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override;

private:
  std::shared_ptr<tesseract_collision::DiscreteContactManager> contact_manager_;
  std::function<void(const DblVec&, sco::AffExprVector&, std::vector<double>&, std::vector<double>&)> fn_;
};

class CollisionCost : public sco::Cost, public Plotter
{
public:
  /* constructor for single timestep */
  CollisionCost(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                std::shared_ptr<const tesseract_environment::Environment> env,
                const trajopt_common::TrajOptCollisionConfig& collision_config,
                sco::VarVector vars,
                CollisionExpressionEvaluatorType type);
  /* constructor for discrete continuous and cast continuous cost */
  CollisionCost(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                std::shared_ptr<const tesseract_environment::Environment> env,
                const trajopt_common::TrajOptCollisionConfig& collision_config,
                sco::VarVector vars0,
                sco::VarVector vars1,
                CollisionExpressionEvaluatorType type,
                bool discrete);
  sco::ConvexObjective::Ptr convex(const DblVec& x, sco::Model* model) override;
  double value(const DblVec&) override;
  void Plot(const std::shared_ptr<tesseract_visualization::Visualization>& plotter, const DblVec& x) override;
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};

class CollisionConstraint : public sco::IneqConstraint
{
public:
  /* constructor for single timestep */
  CollisionConstraint(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                      std::shared_ptr<const tesseract_environment::Environment> env,
                      const trajopt_common::TrajOptCollisionConfig& collision_config,
                      sco::VarVector vars,
                      CollisionExpressionEvaluatorType type);
  /* constructor for discrete continuous and cast continuous cost */
  CollisionConstraint(std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                      std::shared_ptr<const tesseract_environment::Environment> env,
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
