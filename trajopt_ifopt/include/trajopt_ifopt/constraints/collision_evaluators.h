/**
 * @file collision_evaluators.h
 * @brief Contains evaluators for the collision constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
 * @bug Only Discrete Evaluator is implemented
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TRAJOPT_IFOPT_COLLISION_EVALUATOR_H
#define TRAJOPT_IFOPT_COLLISION_EVALUATOR_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>

#include <tesseract_collision/core/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt/cache.hxx>

namespace trajopt
{
/**
 * @brief This contains the different types of expression evaluators used when performing continuous collision checking.
 */
enum class CollisionExpressionEvaluatorType
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

/** @brief Stores information about how the margins allowed between collision objects */
struct CollisionCoeffData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionCoeffData>;
  using ConstPtr = std::shared_ptr<const CollisionCoeffData>;

  CollisionCoeffData(const double& default_collision_coeff = 1) : default_collision_coeff_(default_collision_coeff) {}

  /**
   * @brief Set the margin for a given contact pair
   *
   * The order of the object names does not matter, that is handled internal to
   * the class.
   *
   * @param obj1 The first object name. Order doesn't matter
   * @param obj2 The Second object name. Order doesn't matter
   * @param Coeff used for collision pair
   */
  void setPairCollisionMarginData(const std::string& obj1, const std::string& obj2, const double& collision_coeff)
  {
    auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
    lookup_table_[key] = collision_coeff;
  }

  /**
   * @brief Get the pairs collision coeff
   *
   * If a collision coeff for the request pair does not exist it returns the default collision coeff
   *
   * @param obj1 The first object name
   * @param obj2 The second object name
   * @return Coefficient
   */
  const double& getPairCollisionCoeff(const std::string& obj1, const std::string& obj2) const
  {
    auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
    const auto it = lookup_table_.find(key);

    if (it != lookup_table_.end())
    {
      return it->second;
    }
    return default_collision_coeff_;
  }

private:
  /// Stores the collision margin used if no pair-specific one is set
  double default_collision_coeff_;

  /// A map of link pair names to contact distance
  std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash> lookup_table_;
};

/**
 * @brief Config settings for collision terms.
 */
struct TrajOptCollisionConfig : public tesseract_collision::CollisionCheckConfig
{
  using Ptr = std::shared_ptr<TrajOptCollisionConfig>;
  using ConstPtr = std::shared_ptr<const TrajOptCollisionConfig>;

  TrajOptCollisionConfig() = default;
  TrajOptCollisionConfig(double margin, double coeff);

  /**
   * @brief Use the weighted sum for each link pair. This reduces the number equations added to the problem
   * If set to true, it is recommended to start with the coeff set to one
   */
  bool use_weighted_sum = false;

  /** @brief The collision coeff/weight */
  CollisionCoeffData collision_coeff_data;

  /** @brief Additional collision margin that is added for the collision check but is not used when calculating the
   * error */
  double collision_margin_buffer{ 0 };
};

/** @brief A data structure to contain a links gradient results */
struct LinkGradientResults
{
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
  /**
   * @brief Construct the GradientResults
   * @param data The link pair safety margin data
   */
  GradientResults(const Eigen::Vector2d& data) : data(data) {}

  /** @brief The gradient results data for LinkA and LinkB */
  LinkGradientResults gradients[2];

  /** @brief The link pair safety margin data */
  const Eigen::Vector2d& data;
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

  CollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                     tesseract_environment::Environment::ConstPtr env,
                     tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                     const Eigen::Isometry3d& world_to_base,
                     const TrajOptCollisionConfig& collision_config,
                     bool dynamic_environment = false);
  virtual ~CollisionEvaluator() = default;
  CollisionEvaluator(const CollisionEvaluator&) = default;
  CollisionEvaluator& operator=(const CollisionEvaluator&) = default;
  CollisionEvaluator(CollisionEvaluator&&) = default;
  CollisionEvaluator& operator=(CollisionEvaluator&&) = default;

  /**
   * @brief This function calls GetCollisionsCached and stores the distances in a vector
   * @param x Joint values. For contact managers that use more than one state, the states should be appended
   * @param dists Returned distance values
   */
  virtual void CalcDists(const std::vector<double>& x, std::vector<double>& dists);

  /**
   * @brief Given optimizer parameters calculate the collision results for this evaluator
   * @param x Joint values. For contact managers that use more than one state, the states should be appended
   * @param dist_results Contact results map
   */
  virtual void CalcCollisions(const std::vector<double>& x, tesseract_collision::ContactResultMap& dist_results) = 0;

  /**
   * @brief Given optimizer parameters calculate the collision results for this evaluator
   * @param x Joint values. For contact managers that use more than one state, the states should be appended
   * @param dist_map Contact results map
   * @param dist_map Contact results vector
   */
  void CalcCollisions(const std::vector<double>& x,
                      tesseract_collision::ContactResultMap& dist_map,
                      tesseract_collision::ContactResultVector& dist_vector);

  /**
   * @brief This function checks to see if results are cached for input variable x. If not it calls CalcCollisions and
   * caches the results vector with x as the key.
   * @param x Joint values. For contact managers that use more than one state, the states should be appended
   */
  void GetCollisionsCached(const std::vector<double>& x, tesseract_collision::ContactResultVector&);

  /**
   * @brief This function checks to see if results are cached for input variable x. If not it calls CalcCollisions and
   * caches the results with x as the key.
   * @param x Joint values. For contact managers that use more than one state, the states should be appended
   */
  void GetCollisionsCached(const std::vector<double>& x, tesseract_collision::ContactResultMap&);

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param data Data associated with the link pair the contact results associated with.
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                              const tesseract_collision::ContactResult& contact_result,
                              const Eigen::Vector2d& data,
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
   * @param data Data associated with the link pair the contact results associated with.
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              const tesseract_collision::ContactResult& contact_result,
                              const Eigen::Vector2d& data,
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
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  TrajOptCollisionConfig& getCollisionConfig() { return collision_config_; }
  Cache<size_t, std::pair<tesseract_collision::ContactResultMap, tesseract_collision::ContactResultVector>, 10> m_cache;

protected:
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  TrajOptCollisionConfig collision_config_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  CollisionExpressionEvaluatorType evaluator_type_;
  std::function<tesseract_environment::EnvState::Ptr(const std::vector<std::string>& joint_names,
                                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values)>
      get_state_fn_;
  bool dynamic_environment_;

  /**
   * @brief This takes contacts results at each interpolated timestep and creates a single contact results map.
   * This also updates the cc_time and cc_type for the contact results
   * @param contacts_vector Contact results map at each interpolated timestep
   * @param contact_results The merged contact results map
   * @param dt delta time
   */
  void processInterpolatedCollisionResults(std::vector<tesseract_collision::ContactResultMap>& contacts_vector,
                                           tesseract_collision::ContactResultMap& contact_results,
                                           double dt) const;

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
struct DiscreteCollisionEvaluator : public CollisionEvaluator
{
public:
  using Ptr = std::shared_ptr<DiscreteCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const DiscreteCollisionEvaluator>;

  DiscreteCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                             tesseract_environment::Environment::ConstPtr env,
                             tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                             const Eigen::Isometry3d& world_to_base,
                             const TrajOptCollisionConfig& collision_config,
                             bool dynamic_environment = false);

  void CalcCollisions(const std::vector<double>& x, tesseract_collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals Joint values set prior to collision checking
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                      tesseract_collision::ContactResultMap& dist_results);

private:
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * casted collision objects between to intermediate interpolated states.
 */
struct LVSContinuousCollisionEvaluator : public CollisionEvaluator
{
public:
  using Ptr = std::shared_ptr<LVSContinuousCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const LVSContinuousCollisionEvaluator>;

  LVSContinuousCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                  tesseract_environment::Environment::ConstPtr env,
                                  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                  const Eigen::Isometry3d& world_to_base,
                                  const TrajOptCollisionConfig& collision_config,
                                  bool dynamic_environment = false);

  void CalcCollisions(const std::vector<double>& x, tesseract_collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultMap& dist_results);

private:
  tesseract_collision::ContinuousContactManager::Ptr contact_manager_;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * descrete collision objects at each intermediate interpolated states.
 */
struct LVSDiscreteCollisionEvaluator : public CollisionEvaluator
{
public:
  using Ptr = std::shared_ptr<LVSDiscreteCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const LVSDiscreteCollisionEvaluator>;

  LVSDiscreteCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                tesseract_environment::Environment::ConstPtr env,
                                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                const Eigen::Isometry3d& world_to_base,
                                const TrajOptCollisionConfig& collision_config,
                                bool dynamic_environment = false);

  void CalcCollisions(const std::vector<double>& dof_vals,
                      tesseract_collision::ContactResultMap& dist_results) override;
  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Map
   */
  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultMap& dist_results);

private:
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

}  // namespace trajopt

#endif
