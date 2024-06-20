/**
 * @file discrete_collision_evaluators.h
 * @brief Contains discrete evaluators for the collision constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
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
#ifndef TRAJOPT_IFOPT_TRAJOPT_DISCRETE_COLLISION_EVALUATOR_H
#define TRAJOPT_IFOPT_TRAJOPT_DISCRETE_COLLISION_EVALUATOR_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
#include <functional>

#include <tesseract_collision/core/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_common/eigen_types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/fwd.h>
#include <trajopt_common/cache.h>

namespace trajopt_ifopt
{
using GetStateFn = std::function<tesseract_common::TransformMap(const Eigen::Ref<const Eigen::VectorXd>& joint_values)>;
using CollisionCache = trajopt_common::Cache<size_t, std::shared_ptr<const trajopt_common::CollisionCacheData>>;

/**
 * @brief This collision evaluator only operates on a single state in the trajectory and does not check for collisions
 * between states.
 */
class DiscreteCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<DiscreteCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const DiscreteCollisionEvaluator>;

  DiscreteCollisionEvaluator() = default;
  virtual ~DiscreteCollisionEvaluator() = default;
  DiscreteCollisionEvaluator(const DiscreteCollisionEvaluator&) = default;
  DiscreteCollisionEvaluator& operator=(const DiscreteCollisionEvaluator&) = default;
  DiscreteCollisionEvaluator(DiscreteCollisionEvaluator&&) = default;
  DiscreteCollisionEvaluator& operator=(DiscreteCollisionEvaluator&&) = default;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals Joint values set prior to collision checking
   * @return Collision cache data. If a cache does not exist for the provided joint values it evaluates and stores the
   * data.
   */
  virtual std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals, std::size_t bounds_size) = 0;

  /**
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  virtual const trajopt_common::TrajOptCollisionConfig& GetCollisionConfig() const = 0;

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  virtual trajopt_common::GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                                                      const tesseract_collision::ContactResult& contact_result) = 0;
};

/**
 * @brief This collision evaluator only operates on a single state in the trajectory and does not check for collisions
 * between states.
 */
class SingleTimestepCollisionEvaluator : public DiscreteCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SingleTimestepCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const SingleTimestepCollisionEvaluator>;

  SingleTimestepCollisionEvaluator(std::shared_ptr<CollisionCache> collision_cache,
                                   std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                   std::shared_ptr<const tesseract_environment::Environment> env,
                                   std::shared_ptr<const trajopt_common::TrajOptCollisionConfig> collision_config,
                                   bool dynamic_environment = false);

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals, std::size_t bounds_size) override final;

  trajopt_common::GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                                              const tesseract_collision::ContactResult& contact_result) override;

  const trajopt_common::TrajOptCollisionConfig& GetCollisionConfig() const override;

private:
  std::shared_ptr<CollisionCache> collision_cache_;
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  std::shared_ptr<const tesseract_environment::Environment> env_;
  std::shared_ptr<const trajopt_common::TrajOptCollisionConfig> collision_config_;
  std::vector<std::string> env_active_link_names_;
  std::vector<std::string> manip_active_link_names_;
  std::vector<std::string> diff_active_link_names_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  std::shared_ptr<tesseract_collision::DiscreteContactManager> contact_manager_;

  void CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                            tesseract_collision::ContactResultMap& dist_results);
};

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_TRAJOPT_DISCRETE_COLLISION_EVALUATOR_H
