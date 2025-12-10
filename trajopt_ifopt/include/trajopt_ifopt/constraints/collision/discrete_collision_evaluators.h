/**
 * @file discrete_collision_evaluators.h
 * @brief Contains discrete evaluators for the collision constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
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
#ifndef TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H
#define TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
#include <functional>

#include <tesseract_collision/core/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_common/eigen_types.h>
#include <tesseract_common/collision_margin_data.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_types.h>
#include <trajopt_common/cache.h>

namespace trajopt_ifopt
{
using GetStateFn =
    std::function<void(tesseract_common::TransformMap&, const Eigen::Ref<const Eigen::VectorXd>& joint_values)>;
using CollisionCache = trajopt_common::Cache<std::size_t, std::shared_ptr<const trajopt_common::CollisionCacheData>>;

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
  DiscreteCollisionEvaluator(const DiscreteCollisionEvaluator&) = delete;
  DiscreteCollisionEvaluator& operator=(const DiscreteCollisionEvaluator&) = delete;
  DiscreteCollisionEvaluator(DiscreteCollisionEvaluator&&) = delete;
  DiscreteCollisionEvaluator& operator=(DiscreteCollisionEvaluator&&) = delete;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals Joint values set prior to collision checking
   * @return Collision cache data. If a cache does not exist for the provided joint values it evaluates and stores the
   * data.
   */
  virtual std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals, std::size_t bounds_size) = 0;

  /**
   * @brief Get the collision margin buffer
   * @return The collision margin buffer
   */
  virtual double GetCollisionMarginBuffer() const = 0;

  /**
   * @brief Get the collision margin information.
   * @return Collision margin information
   */
  virtual const tesseract_common::CollisionMarginData& GetCollisionMarginData() const = 0;

  /**
   * @brief Get the collision coefficient information.
   * @return Collision coefficient information
   */
  virtual const trajopt_common::CollisionCoeffData& GetCollisionCoeffData() const = 0;

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
                                   const trajopt_common::TrajOptCollisionConfig& collision_config,
                                   bool dynamic_environment = false);

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals, std::size_t bounds_size) override final;

  trajopt_common::GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                                              const tesseract_collision::ContactResult& contact_result) override;

  double GetCollisionMarginBuffer() const override final;

  const tesseract_common::CollisionMarginData& GetCollisionMarginData() const override final;

  const trajopt_common::CollisionCoeffData& GetCollisionCoeffData() const override final;

private:
  std::shared_ptr<CollisionCache> collision_cache_;
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  std::shared_ptr<const tesseract_environment::Environment> env_;
  tesseract_common::CollisionMarginData margin_data_;
  trajopt_common::CollisionCoeffData coeff_data_;
  double margin_buffer_{ 0.0 };
  tesseract_collision::CollisionCheckConfig collision_check_config_;
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

#endif  // TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H
