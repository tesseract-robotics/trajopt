/**
 * @file discrete_collision_evaluators.h
 * @brief Contains discrete evaluators for the collision constraint
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
#ifndef TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H
#define TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>

#include <tesseract_collision/core/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_kinematics/core/joint_group.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision/collision_types.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_ifopt
{
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
  virtual CollisionCacheData::ConstPtr CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals) = 0;

  /**
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  virtual const TrajOptCollisionConfig& GetCollisionConfig() const = 0;

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  virtual GradientResults GetGradient(const Eigen::VectorXd& dofvals,
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
                                   tesseract_kinematics::JointGroup::ConstPtr manip,
                                   tesseract_environment::Environment::ConstPtr env,
                                   TrajOptCollisionConfig::ConstPtr collision_config,
                                   bool dynamic_environment = false);

  CollisionCacheData::ConstPtr CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals) override;

  GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                              const tesseract_collision::ContactResult& contact_result) override;

  const TrajOptCollisionConfig& GetCollisionConfig() const override;

private:
  std::shared_ptr<CollisionCache> collision_cache_;
  tesseract_kinematics::JointGroup::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  TrajOptCollisionConfig::ConstPtr collision_config_;
  std::vector<std::string> env_active_link_names_;
  std::vector<std::string> manip_active_link_names_;
  std::vector<std::string> diff_active_link_names_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;

  void CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                            tesseract_collision::ContactResultMap& dist_results);
};

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H
