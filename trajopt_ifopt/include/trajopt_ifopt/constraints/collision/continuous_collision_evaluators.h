/**
 * @file continuous_collision_evaluators.h
 * @brief Contains continuous evaluators for the collision constraint
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

#ifndef TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H
#define TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H

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
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * casted collision objects between to intermediate interpolated states.
 */
class ContinuousCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ContinuousCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const ContinuousCollisionEvaluator>;

  ContinuousCollisionEvaluator() = default;
  virtual ~ContinuousCollisionEvaluator() = default;
  ContinuousCollisionEvaluator(const ContinuousCollisionEvaluator&) = default;
  ContinuousCollisionEvaluator& operator=(const ContinuousCollisionEvaluator&) = default;
  ContinuousCollisionEvaluator(ContinuousCollisionEvaluator&&) = default;
  ContinuousCollisionEvaluator& operator=(ContinuousCollisionEvaluator&&) = default;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for start state
   * @param dof_vals1 Joint values for end state
   * @return Collision cache data. If a cache does not exist for the provided joint values it evaluates and stores the
   * data.
   */
  virtual std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                    const std::array<bool, 2>& position_vars_fixed,
                    std::size_t bounds_size) = 0;

  /**
   * @brief Extracts the gradient information based on the contact results for the transition between dof_vals0 and
   * dof_vals1
   * @param dof_vals0 Joint values for start state
   * @param dof_vals1 Joint values for end state
   * @param contact_results The contact results for transition between dof_vals0 and dof_vals1 to compute the gradient
   * data
   * @return The gradient results
   */
  virtual trajopt_common::GradientResults
  CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                   const tesseract_collision::ContactResult& contact_results) = 0;

  /**
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  virtual const trajopt_common::TrajOptCollisionConfig& GetCollisionConfig() const = 0;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * casted collision objects between to intermediate interpolated states.
 */
class LVSContinuousCollisionEvaluator : public ContinuousCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LVSContinuousCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const LVSContinuousCollisionEvaluator>;

  LVSContinuousCollisionEvaluator(std::shared_ptr<CollisionCache> collision_cache,
                                  std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                  std::shared_ptr<const tesseract_environment::Environment> env,
                                  std::shared_ptr<const trajopt_common::TrajOptCollisionConfig> collision_config,
                                  bool dynamic_environment = false);

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                    const std::array<bool, 2>& position_vars_fixed,
                    std::size_t bounds_size) override final;

  trajopt_common::GradientResults
  CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                   const tesseract_collision::ContactResult& contact_results) override final;

  const trajopt_common::TrajOptCollisionConfig& GetCollisionConfig() const override final;

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
  std::shared_ptr<tesseract_collision::ContinuousContactManager> contact_manager_;

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisionsCacheDataHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

  void CalcCollisionsHelper(tesseract_collision::ContactResultMap& dist_results,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                            const std::array<bool, 2>& position_vars_fixed);
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * descrete collision objects at each intermediate interpolated states.
 */
class LVSDiscreteCollisionEvaluator : public ContinuousCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LVSDiscreteCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const LVSDiscreteCollisionEvaluator>;

  LVSDiscreteCollisionEvaluator(std::shared_ptr<CollisionCache> collision_cache,
                                std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                std::shared_ptr<const tesseract_environment::Environment> env,
                                std::shared_ptr<const trajopt_common::TrajOptCollisionConfig> collision_config,
                                bool dynamic_environment = false);

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                    const std::array<bool, 2>& position_vars_fixed,
                    std::size_t bounds_size) override final;

  trajopt_common::GradientResults
  CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                   const tesseract_collision::ContactResult& contact_results) override final;

  const trajopt_common::TrajOptCollisionConfig& GetCollisionConfig() const override final;

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

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  CalcCollisionsCacheDataHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

  void CalcCollisionsHelper(tesseract_collision::ContactResultMap& dist_results,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                            const std::array<bool, 2>& position_vars_fixed);
};
}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H
