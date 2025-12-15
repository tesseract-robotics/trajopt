/**
 * @file continuous_collision_evaluators.h
 * @brief Contains continuous evaluators for the collision constraint
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
  ContinuousCollisionEvaluator(const ContinuousCollisionEvaluator&) = delete;
  ContinuousCollisionEvaluator& operator=(const ContinuousCollisionEvaluator&) = delete;
  ContinuousCollisionEvaluator(ContinuousCollisionEvaluator&&) = delete;
  ContinuousCollisionEvaluator& operator=(ContinuousCollisionEvaluator&&) = delete;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for start state
   * @param dof_vals1 Joint values for end state
   * @return Collision cache data. If a cache does not exist for the provided joint values it evaluates and stores the
   * data.
   */
  virtual std::shared_ptr<const trajopt_common::CollisionCacheData>
  calcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                    bool vars0_fixed,
                    bool vars1_fixed,
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
  calcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                   const tesseract_collision::ContactResult& contact_results) = 0;

  /**
   * @brief Get the collision margin buffer
   * @return The collision margin buffer
   */
  virtual double getCollisionMarginBuffer() const = 0;

  /**
   * @brief Get the collision margin information.
   * @return Collision margin information
   */
  virtual const tesseract_common::CollisionMarginData& getCollisionMarginData() const = 0;

  /**
   * @brief Get the collision coefficient information.
   * @return Collision coefficient information
   */
  virtual const trajopt_common::CollisionCoeffData& getCollisionCoeffData() const = 0;

protected:
  static thread_local tesseract_common::TransformMap transforms_cache0;  // NOLINT
  static thread_local tesseract_common::TransformMap transforms_cache1;  // NOLINT
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
                                  const trajopt_common::TrajOptCollisionConfig& collision_config,
                                  bool dynamic_environment = false);

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  calcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                    bool vars0_fixed,
                    bool vars1_fixed,
                    std::size_t bounds_size) override final;

  trajopt_common::GradientResults
  calcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                   const tesseract_collision::ContactResult& contact_results) override final;

  double getCollisionMarginBuffer() const override final;

  const tesseract_common::CollisionMarginData& getCollisionMarginData() const override final;

  const trajopt_common::CollisionCoeffData& getCollisionCoeffData() const override final;

private:
  std::shared_ptr<CollisionCache> collision_cache_;
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  std::shared_ptr<const tesseract_environment::Environment> env_;
  tesseract_common::CollisionMarginData margin_data_;
  trajopt_common::CollisionCoeffData coeff_data_;
  double margin_buffer_{ 0.0 };
  tesseract_collision::CollisionCheckConfig collision_check_config_;
  bool single_timestep_{ false };
  std::vector<std::string> env_active_link_names_;
  std::vector<std::string> manip_active_link_names_;
  std::vector<std::string> diff_active_link_names_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  std::shared_ptr<tesseract_collision::ContinuousContactManager> contact_manager_;

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  calcCollisionsCacheDataHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

  void calcCollisionsHelper(tesseract_collision::ContactResultMap& dist_results,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                            bool vars0_fixed,
                            bool vars1_fixed);
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
                                const trajopt_common::TrajOptCollisionConfig& collision_config,
                                bool dynamic_environment = false);

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  calcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                    bool vars0_fixed,
                    bool vars1_fixed,
                    std::size_t bounds_size) override final;

  trajopt_common::GradientResults
  calcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                   const tesseract_collision::ContactResult& contact_results) override final;

  double getCollisionMarginBuffer() const override final;

  const tesseract_common::CollisionMarginData& getCollisionMarginData() const override final;

  const trajopt_common::CollisionCoeffData& getCollisionCoeffData() const override final;

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

  std::shared_ptr<const trajopt_common::CollisionCacheData>
  calcCollisionsCacheDataHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

  void calcCollisionsHelper(tesseract_collision::ContactResultMap& dist_results,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                            bool vars0_fixed,
                            bool vars1_fixed);
};
}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H
