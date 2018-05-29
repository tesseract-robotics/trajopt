/**
 * @file kdl_joint_kin.h
 * @brief Tesseract ROS KDL joint kinematics implementation.
 *
 * @author Levi Armstrong
 * @date May 27, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ROS_KDL_JOINT_KIN_H
#define TESSERACT_ROS_KDL_JOINT_KIN_H

#include "tesseract_ros/ros_basic_kin.h"
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <urdf/model.h>

namespace tesseract
{
namespace tesseract_ros
{
/**
 * @brief ROS kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 */
class KDLJointKin : public ROSBasicKin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLJointKin() : ROSBasicKin(), initialized_(false) {}
  bool calcFwdKin(Eigen::Affine3d& pose,
                  const Eigen::Affine3d& change_base,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  bool calcFwdKin(Eigen::Affine3d& pose,
                  const Eigen::Affine3d& change_base,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                  const std::string& link_name,
                  const EnvState& state) const override;

  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                    const Eigen::Affine3d& change_base,
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                    const Eigen::Affine3d& change_base,
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                    const std::string& link_name,
                    const EnvState& state) const override;

  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                    const Eigen::Affine3d& change_base,
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                    const std::string& link_name,
                    const EnvState& state,
                    const Eigen::Ref<const Eigen::Vector3d>& link_point) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;

  const std::vector<std::string>& getJointNames() const override;

  const std::vector<std::string>& getLinkNames() const override;

  const Eigen::MatrixX2d& getLimits() const override;

  urdf::ModelInterfaceConstSharedPtr getURDF() const override { return model_; }
  unsigned int numJoints() const override { return joint_list_.size(); }
  const std::string& getBaseLinkName() const { return model_->getRoot()->name; }
  const std::string& getName() const override { return name_; }
  void addAttachedLink(const std::string& link_name, const std::string& parent_link_name) override;

  void removeAttachedLink(const std::string& link_name) override;

  void clearAttachedLinks() override;

  /**
   * @brief Initializes ROSKin
   * Creates KDL::Chain from urdf::Model, populates joint_list_, joint_limits_, and link_list_
   * @param model The urdf model
   * @param joint_names The list of active joints to be considered
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(urdf::ModelInterfaceConstSharedPtr model,
            const std::vector<std::string>& joint_names,
            const std::string name);

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const
  {
    if (!initialized_)
    {
      ROS_ERROR("Kinematics has not been initialized!");
    }

    return initialized_;
  }

  /**
   * @brief Assigns values from another ROSKin to this
   * @param rhs Input ROSKin object to copy from
   * @return reference to this ROSKin object
   */
  KDLJointKin& operator=(const KDLJointKin& rhs);

private:
  bool initialized_;                         /**< Identifies if the object has been initialized */
  urdf::ModelInterfaceConstSharedPtr model_; /**< URDF MODEL */
  KDL::Tree kdl_tree_;                       /**< KDL tree object */
  std::string name_;                         /**< Name of the kinematic chain */
  std::vector<std::string> joint_list_;      /**< List of joint names */
  std::vector<int> joint_qnr_;               /**< The kdl segment number corrisponding to joint in joint_lists_ */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< The tree joint name to qnr */
  std::vector<std::string> link_list_;                         /**< List of link names */
  Eigen::MatrixX2d joint_limits_;                              /**< Joint limits */
  std::unique_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_;  /**< KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;        /**< KDL Jacobian Solver */
  std::vector<std::string> attached_link_list_;                /**< A list of attached link names */

  /** @brief calcFwdKin helper function */
  bool calcFwdKinHelper(Eigen::Affine3d& pose,
                        const Eigen::Affine3d& change_base,
                        const KDL::JntArray& kdl_joints,
                        const std::string& link_name) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(KDL::Jacobian& jacobian,
                          const Eigen::Affine3d& change_base,
                          const KDL::JntArray& kdl_joints,
                          const std::string& link_name) const;

  KDL::JntArray getKDLJntArray(const EnvState& state,
                               const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const;

  void addChildrenRecursive(const urdf::LinkConstSharedPtr urdf_link);

};  // class KDLChainKin

typedef std::shared_ptr<KDLJointKin> KDLJointKinPtr;
typedef std::shared_ptr<const KDLJointKin> KDLJointKinConstPtr;
}
}
#endif  // TESSERACT_ROS_KDL_JOINT_KIN_H
