/**
 * @file kdl_chain_kin.h
 * @brief Tesseract ROS KDL Chain kinematics implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_ROS_KDL_CHAIN_KIN_H
#define TESSERACT_ROS_KDL_CHAIN_KIN_H

#include "tesseract_ros/ros_basic_kin.h"
#include "tesseract_ros/ros_basic_env.h"
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

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
class KDLChainKin : public ROSBasicKin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLChainKin() : ROSBasicKin(), initialized_(false) {}
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

  const Eigen::MatrixX2d& getLimits() const;

  urdf::ModelInterfaceConstSharedPtr getURDF() const override { return model_; }
  unsigned int numJoints() const override { return robot_chain_.getNrOfJoints(); }
  const std::string& getName() const override { return name_; }
  void addAttachedLink(const std::string& link_name, const std::string& parent_link_name) override;

  void removeAttachedLink(const std::string& link_name) override;

  void clearAttachedLinks() override;

  /**
   * @brief Initializes ROSKin
   * Creates KDL::Chain from urdf::Model, populates joint_list_, joint_limits_, and link_list_
   * @param model_ The urdf model
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(urdf::ModelInterfaceConstSharedPtr model_,
            const std::string& base_link,
            const std::string& tip_link,
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

  /** @brief Get the base link name */
  const std::string& getBaseLinkName() const { return base_name_; }
  /** @brief Get the tip link name */
  const std::string& getTipLinkName() const { return tip_name_; }
  /**
   * @brief Assigns values from another ROSKin to this
   * @param rhs Input ROSKin object to copy from
   * @return reference to this ROSKin object
   */
  KDLChainKin& operator=(const KDLChainKin& rhs);

private:
  bool initialized_;                                           /**< Identifies if the object has been initialized */
  urdf::ModelInterfaceConstSharedPtr model_;                   /**< URDF MODEL */
  KDL::Chain robot_chain_;                                     /**< KDL Chain object */
  KDL::Tree kdl_tree_;                                         /**< KDL tree object */
  std::string base_name_;                                      /**< Link name of first link in the kinematic chain */
  std::string tip_name_;                                       /**< Link name of last kink in the kinematic chain */
  std::string name_;                                           /**< Name of the kinematic chain */
  std::vector<std::string> joint_list_;                        /**< List of joint names */
  std::vector<std::string> link_list_;                         /**< List of link names */
  Eigen::MatrixX2d joint_limits_;                              /**< Joint limits */
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; /**< KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;       /**< KDL Jacobian Solver */
  std::map<std::string, int> segment_index_;    /**< A map from chain link name to kdl chain segment number */
  std::vector<std::string> attached_link_list_; /**< A list of attached link names */
  std::unordered_map<std::string, std::string> link_name_too_chain_link_name_; /**< A map of affected link names to
                                                                                  chain link names */

  /** @brief calcFwdKin helper function */
  bool calcFwdKinHelper(Eigen::Affine3d& pose,
                        const Eigen::Affine3d& change_base,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                        int segment_num = -1) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(KDL::Jacobian& jacobian,
                          const Eigen::Affine3d& change_base,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                          int segment_num = -1) const;

  void addChildrenRecursive(const std::string& chain_link_name,
                            urdf::LinkConstSharedPtr urdf_link,
                            const std::string& next_chain_segment);

};  // class KDLChainKin

typedef std::shared_ptr<KDLChainKin> KDLChainKinPtr;
typedef std::shared_ptr<const KDLChainKin> KDLChainKinConstPtr;
}
}
#endif  // TESSERACT_ROS_KDL_CHAIN_KIN_H
