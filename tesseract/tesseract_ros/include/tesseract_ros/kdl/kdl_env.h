/**
 * @file bullet_env.h
 * @brief Tesseract ROS Bullet environment implementation.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TESSERACT_ROS_BULLET_ENV_H
#define TESSERACT_ROS_BULLET_ENV_H

#include <tesseract_collision/contact_checker_base.h>
#include <tesseract_ros/ros_basic_env.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <pluginlib/class_loader.hpp>

namespace tesseract
{
namespace tesseract_ros
{
typedef pluginlib::ClassLoader<ContactCheckerBase> ContactCheckerBasePluginLoader;
typedef std::shared_ptr<ContactCheckerBasePluginLoader> ContactCheckerBasePluginLoaderPtr;

class KDLEnv : public ROSBasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLEnv() : ROSBasicEnv(), initialized_(false), allowed_collision_matrix_(new AllowedCollisionMatrix())
  {
    is_contact_allowed_fn_ = std::bind(&tesseract::tesseract_ros::KDLEnv::defaultIsContactAllowedFn,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2);
    contact_checker_loader_.reset(new ContactCheckerBasePluginLoader("tesseract_collision",
                                                                     "tesseract::"
                                                                     "ContactCheckerBase"));
  }

  bool init(urdf::ModelInterfaceConstSharedPtr urdf_model) override;
  bool init(urdf::ModelInterfaceConstSharedPtr urdf_model, srdf::ModelConstSharedPtr srdf_model) override;

  void setName(const std::string& name) override { name_ = name; }
  const std::string& getName() const override { return name_; }
  bool checkInitialized() const override { return initialized_; }
  void calcDistancesDiscrete(const ContactRequest& req,
                             const std::vector<std::string>& joint_names,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                             ContactResultMap& contacts) const override;

  void calcDistancesContinuous(const ContactRequest& req,
                               const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values1,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values2,
                               ContactResultMap& contacts) const override;

  void calcCollisionsDiscrete(const ContactRequest& req,
                              const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              ContactResultMap& contacts) const override;

  void calcCollisionsContinuous(const ContactRequest& req,
                                const std::vector<std::string>& joint_names,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_values1,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_values2,
                                ContactResultMap& contacts) const override;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string>& joint_names,
                                          const std::vector<std::string>& link_names,
                                          const Eigen::Ref<const TrajArray>& traj,
                                          ContactResultMap& contacts) const override;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string>& joint_names,
                                          const std::vector<std::string>& link_names,
                                          const Eigen::Ref<const TrajArray>& traj,
                                          ContactResult& contacts) const override;

  EnvStateConstPtr getState() const override { return current_state_; }
  void setState(const std::unordered_map<std::string, double>& joints) override;
  void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) override;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;

  EnvStatePtr getState(const std::unordered_map<std::string, double>& joints) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names,
                       const std::vector<double>& joint_values) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;

  std::vector<std::string> getJointNames() const override { return joint_names_; }
  Eigen::VectorXd getCurrentJointValues() const override;

  Eigen::VectorXd getCurrentJointValues(const std::string& manipulator_name) const override;

  const std::string& getRootLinkName() const override { return kdl_tree_->getRootSegment()->second.segment.getName(); }
  std::vector<std::string> getLinkNames() const override { return link_names_; }
  std::vector<std::string> getActiveLinkNames() const override { return active_link_names_; }
  vector_Affine3d getLinkTransforms() const override;

  const Eigen::Affine3d& getLinkTransform(const std::string& link_name) const override;

  bool hasManipulator(const std::string& manipulator_name) const override;

  BasicKinConstPtr getManipulator(const std::string& manipulator_name) const override;

  bool addManipulator(const std::string& base_link,
                      const std::string& tip_link,
                      const std::string& manipulator_name) override;

  bool addManipulator(const std::vector<std::string>& joint_names, const std::string& manipulator_name) override;

  ObjectColorMapConstPtr getKnownObjectColors() const override { return object_colors_; }
  void clearKnownObjectColors() override { object_colors_->clear(); }
  void addAttachableObject(const AttachableObjectConstPtr attachable_object) override;

  void removeAttachableObject(const std::string& name) override;

  const AttachableObjectConstPtrMap& getAttachableObjects() const override { return attachable_objects_; }
  void clearAttachableObjects() override;

  const AttachedBodyInfo& getAttachedBody(const std::string& name) const override;

  const AttachedBodyInfoMap& getAttachedBodies() const override { return attached_bodies_; }
  void attachBody(const AttachedBodyInfo& attached_body_info) override;

  void detachBody(const std::string& name) override;

  void clearAttachedBodies() override;

  urdf::ModelInterfaceConstSharedPtr getURDF() const override { return urdf_model_; }
  srdf::ModelConstSharedPtr getSRDF() const override { return srdf_model_; }
  AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const override { return allowed_collision_matrix_; }
  AllowedCollisionMatrixPtr getAllowedCollisionMatrixNonConst() override { return allowed_collision_matrix_; }
  IsContactAllowedFn getIsContactAllowedFn() const override { return is_contact_allowed_fn_; }
  void setIsContactAllowedFn(IsContactAllowedFn fn) override { is_contact_allowed_fn_ = fn; }
  void loadContactCheckerPlugin(const std::string& plugin) override;

  // Methods used for environment contact monitors
  void setContactRequest(const ContactRequest& req) override { contact_checker_->setContactRequest(req); }
  void calcDistancesDiscrete(ContactResultMap& contacts) override { contact_checker_->calcDistancesDiscrete(contacts); }
  void calcCollisionsDiscrete(ContactResultMap& contacts) override
  {
    contact_checker_->calcCollisionsDiscrete(contacts);
  }

private:
  bool initialized_;                                           /**< Identifies if the object has been initialized */
  std::string name_;                                           /**< Name of the environment (may be empty) */
  urdf::ModelInterfaceConstSharedPtr urdf_model_;              /**< URDF MODEL */
  srdf::ModelConstSharedPtr srdf_model_;                       /**< SRDF MODEL */
  std::shared_ptr<const KDL::Tree> kdl_tree_;                  /**< KDL tree object */
  EnvStatePtr current_state_;                                  /**< Current state of the robot */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */
  AttachedBodyInfoMap attached_bodies_;                        /**< A map of attached bodies */
  AttachableObjectConstPtrMap
      attachable_objects_; /**< A map of objects that can be attached/detached from environment */
  std::unordered_map<std::string, BasicKinPtr> manipulators_; /**< A map of manipulator names to kinematics object */
  std::vector<std::string> link_names_;                       /**< A vector of link names */
  std::vector<std::string> joint_names_;                      /**< A vector of joint names */
  std::vector<std::string> active_link_names_;                /**< A vector of active link names */
  ObjectColorMapPtr object_colors_;                           /**< A map of objects to color */
  AllowedCollisionMatrixPtr
      allowed_collision_matrix_;          /**< The allowed collision matrix used during collision checking */
  ContactCheckerBasePtr contact_checker_; /**< The contact checker object */
  IsContactAllowedFn
      is_contact_allowed_fn_; /**< The function used to determine if two objects are allowed in collision */
  ContactCheckerBasePluginLoaderPtr contact_checker_loader_; /**< The contact checker loader */

  bool defaultIsContactAllowedFn(const std::string& link_name1, const std::string& link_name2) const;

  void calculateTransforms(TransformMap& transforms,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Affine3d& parent_frame) const;

  void calculateTransformsHelper(TransformMap& transforms,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Affine3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  std::string getManipulatorName(const std::vector<std::string>& joint_names) const;
};
typedef std::shared_ptr<KDLEnv> KDLEnvPtr;
typedef std::shared_ptr<const KDLEnv> KDLEnvConstPtr;
}
}

#endif  // TESSERACT_ROS_BULLET_ENV_H
