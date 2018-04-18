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
#ifndef TESSERACT_ROS_BULLET_ENV_SINGLETON_H
#define TESSERACT_ROS_BULLET_ENV_SINGLETON_H
#include <tesseract_ros/bullet/bullet_utils.h>
#include <tesseract_ros/ros_basic_env.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <boost/thread/mutex.hpp>

namespace tesseract
{
namespace tesseract_ros
{

class BulletEnvSingleton : public ROSBasicEnvSingleton
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BulletEnvSingleton() : ROSBasicEnvSingleton(), initialized_(false), allowed_collision_matrix_(new ROSAllowedCollisionMatrix()) {}

  bool init(const urdf::ModelInterfaceConstSharedPtr urdf_model);
  bool init(const urdf::ModelInterfaceConstSharedPtr urdf_model, const srdf::ModelConstSharedPtr srdf_model);

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const { return initialized_; }

  void setContactRequest(const ContactRequestBase &req);

  const ContactRequestBase& getContactRequest() const { return request_; }

  void calcDistancesDiscrete(ContactResultVector &dists);

  void calcCollisionsDiscrete(ContactResultVector &collisions);

  EnvStateConstPtr getState() const { return current_state_; }

  void setState(const std::unordered_map<std::string, double> &joints);
  void setState(const std::vector<std::string> &joint_names, const std::vector<double> &joint_values);
  void setState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values);

  std::vector<std::string> getJointNames() const { return joint_names_; }

  Eigen::VectorXd getCurrentJointValues() const;

  Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const;

  const std::string& getRootLinkName() const { return kdl_tree_->getRootSegment()->second.segment.getName(); }

  std::vector<std::string> getLinkNames() const { return link_names_; }

  vector_Affine3d getLinkTransforms() const;

  Eigen::Affine3d getLinkTransform(const std::string& link_name) const;

  bool hasManipulator(const std::string &manipulator_name) const;

  BasicKinConstPtr getManipulator(const std::string &manipulator_name) const;

  bool addManipulator(const std::string &base_link, const std::string &tip_link, const std::string &manipulator_name);

  ObjectColorMapConstPtr getKnownObjectColors() const { return object_colors_; }

  void addAttachableObject(const AttachableObjectConstPtr attachable_object);

  void removeAttachableObject(const std::string& name);

  const AttachableObjectConstPtrMap& getAttachableObjects() const { return attachable_objects_; }

  void clearAttachableObjects();

  const AttachedBodyConstPtr getAttachedBody(const std::string& name) const;

  const AttachedBodyConstPtrMap& getAttachedBodies() const { return attached_bodies_; }

  void attachBody(const AttachedBodyInfo &attached_body_info);

  void detachBody(const std::string &name);

  void clearAttachedBodies();

  const urdf::ModelInterfaceConstSharedPtr getURDF() const { return model_; }

  const srdf::ModelConstSharedPtr getSRDF() const { return srdf_model_; }

  AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const { return allowed_collision_matrix_; }

  ROSAllowedCollisionMatrixPtr getAllowedCollisionMatrixNonConst() const { return allowed_collision_matrix_; }

private:
  bool initialized_;                                                /**< Identifies if the object has been initialized */
  urdf::ModelInterfaceConstSharedPtr model_;                        /**< URDF MODEL */
  srdf::ModelConstSharedPtr srdf_model_;                            /**< SRDF MODEL */
  boost::shared_ptr<const KDL::Tree> kdl_tree_;                     /**< KDL tree object */
  BulletManager manager_;                                           /**< Collision object manager */
  ContactRequestBase request_;                                      /**< Contact request information */
  std::vector<std::string> active_objects_;                          /**< A list of active objects ot check for contact */
  EnvStatePtr current_state_;                                       /**< Current state of the robot */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_;      /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                     /**< The kdl joint array */
  AttachedBodyConstPtrMap attached_bodies_;                         /**< A map of attached bodies */
  AttachableObjectConstPtrMap attachable_objects_;                  /**< A map of objects that can be attached/detached from environment */
  std::unordered_map<std::string, BasicKinConstPtr> manipulators_;  /**< A map of manipulator names to kinematics object */
  std::vector<std::string> link_names_;                             /**< A vector of link names */
  std::vector<std::string> joint_names_;                            /**< A vector of joint names */
  ObjectColorMapConstPtr object_colors_;                            /**< A map of objects to color */
  ROSAllowedCollisionMatrixPtr allowed_collision_matrix_;           /**< The allowed collision matrix used during collision checking */

  void calculateTransforms(std::unordered_map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const;
  void calculateTransformsHelper(std::unordered_map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray &q, const std::string &joint_name, const double &joint_value) const;

  std::string getManipulatorName(const std::vector<std::string> &joint_names) const;

  void updateBulletObjects();

};
typedef boost::shared_ptr<BulletEnvSingleton> BulletEnvSingletonPtr;
typedef boost::shared_ptr<const BulletEnvSingleton> BulletEnvSingletonConstPtr;
}
}

#endif // TESSERACT_ROS_BULLET_ENV_SINGLETON_H
