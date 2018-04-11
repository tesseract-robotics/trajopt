/**
 * @file basic_env.h
 * @brief Basic low-level environment with collision and distance functions.
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
#ifndef ROS_BASIC_ENV_H
#define ROS_BASIC_ENV_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <geometric_shapes/shape_operations.h>
#include <tesseract_core/basic_env.h>
#include <map>
#include <unordered_map>
#include <urdf/model.h>
#include <srdfdom/model.h>

namespace tesseract
{

namespace tesseract_ros
{

struct ROSAllowedCollisionMatrix : public AllowedCollisionMatrix
{
  /**
   * @brief Disable collision between two collision objects
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   * @param reason The reason for disabling collison
   */
  virtual void addAllowedCollision(const std::string &link_name1, const std::string &link_name2, const std::string &reason)
  {
    lookup_table_[link_name1 + link_name2] = reason;
    lookup_table_[link_name2 + link_name1] = reason;
  }

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string &link_name1, const std::string &link_name2)
  {
    lookup_table_.erase(link_name1 + link_name2);
    lookup_table_.erase(link_name2 + link_name1);
  }

  bool isCollisionAllowed(const std::string &link_name1, const std::string &link_name2) const
  {
    return (lookup_table_.find(link_name1 + link_name2) != lookup_table_.end());
  }

private:
  std::unordered_map<std::string, std::string> lookup_table_;
};
typedef boost::shared_ptr<ROSAllowedCollisionMatrix> ROSAllowedCollisionMatrixPtr;
typedef boost::shared_ptr<const ROSAllowedCollisionMatrix> ROSAllowedCollisionMatrixConstPtr;

/**< @brief Information on how the object is attached to the environment */
struct AttachedBodyInfo
{
  std::string name;                     /**< @brief The name of the attached body (must be unique) */
  std::string parent_link_name;         /**< @brief The name of the link to attach the body */
  std::string object_name;              /**< @brief The name of the AttachableObject being used */
  std::vector<std::string> touch_links; /**< @brief The names of links which the attached body is allowed to be in contact with */
};

/** @brief Contains geometry data for an attachable object */
struct AttachableObjectGeometry
{
  std::vector<shapes::ShapeConstPtr> shapes;  /**< @brief The shape */
  EigenSTL::vector_Affine3d shape_poses;      /**< @brief The pose of the shape */
  EigenSTL::vector_Vector4d shape_colors;     /**< @brief (Optional) The shape color (R, G, B, A) */
};

/** @brief Contains data about an attachable object */
struct AttachableObject
{
  std::string name;                   /**< @brief The name of the attachable object */
  AttachableObjectGeometry visual;    /**< @brief The objects visual geometry */
  AttachableObjectGeometry collision; /**< @brief The objects collision geometry */
};
typedef boost::shared_ptr<AttachableObject> AttachableObjectPtr;
typedef boost::shared_ptr<const AttachableObject> AttachableObjectConstPtr;

/** @brief Contains data representing an attached body */
struct AttachedBody
{
   AttachedBodyInfo info;        /**< @brief Information on how the object is attached to the environment */
   AttachableObjectConstPtr obj; /**< @brief The attached bodies object data */
};
typedef boost::shared_ptr<AttachedBody> AttachedBodyPtr;
typedef boost::shared_ptr<const AttachedBody> AttachedBodyConstPtr;

/** @brief ObjectColorMap Stores Object color in a 4d vector as RGBA*/
typedef std::unordered_map<std::string, std_msgs::ColorRGBA> ObjectColorMap;
typedef boost::shared_ptr<ObjectColorMap> ObjectColorMapPtr;
typedef boost::shared_ptr<const ObjectColorMap> ObjectColorMapConstPtr;
typedef std::unordered_map<std::string, AttachedBodyConstPtr> AttachedBodyConstPtrMap;
typedef std::unordered_map<std::string, AttachableObjectConstPtr> AttachableObjectConstPtrMap;

class ROSBasicEnv : public BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ROSBasicEnv() : allowed_collision_matrix_(new ROSAllowedCollisionMatrix()) {}

  /**
   * @brief A a manipulator as a kinematic chain
   * @param base_link The base link of the chain
   * @param tip_link The tip link of the chain
   * @param name The name of the manipulator. This must be unique.
   * @return true if successfully created, otherwise false.
   */
  virtual bool addManipulator(const std::string &base_link, const std::string &tip_link, const std::string &manipulator_name) = 0;

  /**
   * @brief Add object so it may be attached/detached.
   *
   * This object is not part of the environment until attached to a link.
   *
   * @param attachable_object The object information
   */
  virtual void addAttachableObject(const AttachableObjectConstPtr attachable_object) = 0;

  /**
   * @brief Remove object from list of available objects to be attached
   *
   * This will not remove any bodies using the object.
   *
   * @param name The name of the object to be removed
   */
  virtual void removeAttachableObject(const std::string& name) = 0;

  /**
   * @brief Get a map of available attachable objects
   * @return A map of attachable objects
   */
  virtual const AttachableObjectConstPtrMap& getAttachableObjects() const = 0;

  /** @brief This will remove all attachable objects */
  virtual void clearAttachableObjects() = 0;

  /**
   * @brief Get attached body
   * @param name The name of the body
   * @return AttachedBody
   */
  virtual const AttachedBodyConstPtr getAttachedBody(const std::string& name) const = 0;

  /**
   * @brief Get all attached bodies
   * @return A map of attached bodies
   */
  virtual const AttachedBodyConstPtrMap& getAttachedBodies() const = 0;

  /**
   * @brief Attached an attachable object to the environment
   * @param attached_body Information of attaching creating the attached body
   */
  virtual void attachBody(const AttachedBodyInfo &attached_body_info) = 0;

  /**
   * @brief Detach an attachable object from the environment
   * @param name The name given to the Attached Body when attached
   */
  virtual void detachBody(const std::string &name) = 0;

  /** @brief This will detach all bodies */
  virtual void clearAttachedBodies() = 0;

  /** @brief Get a map of object names to colors */
  virtual ObjectColorMapConstPtr getKnownObjectColors() const = 0;

  virtual const urdf::ModelInterfaceConstSharedPtr getURDF() const = 0;

  virtual const srdf::ModelConstSharedPtr getSRDF() const = 0;

  /////////////////////////
  // Implemented Methods //
  /////////////////////////

  AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const
  {
    return allowed_collision_matrix_;
  }

  /** @brief Get the allowed collision matrix non const */
  virtual ROSAllowedCollisionMatrixPtr getAllowedCollisionMatrixNonConst() const
  {
    return allowed_collision_matrix_;
  }

protected:
  ROSAllowedCollisionMatrixPtr allowed_collision_matrix_; /**< The allowed collision matrix used during collision checking */

}; // class ROSBasicEnv

typedef boost::shared_ptr<ROSBasicEnv> ROSBasicEnvPtr;
typedef boost::shared_ptr<const ROSBasicEnv> ROSBasicEnvConstPtr;

} //namespace tesseract_ros
} //namespace tesseract

#endif // ROS_BASIC_ENV_H
