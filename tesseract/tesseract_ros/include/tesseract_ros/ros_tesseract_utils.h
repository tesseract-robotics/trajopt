/**
 * @file ros_tesseract_utils.h
 * @brief Tesseract ROS utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_ROS_UTILS_H
#define TESSERACT_ROS_UTILS_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <octomap_msgs/conversions.h>
#pragma GCC diagnostic pop

#include <tesseract_msgs/TesseractState.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_ros/ros_basic_env.h>
#include <tesseract_core/basic_types.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <std_msgs/Int32.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>


namespace tesseract
{

namespace tesseract_ros
{

static inline
bool isMsgEmpty(const sensor_msgs::JointState& msg)
{
  return msg.name.empty() &&
         msg.position.empty() &&
         msg.velocity.empty() &&
         msg.effort.empty();
}

static inline
bool isMsgEmpty(const sensor_msgs::MultiDOFJointState& msg)
{
  return msg.joint_names.empty() &&
         msg.transforms.empty() &&
         msg.twist.empty() &&
         msg.wrench.empty();
}

static inline
void attachableObjectToAttachableObjectMsg(tesseract_msgs::AttachableObject& ao_msg, const AttachableObject& ao)
{
  ao_msg.operation = tesseract_msgs::AttachableObject::ADD;
  ao_msg.name = ao.name;

  // Visual Geometry
  for (auto i = 0u; i < ao.visual.shapes.size(); ++i)
  {
    if (ao.visual.shapes[i]->type == shapes::MESH)
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.visual.shapes[i].get(), shape_msg);
      ao_msg.visual.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.mesh_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.visual.shape_colors[i](0);
        color.g = ao.visual.shape_colors[i](1);
        color.b = ao.visual.shape_colors[i](2);
        color.a = ao.visual.shape_colors[i](3);
        ao_msg.visual.mesh_colors.push_back(color);
      }
    }
    else if (ao.visual.shapes[i]->type == shapes::OCTREE)
    {
      octomap_msgs::Octomap octomap_msg;
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(ao.visual.shapes[i].get());
      octomap_msgs::fullMapToMsg(*o->octree, octomap_msg);
      ao_msg.visual.octomaps.push_back(octomap_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.octomap_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.visual.shape_colors[i](0);
        color.g = ao.visual.shape_colors[i](1);
        color.b = ao.visual.shape_colors[i](2);
        color.a = ao.visual.shape_colors[i](3);
        ao_msg.visual.octomap_colors.push_back(color);
      }
    }
    else if (ao.visual.shapes[i]->type == shapes::PLANE)
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.visual.shapes[i].get(), shape_msg);
      ao_msg.visual.planes.push_back(boost::get<shape_msgs::Plane>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.plane_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.visual.shape_colors[i](0);
        color.g = ao.visual.shape_colors[i](1);
        color.b = ao.visual.shape_colors[i](2);
        color.a = ao.visual.shape_colors[i](3);
        ao_msg.visual.plane_colors.push_back(color);
      }
    }
    else
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.visual.shapes[i].get(), shape_msg);
      ao_msg.visual.primitives.push_back(boost::get<shape_msgs::SolidPrimitive>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.primitive_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.visual.shape_colors[i](0);
        color.g = ao.visual.shape_colors[i](1);
        color.b = ao.visual.shape_colors[i](2);
        color.a = ao.visual.shape_colors[i](3);
        ao_msg.visual.primitive_colors.push_back(color);
      }
    }
  }

  // Collision Geometry
  for (auto i = 0u; i < ao.collision.shapes.size(); ++i)
  {
    if (ao.collision.shapes[i]->type == shapes::MESH)
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.collision.shapes[i].get(), shape_msg);
      ao_msg.collision.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.mesh_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.collision.shape_colors[i](0);
        color.g = ao.collision.shape_colors[i](1);
        color.b = ao.collision.shape_colors[i](2);
        color.a = ao.collision.shape_colors[i](3);
        ao_msg.collision.mesh_colors.push_back(color);
      }

      std_msgs::Int32 cot;
      cot.data = ao.collision.collision_object_types[i];
      ao_msg.collision.mesh_collision_object_types.push_back(cot);
    }
    else if (ao.collision.shapes[i]->type == shapes::OCTREE)
    {
      octomap_msgs::Octomap octomap_msg;
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(ao.collision.shapes[i].get());
      octomap_msgs::fullMapToMsg(*o->octree, octomap_msg);
      ao_msg.collision.octomaps.push_back(octomap_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.octomap_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.collision.shape_colors[i](0);
        color.g = ao.collision.shape_colors[i](1);
        color.b = ao.collision.shape_colors[i](2);
        color.a = ao.collision.shape_colors[i](3);
        ao_msg.collision.octomap_colors.push_back(color);
      }

      std_msgs::Int32 cot;
      cot.data = ao.collision.collision_object_types[i];
      ao_msg.collision.octomap_collision_object_types.push_back(cot);
    }
    else if (ao.collision.shapes[i]->type == shapes::PLANE)
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.collision.shapes[i].get(), shape_msg);
      ao_msg.collision.planes.push_back(boost::get<shape_msgs::Plane>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.plane_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.collision.shape_colors[i](0);
        color.g = ao.collision.shape_colors[i](1);
        color.b = ao.collision.shape_colors[i](2);
        color.a = ao.collision.shape_colors[i](3);
        ao_msg.collision.plane_colors.push_back(color);
      }

      std_msgs::Int32 cot;
      cot.data = ao.collision.collision_object_types[i];
      ao_msg.collision.plane_collision_object_types.push_back(cot);
    }
    else
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.collision.shapes[i].get(), shape_msg);
      ao_msg.collision.primitives.push_back(boost::get<shape_msgs::SolidPrimitive>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.primitive_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = ao.collision.shape_colors[i](0);
        color.g = ao.collision.shape_colors[i](1);
        color.b = ao.collision.shape_colors[i](2);
        color.a = ao.collision.shape_colors[i](3);
        ao_msg.collision.primitive_colors.push_back(color);
      }

      std_msgs::Int32 cot;
      cot.data = ao.collision.collision_object_types[i];
      ao_msg.collision.primitive_collision_object_types.push_back(cot);
    }
  }
}

static inline
void attachableObjectToAttachableObjectMsg(tesseract_msgs::AttachableObjectPtr ao_msg, const AttachableObject& ao)
{
  attachableObjectToAttachableObjectMsg(*ao_msg, ao);
}

static inline
void attachableObjectMsgToAttachableObject(AttachableObject& ao, const tesseract_msgs::AttachableObject& ao_msg)
{
  ao.name = ao_msg.name;

  // Visual Geometry
  for (auto i = 0u; i < ao_msg.visual.primitives.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.visual.primitives[i]));
    ao.visual.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.visual.primitive_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.primitive_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.primitive_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.visual.meshes.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.visual.meshes[i]));
    ao.visual.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.visual.mesh_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.mesh_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.mesh_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.visual.planes.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.visual.planes[i]));
    ao.visual.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.visual.plane_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.plane_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.plane_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.visual.octomaps.size(); ++i)
  {
    std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(ao_msg.visual.octomaps[i])));
    shapes::ShapePtr shape(new shapes::OcTree(om));
    ao.visual.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.visual.octomap_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.octomap_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.octomap_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  // Collision Geometry
  for (auto i = 0u; i < ao_msg.collision.primitives.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.collision.primitives[i]));
    ao.collision.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.collision.primitive_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.primitive_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.primitive_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }

    ao.collision.collision_object_types.push_back((CollisionObjectType)ao_msg.collision.primitive_collision_object_types[i].data);
  }

  for (auto i = 0u; i < ao_msg.collision.meshes.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.collision.meshes[i]));
    ao.collision.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.collision.mesh_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.mesh_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.mesh_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }

    ao.collision.collision_object_types.push_back((CollisionObjectType)ao_msg.collision.mesh_collision_object_types[i].data);
  }

  for (auto i = 0u; i < ao_msg.collision.planes.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.collision.planes[i]));
    ao.collision.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.collision.plane_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.plane_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.plane_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }

    ao.collision.collision_object_types.push_back((CollisionObjectType)ao_msg.collision.plane_collision_object_types[i].data);
  }

  for (auto i = 0u; i < ao_msg.collision.octomaps.size(); ++i)
  {
    std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(ao_msg.collision.octomaps[i])));
    shapes::ShapePtr shape(new shapes::OcTree(om));
    ao.collision.shapes.push_back(shape);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(ao_msg.collision.octomap_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.octomap_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.octomap_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }

    ao.collision.collision_object_types.push_back((CollisionObjectType)ao_msg.collision.octomap_collision_object_types[i].data);
  }
}

static inline
void attachableObjectMsgToAttachableObject(AttachableObjectPtr ao, const tesseract_msgs::AttachableObject& ao_msg)
{
  attachableObjectMsgToAttachableObject(*ao, ao_msg);
}

static inline
void attachedBodyInfoToAttachedBodyInfoMsg(tesseract_msgs::AttachedBodyInfo& ab_info_msg, const AttachedBodyInfo& ab_info)
{
  ab_info_msg.operation = tesseract_msgs::AttachedBodyInfo::ADD;
  ab_info_msg.object_name = ab_info.object_name;
  ab_info_msg.parent_link_name = ab_info.parent_link_name;
  tf::poseEigenToMsg(ab_info.transform, ab_info_msg.transform);
  ab_info_msg.touch_links = ab_info.touch_links;
}

static inline
void attachedBodyInfoToAttachedBodyInfoMsg(tesseract_msgs::AttachedBodyInfoPtr ab_info_msg, const AttachedBodyInfo& ab_info)
{
  attachedBodyInfoToAttachedBodyInfoMsg(*ab_info_msg, ab_info);
}

static inline
void attachedBodyInfoMsgToAttachedBodyInfo(AttachedBodyInfo& ab_info, const tesseract_msgs::AttachedBodyInfo& body)
{
  ab_info.object_name = body.object_name;
  ab_info.parent_link_name = body.parent_link_name;
  tf::poseMsgToEigen(body.transform, ab_info.transform);
  ab_info.touch_links = body.touch_links;
}

static inline
void tesseractEnvStateToJointStateMsg(sensor_msgs::JointState& joint_state, const EnvState& state)
{
  joint_state.header.stamp = ros::Time::now();
  for (const auto& joint : state.joints)
  {
    joint_state.name.push_back(joint.first);
    joint_state.position.push_back(joint.second);
  }
}

static inline
void tesseractEnvStateToJointStateMsg(sensor_msgs::JointStatePtr joint_state, const EnvState& state)
{
  tesseractEnvStateToJointStateMsg(*joint_state, state);
}

static inline
void tesseractToTesseractStateMsg(tesseract_msgs::TesseractState& state_msg, const tesseract_ros::ROSBasicEnv& env)
{
  state_msg.name = env.getName();
  state_msg.urdf_name = env.getURDF()->getName();
  state_msg.is_diff = false;

  for (const auto& ao : env.getAttachableObjects())
  {
    tesseract_msgs::AttachableObject ao_msg;
    attachableObjectToAttachableObjectMsg(ao_msg, *ao.second);
    state_msg.attachable_objects.push_back(ao_msg);
  }

  for (const auto& ab : env.getAttachedBodies())
  {
    tesseract_msgs::AttachedBodyInfo ab_msg;
    attachedBodyInfoToAttachedBodyInfoMsg(ab_msg, ab.second);
    state_msg.attached_bodies.push_back(ab_msg);
  }

  EnvStateConstPtr state = env.getState();
  tesseractEnvStateToJointStateMsg(state_msg.joint_state, *state);
}

static inline
void tesseractToTesseractStateMsg(tesseract_msgs::TesseractStatePtr state_msg, const tesseract_ros::ROSBasicEnv& env)
{
  tesseractToTesseractStateMsg(*state_msg, env);
}

static inline
bool processAttachableObjectMsg(tesseract_ros::ROSBasicEnv& env, const tesseract_msgs::AttachableObject& ao_msg)
{
  if (ao_msg.operation == tesseract_msgs::AttachableObject::REMOVE)
  {
    env.removeAttachableObject(ao_msg.name);
  }
  else if (ao_msg.operation == tesseract_msgs::AttachableObject::ADD)
  {
    AttachableObjectPtr ao(new AttachableObject());
    attachableObjectMsgToAttachableObject(ao, ao_msg);
    env.addAttachableObject(ao);
  }
  else
  {
    ROS_ERROR("AttachableObject, Unknown operation.");
    return false;
  }

  return true;
}

static inline
bool processAttachableObjectMsg(tesseract_ros::ROSBasicEnvPtr env, const tesseract_msgs::AttachableObject& ao_msg)
{
  return processAttachableObjectMsg(*env, ao_msg);
}

static inline
bool processAttachedBodyInfoMsg(tesseract_ros::ROSBasicEnv& env, const tesseract_msgs::AttachedBodyInfo& ab_msg)
{
  if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::REMOVE)
  {
    env.detachBody(ab_msg.object_name);
  }
  else if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::ADD)
  {
    AttachedBodyInfo ab_info;
    tesseract_ros::attachedBodyInfoMsgToAttachedBodyInfo(ab_info, ab_msg);
    env.attachBody(ab_info);
  }
  else if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::MOVE)
  {
    ROS_ERROR("AttachedBody MOVE operation currently not implemented.");
    return false;
  }

  return true;
}

static inline
bool processAttachedBodyInfoMsg(tesseract_ros::ROSBasicEnvPtr env, const tesseract_msgs::AttachedBodyInfo& ab_msg)
{
  return processAttachedBodyInfoMsg(*env, ab_msg);
}

static inline
bool processTesseractStateMsg(tesseract_ros::ROSBasicEnv& env, const tesseract_msgs::TesseractState& state_msg)
{
  bool success = true;

  if (!state_msg.is_diff)
  {
    env.clearAttachedBodies();
    env.clearAttachableObjects();
    env.clearKnownObjectColors();
  }

  if (!isMsgEmpty(state_msg.joint_state))
  {
    std::unordered_map<std::string, double> joints;
    for (auto i = 0u; i < state_msg.joint_state.name.size(); ++i)
    {
      joints[state_msg.joint_state.name[i]] = state_msg.joint_state.position[i];
    }
    env.setState(joints);
  }

  for (const auto& ao_msg : state_msg.attachable_objects)
  {
    if (!processAttachableObjectMsg(env, ao_msg))
      success = false;
  }

  for (const auto& ab_msg : state_msg.attached_bodies)
  {
    if (!processAttachedBodyInfoMsg(env, ab_msg))
      success = false;
  }

  return success;
}

static inline
bool processTesseractStateMsg(tesseract_ros::ROSBasicEnvPtr env, const tesseract_msgs::TesseractState& state_msg)
{
  return processTesseractStateMsg(*env, state_msg);
}

static inline
void tesseractContactResultToContactResultMsg(tesseract_msgs::ContactResult& contact_result_msg, const tesseract::ContactResult& contact_result)
{
  contact_result_msg.distance = contact_result.distance;
  contact_result_msg.link_names[0] = contact_result.link_names[0];
  contact_result_msg.link_names[1] = contact_result.link_names[1];
  contact_result_msg.normal.x = contact_result.normal[0];
  contact_result_msg.normal.y = contact_result.normal[1];
  contact_result_msg.normal.z = contact_result.normal[2];
  contact_result_msg.nearest_points[0].x = contact_result.nearest_points[0][0];
  contact_result_msg.nearest_points[0].y = contact_result.nearest_points[0][1];
  contact_result_msg.nearest_points[0].z = contact_result.nearest_points[0][2];
  contact_result_msg.nearest_points[1].x = contact_result.nearest_points[1][0];
  contact_result_msg.nearest_points[1].y = contact_result.nearest_points[1][1];
  contact_result_msg.nearest_points[1].z = contact_result.nearest_points[1][2];
  contact_result_msg.cc_time = contact_result.cc_time;
  contact_result_msg.cc_nearest_points[0].x = contact_result.cc_nearest_points[0][0];
  contact_result_msg.cc_nearest_points[0].y = contact_result.cc_nearest_points[0][1];
  contact_result_msg.cc_nearest_points[0].z = contact_result.cc_nearest_points[0][2];
  contact_result_msg.cc_nearest_points[1].x = contact_result.cc_nearest_points[1][0];
  contact_result_msg.cc_nearest_points[1].y = contact_result.cc_nearest_points[1][1];
  contact_result_msg.cc_nearest_points[1].z = contact_result.cc_nearest_points[1][2];

  contact_result_msg.type_id[0] = contact_result.type_id[0];
  contact_result_msg.type_id[1] = contact_result.type_id[1] ;

  if (contact_result.cc_type == ContinouseCollisionTypes::CCType_Time0)
    contact_result_msg.cc_type = 1;
  else if (contact_result.cc_type == ContinouseCollisionTypes::CCType_Time1)
    contact_result_msg.cc_type = 2;
  else if (contact_result.cc_type == ContinouseCollisionTypes::CCType_Between)
    contact_result_msg.cc_type = 3;
  else
    contact_result_msg.cc_type = 0;
}

static inline
void tesseractContactResultToContactResultMsg(tesseract_msgs::ContactResultPtr contact_result_msg, const tesseract::ContactResult& contact_result)
{
  tesseractContactResultToContactResultMsg(*contact_result_msg, contact_result);
}

static inline
void getActiveLinkNamesRecursive(std::vector<std::string>& active_links, const urdf::LinkConstSharedPtr urdf_link, bool active)
{
  // recursively get all active links
  if (active)
  {
    active_links.push_back(urdf_link->name);
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      getActiveLinkNamesRecursive(active_links, urdf_link->child_links[i], active);
    }
  }
  else
  {
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      if ((urdf_link->parent_joint) && (urdf_link->parent_joint->type != urdf::Joint::FIXED))
        getActiveLinkNamesRecursive(active_links, urdf_link->child_links[i], true);
      else
        getActiveLinkNamesRecursive(active_links, urdf_link->child_links[i], active);
    }
  }
}

inline
shapes::ShapePtr constructShape(const urdf::Geometry* geom)
{
  shapes::Shape* result = NULL;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
      result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
    case urdf::Geometry::CYLINDER:
      result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                    static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        result = m;
      }
    }
    break;
    default:
      ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
      break;
  }

  return shapes::ShapePtr(result);
}

inline Eigen::Affine3d urdfPose2Affine3d(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Affine3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q.toRotationMatrix());
  return af;
}

}
}
#endif // TESSERACT_ROS_UTILS_H
