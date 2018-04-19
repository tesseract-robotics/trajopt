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

#include <tesseract_msgs/TesseractState.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_ros/ros_basic_env.h>
#include <octomap_msgs/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>


namespace tesseract
{

namespace tesseract_ros
{

static inline
void attachableObjectToAttachableObjectMsg(tesseract_msgs::AttachableObject& ao_msg, const tesseract_ros::AttachableObject& ao)
{
  ao_msg.operation = tesseract_msgs::AttachableObject::ADD;
  ao_msg.name = ao.name;

  // Visual Geometry
  for (auto i = 0; i < ao.visual.shapes.size(); ++i)
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
  for (auto i = 0; i < ao.collision.shapes.size(); ++i)
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
    }
  }
}

static inline
void attachableObjectToAttachableObjectMsg(tesseract_msgs::AttachableObjectPtr ao_msg, const tesseract_ros::AttachableObject& ao)
{
  attachableObjectToAttachableObjectMsg(*ao_msg, ao);
}

static inline
void attachableObjectMsgToAttachableObject(tesseract_ros::AttachableObject& ao, const tesseract_msgs::AttachableObject& ao_msg)
{
  ao.name = ao_msg.name;

  // Visual Geometry
  for (auto i = 0; i < ao_msg.visual.primitives.size(); ++i)
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

  for (auto i = 0; i < ao_msg.visual.meshes.size(); ++i)
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

  for (auto i = 0; i < ao_msg.visual.planes.size(); ++i)
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

  for (auto i = 0; i < ao_msg.visual.octomaps.size(); ++i)
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
  for (auto i = 0; i < ao_msg.collision.primitives.size(); ++i)
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
  }

  for (auto i = 0; i < ao_msg.collision.meshes.size(); ++i)
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
  }

  for (auto i = 0; i < ao_msg.collision.planes.size(); ++i)
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
  }

  for (auto i = 0; i < ao_msg.collision.octomaps.size(); ++i)
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
  }
}

static inline
void attachableObjectMsgToAttachableObject(tesseract_ros::AttachableObjectPtr ao, const tesseract_msgs::AttachableObject& ao_msg)
{
  attachableObjectMsgToAttachableObject(*ao, ao_msg);
}

static inline
void attachedBodyToAttachedBodyInfoMsg(tesseract_msgs::AttachedBodyInfo& ab_info_msg, const tesseract_ros::AttachedBody& ab)
{
  ab_info_msg.operation = tesseract_msgs::AttachedBodyInfo::ADD;
  ab_info_msg.name = ab.info.name;
  ab_info_msg.object_name = ab.info.object_name;
  ab_info_msg.parent_link_name = ab.info.parent_link_name;
  ab_info_msg.touch_links = ab.info.touch_links;
}

static inline
void attachedBodyToAttachedBodyInfoMsg(tesseract_msgs::AttachedBodyInfoPtr ab_info_msg, const tesseract_ros::AttachedBody& ab)
{
  attachedBodyToAttachedBodyInfoMsg(*ab_info_msg, ab);
}

static inline
void attachedBodyInfoMsgToAttachedBodyInfo(tesseract_ros::AttachedBodyInfo& ab_info, const tesseract_msgs::AttachedBodyInfo& body)
{
  ab_info.name = body.name;
  ab_info.object_name = body.object_name;
  ab_info.parent_link_name = body.parent_link_name;
  ab_info.touch_links = body.touch_links;
}

static inline
void tesseractEnvStateToJointStateMsg(sensor_msgs::JointState& joint_state, const tesseract_ros::EnvState& state)
{
  for (const auto& joint : state.joints)
  {
    joint_state.name.push_back(joint.first);
    joint_state.position.push_back(joint.second);
  }
}

static inline
void tesseractEnvStateToJointStateMsg(sensor_msgs::JointStatePtr joint_state, const tesseract_ros::EnvState& state)
{
  tesseractEnvStateToJointStateMsg(*joint_state, state);
}

static inline
void tesseractToTesseractStateMsg(tesseract_msgs::TesseractState& state_msg, const tesseract_ros::ROSBasicEnv& env)
{
  for (const auto& ao : env.getAttachableObjects())
  {
    tesseract_msgs::AttachableObject ao_msg;
    attachableObjectToAttachableObjectMsg(ao_msg, *ao.second);
    state_msg.attachable_objects.push_back(ao_msg);
  }

  for (const auto& ab : env.getAttachedBodies())
  {
    tesseract_msgs::AttachedBodyInfo ab_msg;
    attachedBodyToAttachedBodyInfoMsg(ab_msg, *ab.second);
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
bool processTesseractStateMsg(tesseract_ros::ROSEnvBase& env, const tesseract_msgs::TesseractState& state_msg)
{
  bool success = true;
  for (const auto& ao_msg : state_msg.attachable_objects)
  {
    if (ao_msg.operation == tesseract_msgs::AttachableObject::REMOVE)
    {
      env.removeAttachableObject(ao_msg.name);
    }
    else if (ao_msg.operation == tesseract_msgs::AttachableObject::ADD)
    {
      tesseract_ros::AttachableObjectPtr ao(new tesseract_ros::AttachableObject());
      attachableObjectMsgToAttachableObject(ao, ao_msg);
      env.addAttachableObject(ao);
    }
    else if (ao_msg.operation == tesseract_msgs::AttachableObject::APPEND)
    {
      ROS_ERROR("AttachableObject APPEND operation currently not implemented.");
      success = false;
    }
  }

  for (const auto& ab_msg : state_msg.attached_bodies)
  {
    if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::REMOVE)
    {
      env.detachBody(ab_msg.name);
    }
    else if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::ADD)
    {
      tesseract_ros::AttachedBodyInfo ab_info;
      tesseract_ros::attachedBodyInfoMsgToAttachedBodyInfo(ab_info, ab_msg);
      env.attachBody(ab_info);
    }
    else if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::MOVE)
    {
      ROS_ERROR("AttachedBody MOVE operation currently not implemented.");
      success = false;
    }
  }

  std::unordered_map<std::string, double> joints;
  for (auto i = 0; i < state_msg.joint_state.name.size(); ++i)
  {
    joints[state_msg.joint_state.name[i]] =  state_msg.joint_state.position[i];
  }
  env.setState(joints);

  return success;
}

static inline
bool processTesseractStateMsg(tesseract_ros::ROSEnvBasePtr env, const tesseract_msgs::TesseractState& state_msg)
{
  return processTesseractStateMsg(*env, state_msg);
}

static inline
void tesseractContactResultToContactResultMsg(tesseract_msgs::ContactResult& contact_result_msg, const tesseract::ContactResult& contact_result)
{
  contact_result_msg.distance = contact_result.distance;
  contact_result_msg.link_names[0] = contact_result.link_names[0];
  contact_result_msg.link_names[1] = contact_result.link_names[1];
  contact_result_msg.attached_link_names[0] = contact_result.attached_link_names[0];
  contact_result_msg.attached_link_names[1] = contact_result.attached_link_names[1];
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
  contact_result_msg.valid = contact_result.valid;

  if (contact_result.body_types[0] == BodyTypes::ROBOT_ATTACHED)
    contact_result_msg.body_types[0] = 1;
  else
    contact_result_msg.body_types[0] = 0;

  if (contact_result.body_types[1] == BodyTypes::ROBOT_ATTACHED)
    contact_result_msg.body_types[1] = 1;
  else
    contact_result_msg.body_types[1] = 0;

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

}
}
#endif // TESSERACT_ROS_UTILS_H
