#ifndef ROS_TESSERACT_CONVERSIONS_H
#define ROS_TESSERACT_CONVERSIONS_H

#include <tesseract_msgs/TesseractState.h>
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
void attachableObjectToAttachableObjectMsg(const tesseract_msgs::AttachableObjectPtr ao_msg, const tesseract_ros::AttachableObject& ao)
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
void attachableObjectMsgToAttachableObject(const tesseract_ros::AttachableObjectPtr ao, const tesseract_msgs::AttachableObject& ao_msg)
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
void attachedBodyToAttachedBodyInfoMsg(const tesseract_msgs::AttachedBodyInfoPtr ab_info_msg, const tesseract_ros::AttachedBody& ab)
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
void tesseractEnvStateToJointStateMsg(const sensor_msgs::JointStatePtr joint_state, const tesseract_ros::EnvState& state)
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

  tesseractEnvStateToJointStateMsg(state_msg.joint_state, *env.getState());
}

static inline
void tesseractToTesseractStateMsg(const tesseract_msgs::TesseractStatePtr state_msg, const tesseract_ros::ROSBasicEnv& env)
{
  tesseractToTesseractStateMsg(*state_msg, env);
}

static inline
void processTesseractStateMsg(tesseract_ros::ROSBasicEnv& env, const tesseract_msgs::TesseractState& state_msg)
{
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
    }
  }

  std::unordered_map<std::string, double> joints;
  for (auto i = 0; i < state_msg.joint_state.name.size(); ++i)
  {
    joints[state_msg.joint_state.name[i]] =  state_msg.joint_state.position[i];
  }
  env.setState(joints);
}

static inline
void processTesseractStateMsg(const tesseract_ros::ROSBasicEnvPtr env, const tesseract_msgs::TesseractState& state_msg)
{
  processTesseractStateMsg(*env, state_msg);
}

}
}
#endif // ROS_TESSERACT_CONVERSIONS_H
