/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/filesystem.hpp>

#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgreRibbonTrail.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <ros/console.h>

#include <resource_retriever/retriever.h>
#include <urdf_model/link.h>
#include <urdf_model/model.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>

#include "rviz/load_resource.h"
#include "rviz/mesh_loader.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/mesh_shape.h"
#include "rviz/ogre_helpers/object.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"
#pragma GCC diagnostic pop

#include "tesseract_rviz/render_tools/env/robot.h"
#include "tesseract_rviz/render_tools/env/robot_joint.h"
#include "tesseract_rviz/render_tools/env/robot_link.h"
#include <geometric_shapes/mesh_operations.h>
#include <tesseract_core/basic_types.h>

namespace fs = boost::filesystem;

namespace tesseract_rviz
{
class RobotLinkSelectionHandler : public rviz::SelectionHandler
{
public:
  RobotLinkSelectionHandler(RobotLink* link, rviz::DisplayContext* context);
  virtual ~RobotLinkSelectionHandler();

  virtual void createProperties(const rviz::Picked& /*obj*/, rviz::Property* parent_property);
  virtual void updateProperties();

  virtual void preRenderPass(uint32_t /*pass*/);
  virtual void postRenderPass(uint32_t /*pass*/);

private:
  RobotLink* link_;
  rviz::VectorProperty* position_property_;
  rviz::QuaternionProperty* orientation_property_;
};

RobotLinkSelectionHandler::RobotLinkSelectionHandler(RobotLink* link, rviz::DisplayContext* context)
  : rviz::SelectionHandler(context), link_(link)
{
}

RobotLinkSelectionHandler::~RobotLinkSelectionHandler() {}
void RobotLinkSelectionHandler::createProperties(const rviz::Picked& /*obj*/, rviz::Property* parent_property)
{
  rviz::Property* group =
      new rviz::Property("Link " + QString::fromStdString(link_->getName()), QVariant(), "", parent_property);
  properties_.push_back(group);

  position_property_ = new rviz::VectorProperty("Position", Ogre::Vector3::ZERO, "", group);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz::QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY, "", group);
  orientation_property_->setReadOnly(true);

  group->expand();
}

void RobotLinkSelectionHandler::updateProperties()
{
  position_property_->setVector(link_->getPosition());
  orientation_property_->setQuaternion(link_->getOrientation());
}

void RobotLinkSelectionHandler::preRenderPass(uint32_t /*pass*/)
{
  if (!link_->is_selectable_)
  {
    if (link_->visual_node_)
    {
      link_->visual_node_->setVisible(false);
    }
    if (link_->collision_node_)
    {
      link_->collision_node_->setVisible(false);
    }
    if (link_->trail_)
    {
      link_->trail_->setVisible(false);
    }
    if (link_->axes_)
    {
      link_->axes_->getSceneNode()->setVisible(false);
    }
  }
}

void RobotLinkSelectionHandler::postRenderPass(uint32_t /*pass*/)
{
  if (!link_->is_selectable_)
  {
    link_->updateVisibility();
  }
}

RobotLink::RobotLink(Robot* robot,
                     const urdf::LinkConstSharedPtr& link,
                     const std::string& parent_joint_name,
                     bool visual,
                     bool collision)
  : robot_(robot)
  , scene_manager_(robot->getDisplayContext()->getSceneManager())
  , context_(robot->getDisplayContext())
  , name_(link->name)
  , parent_joint_name_(parent_joint_name)
  , visual_node_(NULL)
  , collision_node_(NULL)
  , trail_(NULL)
  , axes_(NULL)
  , material_alpha_(1.0)
  , robot_alpha_(1.0)
  , only_render_depth_(false)
  , is_selectable_(true)
  , using_color_(false)
{
  link_property_ = new rviz::Property(link->name.c_str(), true, "", NULL, SLOT(updateVisibility()), this);
  link_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotLink.png"));

  details_ = new rviz::Property("Details", QVariant(), "", NULL);

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1, "Amount of transparency to apply to this link.", link_property_, SLOT(updateAlpha()), this);

  trail_property_ = new rviz::Property("Show Trail",
                                       false,
                                       "Enable/disable a 2 meter \"ribbon\" which follows this link.",
                                       link_property_,
                                       SLOT(updateTrail()),
                                       this);

  axes_property_ = new rviz::Property(
      "Show Axes", false, "Enable/disable showing the axes of this link.", link_property_, SLOT(updateAxes()), this);

  position_property_ = new rviz::VectorProperty("Position",
                                                Ogre::Vector3::ZERO,
                                                "Position of this link, in the current Fixed Frame.  (Not editable)",
                                                link_property_);
  position_property_->setReadOnly(true);

  orientation_property_ =
      new rviz::QuaternionProperty("Orientation",
                                   Ogre::Quaternion::IDENTITY,
                                   "Orientation of this link, in the current Fixed Frame.  (Not editable)",
                                   link_property_);
  orientation_property_->setReadOnly(true);

  link_property_->collapse();

  visual_node_ = robot_->getVisualNode()->createChildSceneNode();
  collision_node_ = robot_->getCollisionNode()->createChildSceneNode();

  // create material for coloring links
  std::stringstream ss;
  static int count = 1;
  ss << "robot link color material " << count++;
  color_material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
  color_material_->setReceiveShadows(false);
  color_material_->getTechnique(0)->setLightingEnabled(true);

  // create the ogre objects to display

  if (visual)
  {
    createVisual(link);
  }

  if (collision)
  {
    createCollision(link);
  }

  if (collision || visual)
  {
    createSelection();
  }

  // create description and fill in child_joint_names_ vector
  std::stringstream desc;
  if (parent_joint_name_.empty())
  {
    desc << "Root Link <b>" << name_ << "</b>";
  }
  else
  {
    desc << "Link <b>" << name_ << "</b>";
    desc << " with parent joint <b>" << parent_joint_name_ << "</b>";
  }

  if (link->child_joints.empty())
  {
    desc << " has no children.";
  }
  else
  {
    desc << " has " << link->child_joints.size();

    if (link->child_joints.size() > 1)
    {
      desc << " child joints: ";
    }
    else
    {
      desc << " child joint: ";
    }

    std::vector<urdf::JointSharedPtr>::const_iterator child_it = link->child_joints.begin();
    std::vector<urdf::JointSharedPtr>::const_iterator child_end = link->child_joints.end();
    for (; child_it != child_end; ++child_it)
    {
      urdf::Joint* child_joint = child_it->get();
      if (child_joint && !child_joint->name.empty())
      {
        child_joint_names_.push_back(child_joint->name);
        desc << "<b>" << child_joint->name << "</b>" << ((child_it + 1 == child_end) ? "." : ", ");
      }
    }
  }
  if (hasGeometry())
  {
    desc << "  Check/uncheck to show/hide this link in the display.";
    if (visual_meshes_.empty())
    {
      desc << "  This link has collision geometry but no visible geometry.";
    }
    else if (collision_meshes_.empty())
    {
      desc << "  This link has visible geometry but no collision geometry.";
    }
  }
  else
  {
    desc << "  This link has NO geometry.";
  }

  link_property_->setDescription(desc.str().c_str());

  if (!hasGeometry())
  {
    link_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotLinkNoGeom.png"));
    alpha_property_->hide();
    link_property_->setValue(QVariant());
  }
}

RobotLink::RobotLink(Robot* robot,
                     const tesseract::AttachableObject& ao,
                     const std::string& parent_joint_name,
                     bool visual,
                     bool collision)
  : robot_(robot)
  , scene_manager_(robot->getDisplayContext()->getSceneManager())
  , context_(robot->getDisplayContext())
  , name_(ao.name)
  , parent_joint_name_(parent_joint_name)
  , visual_node_(NULL)
  , collision_node_(NULL)
  , trail_(NULL)
  , axes_(NULL)
  , material_alpha_(1.0)
  , robot_alpha_(1.0)
  , only_render_depth_(false)
  , is_selectable_(true)
  , using_color_(false)
{
  link_property_ = new rviz::Property(ao.name.c_str(), true, "", NULL, SLOT(updateVisibility()), this);
  link_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotLink.png"));

  details_ = new rviz::Property("Details", QVariant(), "", NULL);

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1, "Amount of transparency to apply to this link.", link_property_, SLOT(updateAlpha()), this);

  trail_property_ = new rviz::Property("Show Trail",
                                       false,
                                       "Enable/disable a 2 meter \"ribbon\" which follows this link.",
                                       link_property_,
                                       SLOT(updateTrail()),
                                       this);

  axes_property_ = new rviz::Property(
      "Show Axes", false, "Enable/disable showing the axes of this link.", link_property_, SLOT(updateAxes()), this);

  position_property_ = new rviz::VectorProperty("Position",
                                                Ogre::Vector3::ZERO,
                                                "Position of this link, in the current Fixed Frame.  (Not editable)",
                                                link_property_);
  position_property_->setReadOnly(true);

  orientation_property_ =
      new rviz::QuaternionProperty("Orientation",
                                   Ogre::Quaternion::IDENTITY,
                                   "Orientation of this link, in the current Fixed Frame.  (Not editable)",
                                   link_property_);
  orientation_property_->setReadOnly(true);

  link_property_->collapse();

  visual_node_ = robot_->getVisualNode()->createChildSceneNode();
  collision_node_ = robot_->getCollisionNode()->createChildSceneNode();

  // create material for coloring links
  std::stringstream ss;
  static int count = 1;
  ss << "attached link color material " << count++;
  color_material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
  color_material_->setReceiveShadows(false);
  color_material_->getTechnique(0)->setLightingEnabled(true);

  // create the ogre objects to display

  if (visual)
  {
    createVisual(ao);
  }

  if (collision)
  {
    createCollision(ao);
  }

  if (collision || visual)
  {
    createSelection();
  }

  // create description and fill in child_joint_names_ vector
  std::stringstream desc;
  if (parent_joint_name_.empty())
  {
    desc << "Root Link <b>" << name_ << "</b>";
  }
  else
  {
    desc << "Link <b>" << name_ << "</b>";
    desc << " with parent joint <b>" << parent_joint_name_ << "</b>";
  }

  desc << " has no children.";

  if (hasGeometry())
  {
    desc << "  Check/uncheck to show/hide this link in the display.";
    if (visual_meshes_.empty())
    {
      desc << "  This link has collision geometry but no visible geometry.";
    }
    else if (collision_meshes_.empty())
    {
      desc << "  This link has visible geometry but no collision geometry.";
    }
  }
  else
  {
    desc << "  This link has NO geometry.";
  }

  link_property_->setDescription(desc.str().c_str());

  if (!hasGeometry())
  {
    link_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotLinkNoGeom.png"));
    alpha_property_->hide();
    link_property_->setValue(QVariant());
  }
}

RobotLink::~RobotLink()
{
  for (size_t i = 0; i < visual_meshes_.size(); i++)
  {
    scene_manager_->destroyEntity(visual_meshes_[i]);
  }

  for (size_t i = 0; i < collision_meshes_.size(); i++)
  {
    scene_manager_->destroyEntity(collision_meshes_[i]);
  }

  for (size_t i = 0; i < visual_octrees_.size(); i++)
  {
    //    scene_manager_->destroyMovableObject( octree_objects_[ i ]); TODO:
    //    Need to create a MovableObjectFactory for this type.
    delete visual_octrees_[i];
  }
  visual_octrees_.clear();

  for (size_t i = 0; i < collision_octrees_.size(); i++)
  {
    //    scene_manager_->destroyMovableObject( octree_objects_[ i ]); TODO:
    //    Need to create a MovableObjectFactory for this type.
    delete collision_octrees_[i];
  }
  collision_octrees_.clear();

  scene_manager_->destroySceneNode(visual_node_);
  scene_manager_->destroySceneNode(collision_node_);

  if (trail_)
  {
    scene_manager_->destroyRibbonTrail(trail_);
  }

  delete axes_;
  delete details_;
  delete link_property_;
}

bool RobotLink::hasGeometry() const
{
  return visual_meshes_.size() + collision_meshes_.size() + visual_octrees_.size() + collision_octrees_.size() > 0;
}

bool RobotLink::getEnabled() const
{
  if (!hasGeometry())
    return true;
  return link_property_->getValue().toBool();
}

void RobotLink::setRobotAlpha(float a)
{
  robot_alpha_ = a;
  updateAlpha();
}

void RobotLink::setRenderQueueGroup(Ogre::uint8 group)
{
  Ogre::SceneNode::ChildNodeIterator child_it = visual_node_->getChildIterator();
  while (child_it.hasMoreElements())
  {
    Ogre::SceneNode* child = dynamic_cast<Ogre::SceneNode*>(child_it.getNext());
    if (child)
    {
      Ogre::SceneNode::ObjectIterator object_it = child->getAttachedObjectIterator();
      while (object_it.hasMoreElements())
      {
        Ogre::MovableObject* obj = object_it.getNext();
        obj->setRenderQueueGroup(group);
      }
    }
  }
}

void RobotLink::setOnlyRenderDepth(bool onlyRenderDepth)
{
  setRenderQueueGroup(onlyRenderDepth ? Ogre::RENDER_QUEUE_BACKGROUND : Ogre::RENDER_QUEUE_MAIN);
  only_render_depth_ = onlyRenderDepth;
  updateAlpha();
}

void RobotLink::updateAlpha()
{
  float link_alpha = alpha_property_->getFloat();
  M_SubEntityToMaterial::iterator it = materials_.begin();
  M_SubEntityToMaterial::iterator end = materials_.end();
  for (; it != end; ++it)
  {
    const Ogre::MaterialPtr& material = it->second;

    if (only_render_depth_)
    {
      material->setColourWriteEnabled(false);
      material->setDepthWriteEnabled(true);
    }
    else
    {
      Ogre::ColourValue color = material->getTechnique(0)->getPass(0)->getDiffuse();
      color.a = robot_alpha_ * material_alpha_ * link_alpha;
      material->setDiffuse(color);

      if (color.a < 0.9998)
      {
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->setDepthWriteEnabled(false);
      }
      else
      {
        material->setSceneBlending(Ogre::SBT_REPLACE);
        material->setDepthWriteEnabled(true);
      }
    }
  }

  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = robot_alpha_ * link_alpha;
  color_material_->setDiffuse(color);

  if (color.a < 0.9998)
  {
    color_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    color_material_->setDepthWriteEnabled(false);
  }
  else
  {
    color_material_->setSceneBlending(Ogre::SBT_REPLACE);
    color_material_->setDepthWriteEnabled(true);
  }

  for (const auto& octree : visual_octrees_)
  {
    octree->setAlpha(robot_alpha_ * link_alpha);
  }

  for (const auto& octree : collision_octrees_)
  {
    octree->setAlpha(robot_alpha_ * link_alpha);
  }
}

void RobotLink::updateVisibility()
{
  bool enabled = getEnabled();

  robot_->calculateJointCheckboxes();

  if (visual_node_)
  {
    visual_node_->setVisible(enabled && robot_->isVisible() && robot_->isVisualVisible());
  }
  if (collision_node_)
  {
    collision_node_->setVisible(enabled && robot_->isVisible() && robot_->isCollisionVisible());
  }
  if (trail_)
  {
    trail_->setVisible(enabled && robot_->isVisible());
  }
  if (axes_)
  {
    axes_->getSceneNode()->setVisible(enabled && robot_->isVisible());
  }
}

Ogre::MaterialPtr RobotLink::getMaterialForLink(const urdf::LinkConstSharedPtr& link, const std::string material_name)
{
  if (!link->visual || !link->visual->material)
  {
    return Ogre::MaterialManager::getSingleton().getByName("RVIZ/ShadedRed");
  }

  static int count = 0;
  std::stringstream ss;
  ss << "Robot Link Material" << count++;

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
  mat->getTechnique(0)->setLightingEnabled(true);

  urdf::VisualSharedPtr visual = link->visual;
  std::vector<urdf::VisualSharedPtr>::const_iterator vi;
  for (vi = link->visual_array.begin(); vi != link->visual_array.end(); vi++)
  {
    if ((*vi) && material_name != "" && (*vi)->material_name == material_name)
    {
      visual = *vi;
      break;
    }
  }
  if (vi == link->visual_array.end())
  {
    visual = link->visual;  // if link does not have material, use default oneee
  }

  if (visual->material->texture_filename.empty())
  {
    const urdf::Color& col = visual->material->color;
    mat->getTechnique(0)->setAmbient(col.r * 0.5, col.g * 0.5, col.b * 0.5);
    mat->getTechnique(0)->setDiffuse(col.r, col.g, col.b, col.a);

    material_alpha_ = col.a;
  }
  else
  {
    std::string filename = visual->material->texture_filename;
    if (!Ogre::TextureManager::getSingleton().resourceExists(filename))
    {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(filename);
      }
      catch (resource_retriever::Exception& e)
      {
        ROS_ERROR("%s", e.what());
      }

      if (res.size != 0)
      {
        Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
        Ogre::Image image;
        std::string extension = fs::extension(fs::path(filename));

        if (extension[0] == '.')
        {
          extension = extension.substr(1, extension.size() - 1);
        }

        try
        {
          image.load(stream, extension);
          Ogre::TextureManager::getSingleton().loadImage(
              filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load texture [%s]: %s", filename.c_str(), e.what());
        }
      }
    }

    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();
    ;
    tex_unit->setTextureName(filename);
  }

  return mat;
}

Ogre::MaterialPtr RobotLink::getMaterialForAttachedLink(const Eigen::Vector4d color)
{
  static int count = 0;
  std::stringstream ss;
  ss << "Robot Attached Link Material" << count++;

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
  mat->getTechnique(0)->setLightingEnabled(true);

  bool has_texture_file = false;
  if (!has_texture_file)
  {
    mat->getTechnique(0)->setAmbient(color(0) * 0.5, color(1) * 0.5, color(2) * 0.5);
    mat->getTechnique(0)->setDiffuse(color(0), color(1), color(2), color(3));

    material_alpha_ = color(3);
  }
  else
  {
    //    std::string filename = visual->material->texture_filename;
    //    if (!Ogre::TextureManager::getSingleton().resourceExists(filename))
    //    {
    //      resource_retriever::Retriever retriever;
    //      resource_retriever::MemoryResource res;
    //      try
    //      {
    //        res = retriever.get(filename);
    //      }
    //      catch (resource_retriever::Exception& e)
    //      {
    //        ROS_ERROR("%s", e.what());
    //      }

    //      if (res.size != 0)
    //      {
    //        Ogre::DataStreamPtr stream(new
    //        Ogre::MemoryDataStream(res.data.get(), res.size));
    //        Ogre::Image image;
    //        std::string extension = fs::extension(fs::path(filename));

    //        if (extension[0] == '.')
    //        {
    //          extension = extension.substr(1, extension.size() - 1);
    //        }

    //        try
    //        {
    //          image.load(stream, extension);
    //          Ogre::TextureManager::getSingleton().loadImage(filename,
    //          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
    //        }
    //        catch (Ogre::Exception& e)
    //        {
    //          ROS_ERROR("Could not load texture [%s]: %s", filename.c_str(),
    //          e.what());
    //        }
    //      }
    //    }

    //    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    //    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();;
    //    tex_unit->setTextureName(filename);
  }

  return mat;
}

bool RobotLink::createEntityForGeometryElement(const urdf::LinkConstSharedPtr& link,
                                               const urdf::Geometry& geom,
                                               const urdf::Pose& origin,
                                               const std::string material_name,
                                               bool isVisual)
{
  Ogre::Entity* entity = nullptr;  // default in case nothing works.

  static int count = 0;
  std::stringstream ss;
  ss << "Robot Link" << count++;
  std::string entity_name = ss.str();

  Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);

  Ogre::Vector3 offset_position(Ogre::Vector3::ZERO);
  Ogre::Quaternion offset_orientation(Ogre::Quaternion::IDENTITY);

  {
    Ogre::Vector3 position(origin.position.x, origin.position.y, origin.position.z);
    Ogre::Quaternion orientation(Ogre::Quaternion::IDENTITY);
    orientation =
        orientation * Ogre::Quaternion(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);

    offset_position = position;
    offset_orientation = orientation;
  }

  switch (geom.type)
  {
    case urdf::Geometry::SPHERE:
    {
      const urdf::Sphere& sphere = static_cast<const urdf::Sphere&>(geom);
      entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Sphere, scene_manager_);

      scale = Ogre::Vector3(sphere.radius * 2, sphere.radius * 2, sphere.radius * 2);
      break;
    }
    case urdf::Geometry::BOX:
    {
      const urdf::Box& box = static_cast<const urdf::Box&>(geom);
      entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Cube, scene_manager_);

      scale = Ogre::Vector3(box.dim.x, box.dim.y, box.dim.z);

      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      const urdf::Cylinder& cylinder = static_cast<const urdf::Cylinder&>(geom);

      Ogre::Quaternion rotX;
      rotX.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
      offset_orientation = offset_orientation * rotX;

      entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Cylinder, scene_manager_);
      scale = Ogre::Vector3(cylinder.radius * 2, cylinder.length, cylinder.radius * 2);
      break;
    }
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geom);

      if (mesh.filename.empty())
        return false;

      scale = Ogre::Vector3(mesh.scale.x, mesh.scale.y, mesh.scale.z);

      std::string model_name = mesh.filename;

      try
      {
        rviz::loadMeshFromResource(model_name);
        entity = scene_manager_->createEntity(ss.str(), model_name);
      }
      catch (Ogre::InvalidParametersException& e)
      {
        ROS_ERROR("Could not convert mesh resource '%s' for link '%s'. It might "
                  "be an empty mesh: %s",
                  model_name.c_str(),
                  link->name.c_str(),
                  e.what());
      }
      catch (Ogre::Exception& e)
      {
        ROS_ERROR("Could not load model '%s' for link '%s': %s", model_name.c_str(), link->name.c_str(), e.what());
      }
      break;
    }
    default:
      ROS_WARN("Unsupported geometry type for element: %d", geom.type);
      break;
  }

  if (entity)
  {
    Ogre::SceneNode* offset_node;
    std::vector<Ogre::Entity*>* meshes;
    if (isVisual)
    {
      offset_node = visual_node_->createChildSceneNode();
      meshes = &visual_meshes_;
    }
    else
    {
      offset_node = collision_node_->createChildSceneNode();
      meshes = &visual_meshes_;
    }

    offset_node->attachObject(entity);
    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    static int count = 0;
    if (default_material_name_.empty())
    {
      default_material_ = getMaterialForLink(link);

      std::stringstream ss;
      ss << default_material_->getName() << count++ << "Robot";
      std::string cloned_name = ss.str();

      default_material_ = default_material_->clone(cloned_name);
      default_material_name_ = default_material_->getName();
    }

    for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
    {
      default_material_ = getMaterialForLink(link, material_name);
      std::stringstream ss;
      ss << default_material_->getName() << count++ << "Robot";
      std::string cloned_name = ss.str();

      default_material_ = default_material_->clone(cloned_name);
      default_material_name_ = default_material_->getName();

      // Assign materials only if the submesh does not have one already

      Ogre::SubEntity* sub = entity->getSubEntity(i);
      const std::string& material_name = sub->getMaterialName();

      if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
      {
        sub->setMaterialName(default_material_name_);
      }
      else
      {
        // Need to clone here due to how selection works.  Once selection id is
        // done per object and not per material,
        // this can go away
        std::stringstream ss;
        ss << material_name << count++ << "Robot";
        std::string cloned_name = ss.str();
        sub->getMaterial()->clone(cloned_name);
        sub->setMaterialName(cloned_name);
      }

      materials_[sub] = sub->getMaterial();
    }

    meshes->push_back(entity);
    return true;
  }

  return false;
}

bool RobotLink::createEntityForGeometryElement(const shapes::Shape& geom,
                                               Eigen::Affine3d origin,
                                               Eigen::Vector4d color,
                                               bool isVisual)
{
  Ogre::Entity* entity = nullptr;  // default in case nothing works.

  static int count = 0;
  std::stringstream ss;
  ss << "Attached Link" << count++;
  std::string entity_name = ss.str();

  Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);

  Ogre::Vector3 offset_position(Ogre::Vector3::ZERO);
  Ogre::Quaternion offset_orientation(Ogre::Quaternion::IDENTITY);

  {
    const Eigen::Vector3d& origin_position = origin.translation();
    Eigen::Quaterniond origin_orientation(origin.rotation());
    Ogre::Vector3 position = Ogre::Vector3(origin_position.x(), origin_position.y(), origin_position.z());
    Ogre::Quaternion orientation = Ogre::Quaternion(
        origin_orientation.w(), origin_orientation.x(), origin_orientation.y(), origin_orientation.z());

    offset_position = position;
    offset_orientation = Ogre::Quaternion::IDENTITY * orientation;
  }
  if (geom.type != shapes::OCTREE)
  {
    switch (geom.type)
    {
      case shapes::SPHERE:
      {
        entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Sphere, scene_manager_);
        double d = 2.0 * static_cast<const shapes::Sphere&>(geom).radius;
        scale = Ogre::Vector3(d, d, d);
        break;
      }
      case shapes::BOX:
      {
        entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Cube, scene_manager_);
        const double* sz = static_cast<const shapes::Box&>(geom).size;
        scale = Ogre::Vector3(sz[0], sz[1], sz[2]);
        break;
      }
      case shapes::CYLINDER:
      {
        Ogre::Quaternion rotX;
        rotX.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
        offset_orientation = offset_orientation * rotX;

        entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Cylinder, scene_manager_);
        double d = 2.0 * static_cast<const shapes::Cylinder&>(geom).radius;
        double z = static_cast<const shapes::Cylinder&>(geom).length;
        scale = Ogre::Vector3(d, z, d);
        break;
      }
      case shapes::CONE:
      {
        entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Cone, scene_manager_);
        double d = 2.0 * static_cast<const shapes::Cone&>(geom).radius;
        double z = static_cast<const shapes::Cylinder&>(geom).length;
        scale = Ogre::Vector3(d, d, z);
        break;
      }
      case shapes::MESH:
      {
        bool use_resource = false;
        if (use_resource)
        {
          //      const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geom);

          //      if ( mesh.filename.empty() )
          //        return;

          //      scale = Ogre::Vector3(mesh.scale.x, mesh.scale.y, mesh.scale.z);

          //      std::string model_name = mesh.filename;

          //      try
          //      {
          //        rviz::loadMeshFromResource(model_name);
          //        entity = scene_manager_->createEntity( ss.str(), model_name );
          //      }
          //      catch( Ogre::InvalidParametersException& e )
          //      {
          //        ROS_ERROR( "Could not convert mesh resource '%s' for link
          //        '%s'. It might be an empty mesh: %s", model_name.c_str(),
          //        link->name.c_str(), e.what() );
          //      }
          //      catch( Ogre::Exception& e )
          //      {
          //        ROS_ERROR( "Could not load model '%s' for link '%s': %s",
          //        model_name.c_str(), link->name.c_str(), e.what() );
          //      }
        }
        else
        {
          const shapes::Mesh& mesh = static_cast<const shapes::Mesh&>(geom);
          if (mesh.triangle_count > 0)
          {
            Ogre::ManualObject* object = new Ogre::ManualObject("the one and only");
            object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

            unsigned int vertexCount = 0;
            Ogre::Vector3 normal(0.0, 0.0, 0.0);
            for (unsigned int i = 0; i < mesh.triangle_count; ++i)
            {
              if (vertexCount >= 2004)
              {
                // Subdivide large meshes into submeshes with at most 2004
                // vertices to prevent problems on some graphics cards.
                object->end();
                object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
                vertexCount = 0;
              }

              unsigned int i3 = i * 3;
              if (mesh.triangle_normals && !mesh.vertex_normals)
              {
                normal.x = mesh.triangle_normals[i3];
                normal.y = mesh.triangle_normals[i3 + 1];
                normal.z = mesh.triangle_normals[i3 + 2];
              }

              std::vector<Ogre::Vector3> verticies(3);
              std::vector<Ogre::Vector3> normals(3);
              for (int k = 0; k < 3; ++k)
              {
                unsigned int vi = 3 * mesh.triangles[i3 + k];
                verticies[k] = Ogre::Vector3(mesh.vertices[vi], mesh.vertices[vi + 1], mesh.vertices[vi + 2]);
                if (mesh.vertex_normals)
                {
                  normals[k] =
                      Ogre::Vector3(mesh.vertex_normals[vi], mesh.vertex_normals[vi + 1], mesh.vertex_normals[vi + 2]);
                }
                else if (mesh.triangle_normals)
                {
                  normals[k] = normal;
                }
              }

              if (!mesh.triangle_normals && !mesh.vertex_normals)
              {
                Ogre::Vector3 side1 = verticies[0] - verticies[1];
                Ogre::Vector3 side2 = verticies[1] - verticies[2];
                normal = side1.crossProduct(side2);
                normal.normalise();

                normals[0] = normal;
                normals[1] = normal;
                normals[2] = normal;
              }

              //            float u, v;
              //            u = v = 0.0f;
              object->position(verticies[0]);
              object->normal(normals[0]);
              //          calculateUV( verticies[0], u, v );
              //          object->textureCoord( u, v );

              object->position(verticies[1]);
              object->normal(normals[1]);
              //          calculateUV( verticies[1], u, v );
              //          object->textureCoord( u, v );

              object->position(verticies[2]);
              object->normal(normals[2]);
              //          calculateUV( verticies[2], u, v );
              //          object->textureCoord( u, v );

              object->triangle(vertexCount + 0, vertexCount + 1, vertexCount + 2);

              vertexCount += 3;
            }

            object->end();

            std::string mesh_name = entity_name + "mesh";
            Ogre::MeshPtr ogre_mesh =
                object->convertToMesh(mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            ogre_mesh->buildEdgeList();

            entity = scene_manager_->createEntity(entity_name, mesh_name);

            delete object;
          }
        }
        break;
      }
      default:
        ROS_WARN("Unsupported geometry type for element: %d", geom.type);
        break;
    }

    if (entity)
    {
      Ogre::SceneNode* offset_node;
      std::vector<Ogre::Entity*>* meshes;
      if (isVisual)
      {
        offset_node = visual_node_->createChildSceneNode();
        meshes = &visual_meshes_;
      }
      else
      {
        offset_node = collision_node_->createChildSceneNode();
        meshes = &visual_meshes_;
      }

      offset_node->attachObject(entity);
      offset_node->setScale(scale);
      offset_node->setPosition(offset_position);
      offset_node->setOrientation(offset_orientation);

      default_material_ = getMaterialForAttachedLink(color);
      for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
      {
        std::stringstream ss;
        ss << default_material_->getName() << count++ << "Robot";
        std::string cloned_name = ss.str();

        default_material_ = default_material_->clone(cloned_name);
        default_material_name_ = default_material_->getName();

        // Assign materials only if the submesh does not have one already

        Ogre::SubEntity* sub = entity->getSubEntity(i);
        const std::string& material_name = sub->getMaterialName();

        if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
        {
          sub->setMaterialName(default_material_name_);
        }
        else
        {
          // Need to clone here due to how selection works.  Once selection id
          // is done per object and not per material,
          // this can go away
          std::stringstream ss;
          ss << material_name << count++ << "Robot";
          std::string cloned_name = ss.str();
          sub->getMaterial()->clone(cloned_name);
          sub->setMaterialName(cloned_name);
        }

        materials_[sub] = sub->getMaterial();
      }

      meshes->push_back(entity);
      return true;
    }
  }
  else
  {
    std::size_t max_octree_depth = 0;
    double color_factor = 0.8;
    OctreeVoxelRenderMode octree_voxel_rendering = OCTOMAP_OCCUPIED_VOXELS;
    OctreeVoxelColorMode octree_color_mode = OCTOMAP_Z_AXIS_COLOR;
    std::size_t octree_depth;
    Ogre::SceneNode* offset_node;
    std::vector<rviz::PointCloud*>* octree_objects;

    const std::shared_ptr<const octomap::OcTree>& octree = static_cast<const shapes::OcTree&>(geom).octree;

    if (!max_octree_depth)
      octree_depth = octree->getTreeDepth();
    else
      octree_depth = std::min(max_octree_depth, (std::size_t)octree->getTreeDepth());

    if (isVisual)
    {
      offset_node = visual_node_->createChildSceneNode();
      octree_objects = &visual_octrees_;
    }
    else
    {
      offset_node = collision_node_->createChildSceneNode();
      octree_objects = &collision_octrees_;
    }

    std::vector<std::vector<rviz::PointCloud::Point>> pointBuf;
    pointBuf.resize(octree_depth);

    // get dimensions of octree
    double minX, minY, minZ, maxX, maxY, maxZ;
    octree->getMetricMin(minX, minY, minZ);
    octree->getMetricMax(maxX, maxY, maxZ);

    unsigned int render_mode_mask = static_cast<unsigned int>(octree_voxel_rendering);

    size_t pointCount = 0;
    {
      // traverse all leafs in the tree:
      for (octomap::OcTree::iterator it = octree->begin(octree_depth), end = octree->end(); it != end; ++it)
      {
        bool display_voxel = false;

        // the left part evaluates to 1 for free voxels and 2 for occupied
        // voxels
        if (((int)octree->isNodeOccupied(*it) + 1) & render_mode_mask)
        {
          // check if current voxel has neighbors on all sides -> no need to be
          // displayed
          bool allNeighborsFound = true;

          octomap::OcTreeKey key;
          octomap::OcTreeKey nKey = it.getKey();

          for (key[2] = nKey[2] - 1; allNeighborsFound && key[2] <= nKey[2] + 1; ++key[2])
          {
            for (key[1] = nKey[1] - 1; allNeighborsFound && key[1] <= nKey[1] + 1; ++key[1])
            {
              for (key[0] = nKey[0] - 1; allNeighborsFound && key[0] <= nKey[0] + 1; ++key[0])
              {
                if (key != nKey)
                {
                  octomap::OcTreeNode* node = octree->search(key);

                  // the left part evaluates to 1 for free voxels and 2 for
                  // occupied voxels
                  if (!(node && (((int)octree->isNodeOccupied(node)) + 1) & render_mode_mask))
                  {
                    // we do not have a neighbor => break!
                    allNeighborsFound = false;
                  }
                }
              }
            }
          }

          display_voxel |= !allNeighborsFound;
        }

        if (display_voxel)
        {
          rviz::PointCloud::Point newPoint;

          newPoint.position.x = it.getX();
          newPoint.position.y = it.getY();
          newPoint.position.z = it.getZ();

          float cell_probability;

          switch (octree_color_mode)
          {
            case OCTOMAP_Z_AXIS_COLOR:
              setOctomapColor(newPoint.position.z, minZ, maxZ, color_factor, &newPoint);
              break;
            case OCTOMAP_PROBABLILTY_COLOR:
              cell_probability = it->getOccupancy();
              newPoint.setColor((1.0f - cell_probability), cell_probability, 0.0);
              break;
            default:
              break;
          }

          // push to point vectors
          unsigned int depth = it.getDepth();
          pointBuf[depth - 1].push_back(newPoint);

          ++pointCount;
        }
      }
    }

    for (size_t i = 0; i < octree_depth; ++i)
    {
      double size = octree->getNodeSize(i + 1);

      std::stringstream sname;
      static int count = 0;
      sname << "PointCloud Nr." << count++;
      rviz::PointCloud* cloud = new rviz::PointCloud();
      cloud->setName(sname.str());
      cloud->setRenderMode(rviz::PointCloud::RM_BOXES);
      cloud->clear();
      cloud->setDimensions(size, size, size);

      cloud->addPoints(&pointBuf[i].front(), pointBuf[i].size());
      pointBuf[i].clear();

      offset_node->attachObject(cloud);
      octree_objects->push_back(cloud);
    }

    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    return true;
  }

  return false;
}

void RobotLink::setOctomapColor(double z_pos,
                                double min_z,
                                double max_z,
                                double color_factor,
                                rviz::PointCloud::Point* point)
{
  int i;
  double m, n, f;

  double s = 1.0;
  double v = 1.0;

  double h = (1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor;

  h -= floor(h);
  h *= 6;
  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
    case 6:
    case 0:
      point->setColor(v, n, m);
      break;
    case 1:
      point->setColor(n, v, m);
      break;
    case 2:
      point->setColor(m, v, n);
      break;
    case 3:
      point->setColor(m, n, v);
      break;
    case 4:
      point->setColor(n, m, v);
      break;
    case 5:
      point->setColor(v, m, n);
      break;
    default:
      point->setColor(1, 0.5, 0.5);
      break;
  }
}

void RobotLink::createCollision(const urdf::LinkConstSharedPtr& link)
{
  bool valid_collision_found = false;
  std::vector<urdf::CollisionSharedPtr>::const_iterator vi;
  for (vi = link->collision_array.begin(); vi != link->collision_array.end(); vi++)
  {
    urdf::CollisionSharedPtr collision = *vi;
    if (collision && collision->geometry)
    {
      if (createEntityForGeometryElement(link, *collision->geometry, collision->origin, "", false))
        valid_collision_found = true;
    }
  }

  if (!valid_collision_found && link->collision && link->collision->geometry)
  {
    createEntityForGeometryElement(link, *link->collision->geometry, link->collision->origin, "", false);
  }

  collision_node_->setVisible(getEnabled());
}

void RobotLink::createVisual(const urdf::LinkConstSharedPtr& link)
{
  bool valid_visual_found = false;

  std::vector<urdf::VisualSharedPtr>::const_iterator vi;
  for (vi = link->visual_array.begin(); vi != link->visual_array.end(); vi++)
  {
    urdf::VisualSharedPtr visual = *vi;
    if (visual && visual->geometry)
    {
      if (createEntityForGeometryElement(link, *visual->geometry, visual->origin, visual->material_name, true))
        valid_visual_found = true;
    }
  }

  if (!valid_visual_found && link->visual && link->visual->geometry)
  {
    createEntityForGeometryElement(
        link, *link->visual->geometry, link->visual->origin, link->visual->material_name, true);
  }

  visual_node_->setVisible(getEnabled());
}

void RobotLink::createVisual(const tesseract::AttachableObject& ao)
{
  for (unsigned i = 0; i < ao.visual.shapes.size(); ++i)
  {
    const shapes::ShapeConstPtr& visual = ao.visual.shapes[i];
    if (visual)
    {
      if (ao.visual.shape_colors.empty())
        createEntityForGeometryElement(
            *visual, ao.visual.shape_poses[i], Eigen::Vector4d(0.0f, 0.7f, 0.0f, 1.0f), true);
      else
        createEntityForGeometryElement(*visual, ao.visual.shape_poses[i], ao.visual.shape_colors[i], true);
    }
  }

  visual_node_->setVisible(getEnabled());
}

void RobotLink::createCollision(const tesseract::AttachableObject& ao)
{
  for (unsigned i = 0; i < ao.collision.shapes.size(); ++i)
  {
    const shapes::ShapeConstPtr& collision = ao.collision.shapes[i];
    if (collision)
    {
      if (ao.collision.shape_colors.empty())
        createEntityForGeometryElement(
            *collision, ao.collision.shape_poses[i], Eigen::Vector4d(0.0f, 0.7f, 0.0f, 1.0f), false);
      else
        createEntityForGeometryElement(*collision, ao.collision.shape_poses[i], ao.collision.shape_colors[i], false);
    }
  }

  collision_node_->setVisible(getEnabled());
}

void RobotLink::createSelection()
{
  selection_handler_.reset(new RobotLinkSelectionHandler(this, context_));
  for (size_t i = 0; i < visual_meshes_.size(); i++)
  {
    selection_handler_->addTrackedObject(visual_meshes_[i]);
  }
  for (size_t i = 0; i < collision_meshes_.size(); i++)
  {
    selection_handler_->addTrackedObject(collision_meshes_[i]);
  }
  for (size_t i = 0; i < visual_octrees_.size(); i++)
  {
    selection_handler_->addTrackedObject(visual_octrees_[i]);
  }
  for (size_t i = 0; i < collision_octrees_.size(); i++)
  {
    selection_handler_->addTrackedObject(collision_octrees_[i]);
  }
}

void RobotLink::updateTrail()
{
  if (trail_property_->getValue().toBool())
  {
    if (!trail_)
    {
      if (visual_node_)
      {
        static int count = 0;
        std::stringstream ss;
        ss << "Trail for link " << name_ << count++;
        trail_ = scene_manager_->createRibbonTrail(ss.str());
        trail_->setMaxChainElements(100);
        trail_->setInitialWidth(0, 0.01f);
        trail_->setInitialColour(0, 0.0f, 0.5f, 0.5f);
        trail_->addNode(visual_node_);
        trail_->setTrailLength(2.0f);
        trail_->setVisible(getEnabled());
        robot_->getOtherNode()->attachObject(trail_);
      }
      else
      {
        ROS_WARN("No visual node for link %s, cannot create a trail", name_.c_str());
      }
    }
  }
  else
  {
    if (trail_)
    {
      scene_manager_->destroyRibbonTrail(trail_);
      trail_ = NULL;
    }
  }
}

void RobotLink::updateAxes()
{
  if (axes_property_->getValue().toBool())
  {
    if (!axes_)
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Axes for link " << name_ << count++;
      axes_ = new rviz::Axes(scene_manager_, robot_->getOtherNode(), 0.1, 0.01);
      axes_->getSceneNode()->setVisible(getEnabled());

      axes_->setPosition(position_property_->getVector());
      axes_->setOrientation(orientation_property_->getQuaternion());
    }
  }
  else
  {
    if (axes_)
    {
      delete axes_;
      axes_ = NULL;
    }
  }
}

void RobotLink::setTransforms(const Ogre::Vector3& visual_position,
                              const Ogre::Quaternion& visual_orientation,
                              const Ogre::Vector3& collision_position,
                              const Ogre::Quaternion& collision_orientation)
{
  if (visual_node_)
  {
    visual_node_->setPosition(visual_position);
    visual_node_->setOrientation(visual_orientation);
  }

  if (collision_node_)
  {
    collision_node_->setPosition(collision_position);
    collision_node_->setOrientation(collision_orientation);
  }

  position_property_->setVector(visual_position);
  orientation_property_->setQuaternion(visual_orientation);

  if (axes_)
  {
    axes_->setPosition(visual_position);
    axes_->setOrientation(visual_orientation);
  }
}

void RobotLink::setToErrorMaterial()
{
  for (size_t i = 0; i < visual_meshes_.size(); i++)
  {
    visual_meshes_[i]->setMaterialName("BaseWhiteNoLighting");
  }
  for (size_t i = 0; i < collision_meshes_.size(); i++)
  {
    collision_meshes_[i]->setMaterialName("BaseWhiteNoLighting");
  }

  // Currently not handling color for octree_objects_
}

void RobotLink::setToNormalMaterial()
{
  if (using_color_)
  {
    for (size_t i = 0; i < visual_meshes_.size(); i++)
    {
      visual_meshes_[i]->setMaterial(color_material_);
    }
    for (size_t i = 0; i < collision_meshes_.size(); i++)
    {
      collision_meshes_[i]->setMaterial(color_material_);
    }

    // Currently not handling color for octree_objects_
  }
  else
  {
    M_SubEntityToMaterial::iterator it = materials_.begin();
    M_SubEntityToMaterial::iterator end = materials_.end();
    for (; it != end; ++it)
    {
      it->first->setMaterial(it->second);
    }
  }
}

void RobotLink::setColor(float red, float green, float blue)
{
  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.r = red;
  color.g = green;
  color.b = blue;
  color_material_->getTechnique(0)->setAmbient(0.5 * color);
  color_material_->getTechnique(0)->setDiffuse(color);

  using_color_ = true;
  setToNormalMaterial();
}

void RobotLink::unsetColor()
{
  using_color_ = false;
  setToNormalMaterial();
}

bool RobotLink::setSelectable(bool selectable)
{
  bool old = is_selectable_;
  is_selectable_ = selectable;
  return old;
}

bool RobotLink::getSelectable() { return is_selectable_; }
void RobotLink::hideSubProperties(bool hide)
{
  position_property_->setHidden(hide);
  orientation_property_->setHidden(hide);
  trail_property_->setHidden(hide);
  axes_property_->setHidden(hide);
  alpha_property_->setHidden(hide);
}

Ogre::Vector3 RobotLink::getPosition() { return position_property_->getVector(); }
Ogre::Quaternion RobotLink::getOrientation() { return orientation_property_->getQuaternion(); }
void RobotLink::setParentProperty(rviz::Property* new_parent)
{
  rviz::Property* old_parent = link_property_->getParent();
  if (old_parent)
    old_parent->takeChild(link_property_);

  if (new_parent)
    new_parent->addChild(link_property_);
}

// if use_detail:
//    - all sub properties become children of details_ property.
//    - details_ property becomes a child of link_property_
// else (!use_detail)
//    - all sub properties become children of link_property_.
//    details_ property does not have a parent.
void RobotLink::useDetailProperty(bool use_detail)
{
  rviz::Property* old_parent = details_->getParent();
  if (old_parent)
    old_parent->takeChild(details_);

  if (use_detail)
  {
    while (link_property_->numChildren() > 0)
    {
      rviz::Property* child = link_property_->childAt(0);
      link_property_->takeChild(child);
      details_->addChild(child);
    }

    link_property_->addChild(details_);
  }
  else
  {
    while (details_->numChildren() > 0)
    {
      rviz::Property* child = details_->childAt(0);
      details_->takeChild(child);
      link_property_->addChild(child);
    }
  }
}

void RobotLink::expandDetails(bool expand)
{
  rviz::Property* parent = details_->getParent() ? details_ : link_property_;
  if (expand)
  {
    parent->expand();
  }
  else
  {
    parent->collapse();
  }
}

}  // namespace rviz
