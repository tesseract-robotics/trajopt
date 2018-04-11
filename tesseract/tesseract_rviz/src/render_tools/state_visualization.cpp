/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "tesseract_rviz/render_tools/state_visualization.h"
#include "tesseract_rviz/render_tools/link_updater.h"
#include "tesseract_rviz/render_tools/render_shapes.h"
#include <QApplication>

namespace tesseract_rviz
{
using namespace tesseract;

StateVisualization::StateVisualization(Ogre::SceneNode* root_node, rviz::DisplayContext* context,
                                       const std::string& name, rviz::Property* parent_property)
  : robot_(root_node, context, name, parent_property)
  , octree_voxel_render_mode_(OCTOMAP_OCCUPIED_VOXELS)
  , octree_voxel_color_mode_(OCTOMAP_Z_AXIS_COLOR)
  , visible_(true)
  , visual_visible_(true)
  , collision_visible_(false)
  , attached_visual_visible_(true)
  , attached_collision_visible_(false)
{
  default_attached_object_color_.r = 0.0f;
  default_attached_object_color_.g = 0.7f;
  default_attached_object_color_.b = 0.0f;
  default_attached_object_color_.a = 1.0f;
  render_shapes_.reset(new RenderShapes(context));
}

void StateVisualization::load(const urdf::ModelInterface& descr, bool visual, bool collision)
{
  // clear previously loaded model
  clear();

  robot_.load(descr, visual, collision);
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
  QApplication::processEvents();
}

void StateVisualization::clear()
{
  render_shapes_->clear();
  robot_.clear();
}

void StateVisualization::setDefaultAttachedObjectColor(const std_msgs::ColorRGBA& default_attached_object_color)
{
  default_attached_object_color_ = default_attached_object_color;
}

void StateVisualization::update(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
                                const tesseract::EnvStateConstPtr state)
{
  updateHelper(env, state, default_attached_object_color_, NULL);
}

void StateVisualization::update(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
                                const tesseract::EnvStateConstPtr state,
                                const std_msgs::ColorRGBA& default_attached_object_color)
{
  updateHelper(env, state, default_attached_object_color, NULL);
}

void StateVisualization::update(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
                                const tesseract::EnvStateConstPtr state,
                                const std_msgs::ColorRGBA& default_attached_object_color,
                                const tesseract::tesseract_ros::ObjectColorMapConstPtr color_map)
{
  updateHelper(env, state, default_attached_object_color, color_map);
}

void StateVisualization::updateHelper(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
                                      const tesseract::EnvStateConstPtr state,
                                      const std_msgs::ColorRGBA& default_attached_object_color,
                                      const tesseract::tesseract_ros::ObjectColorMapConstPtr color_map)
{
  robot_.update(LinkUpdater(state));
  render_shapes_->clear();

  const tesseract_ros::AttachedBodyConstPtrMap& attached_bodies = env->getAttachedBodies();
  for (const auto &body : attached_bodies)
  {
    std_msgs::ColorRGBA color = default_attached_object_color;
    float alpha = robot_.getAlpha();
    bool use_color_map = false;
    if (color_map)
    {
      std::unordered_map<std::string, std_msgs::ColorRGBA>::const_iterator it = color_map->find(body.second->info.name);
      if (it != color_map->end())
      {  // render attached bodies with a color that is a bit different
        color.r = std::max(1.0f, it->second.r * 1.05f);
        color.g = std::max(1.0f, it->second.g * 1.05f);
        color.b = std::max(1.0f, it->second.b * 1.05f);
        alpha = color.a = it->second.a;
        use_color_map = true;
      }
    }


    const Eigen::Affine3d &link_tf = state->transforms.at(body.second->info.name);
    const EigenSTL::vector_Affine3d& ab_visual_pose = body.second->obj->visual.shape_poses;
    const std::vector<shapes::ShapeConstPtr>& ab_visual_shapes = body.second->obj->visual.shapes;
    const EigenSTL::vector_Vector4d& ab_visual_colors = body.second->obj->visual.shape_colors;
    for (std::size_t j = 0; j < ab_visual_shapes.size(); ++j)
    {
      if (!use_color_map && !ab_visual_colors.empty())
      {
        color.r = ab_visual_colors[j](0);
        color.g = ab_visual_colors[j](1);
        color.b = ab_visual_colors[j](2);
        alpha = color.a = ab_visual_colors[j](3);
      }

      rviz::Color rcolor(color.r, color.g, color.b);
      render_shapes_->renderShape(robot_.getVisualNode(), ab_visual_shapes[j].get(), link_tf * ab_visual_pose[j], octree_voxel_render_mode_,
                                  octree_voxel_color_mode_, rcolor, alpha, false);

    }

    const EigenSTL::vector_Affine3d& ab_collision_pose = body.second->obj->collision.shape_poses;
    const std::vector<shapes::ShapeConstPtr>& ab_collision_shapes = body.second->obj->collision.shapes;
    const EigenSTL::vector_Vector4d& ab_collision_colors = body.second->obj->collision.shape_colors;
    for (std::size_t j = 0; j < ab_collision_shapes.size(); ++j)
    {
      if (!use_color_map && !ab_collision_colors.empty())
      {
        color.r = ab_collision_colors[j](0);
        color.g = ab_collision_colors[j](1);
        color.b = ab_collision_colors[j](2);
        alpha = color.a = ab_collision_colors[j](3);
      }

      rviz::Color rcolor(color.r, color.g, color.b);
      render_shapes_->renderShape(robot_.getCollisionNode(), ab_collision_shapes[j].get(), link_tf * ab_collision_pose[j], octree_voxel_render_mode_,
                                  octree_voxel_color_mode_, rcolor, alpha, true);
    }
  }

  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);

  render_shapes_->setVisualVisible(attached_visual_visible_);
  render_shapes_->setCollisionVisible(attached_collision_visible_);
}

void StateVisualization::setVisible(bool visible)
{
  visible_ = visible;
  robot_.setVisible(visible);
}

void StateVisualization::setVisualVisible(bool visible)
{
  visual_visible_ = visible;
  robot_.setVisualVisible(visible);
}

void StateVisualization::setCollisionVisible(bool visible)
{
  collision_visible_ = visible;
  robot_.setCollisionVisible(visible);
}

void StateVisualization::setAttachedVisualVisible(bool visible)
{
  attached_visual_visible_ = visible;
  render_shapes_->setVisualVisible(visible);
}

void StateVisualization::setAttachedCollisionVisible(bool visible)
{
  attached_collision_visible_ = visible;
  render_shapes_->setCollisionVisible(visible);
}

void StateVisualization::setAlpha(float alpha)
{
  robot_.setAlpha(alpha);
}
}
