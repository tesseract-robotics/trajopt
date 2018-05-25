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
#include <QApplication>
#include <tesseract_ros/ros_tesseract_utils.h>

namespace tesseract_rviz
{
using namespace tesseract;

StateVisualization::StateVisualization(Ogre::SceneNode* root_node,
                                       rviz::DisplayContext* context,
                                       const std::string& name,
                                       rviz::Property* parent_property)
  : robot_(root_node, context, name, parent_property), visible_(true), visual_visible_(true), collision_visible_(false)
{
  default_attached_object_color_.r = 0.0f;
  default_attached_object_color_.g = 0.7f;
  default_attached_object_color_.b = 0.0f;
  default_attached_object_color_.a = 1.0f;
}

void StateVisualization::load(urdf::ModelInterfaceConstSharedPtr urdf,
                              bool visual,
                              bool collision,
                              bool show_active,
                              bool show_static)
{
  // clear previously loaded model
  clear();

  robot_.load(urdf, visual, collision, show_active, show_static);
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
}

void StateVisualization::clear() { robot_.clear(); }
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
                                const tesseract::ObjectColorMapConstPtr color_map)
{
  updateHelper(env, state, default_attached_object_color, color_map);
}

void StateVisualization::updateHelper(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
                                      const tesseract::EnvStateConstPtr state,
                                      const std_msgs::ColorRGBA& /*default_attached_object_color*/,
                                      const tesseract::ObjectColorMapConstPtr /*color_map*/)
{
  const AttachedBodyInfoMap& attached_bodies = env->getAttachedBodies();
  const auto& attachable_objects = env->getAttachableObjects();

  // Need to remove links that no longer exist
  for (const auto& ab : attached_bodies)
  {
    const auto it = attached_bodies_.find(ab.second.object_name);
    if (it == attached_bodies_.end())  // Add body if it does not already exist
    {
      const auto& ao = attachable_objects.at(ab.second.object_name);
      robot_.attachBody(*ao, ab.second);
    }
    else
    {
      const auto& ao = attachable_objects.at(ab.second.object_name);
      const auto& ao_prev = attachable_objects_.at(ab.second.object_name);
      if (!tesseract_ros::isIdentical(*ao, *ao_prev))
      {
        robot_.detachBody(ab.second.object_name);
        robot_.attachBody(*ao, ab.second);
      }
    }
  }

  attached_bodies_ = attached_bodies;
  attachable_objects_ = attachable_objects;
  robot_.update(LinkUpdater(state));
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
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

void StateVisualization::setAlpha(float alpha) { robot_.setAlpha(alpha); }
}
