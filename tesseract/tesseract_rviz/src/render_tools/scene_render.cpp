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

#include "tesseract_rviz/render_tools/scene_render.h"
#include "tesseract_rviz/render_tools/state_visualization.h"
#include "tesseract_rviz/render_tools/render_shapes.h"
#include <rviz/display_context.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace tesseract_rviz
{
using namespace tesseract;

SceneRender::SceneRender(Ogre::SceneNode* node, rviz::DisplayContext* context, const StateVisualizationPtr state)
  : geometry_node_(node->createChildSceneNode()), context_(context), state_(state)
{
  render_shapes_.reset(new RenderShapes(context));
}

SceneRender::~SceneRender()
{
  context_->getSceneManager()->destroySceneNode(geometry_node_->getName());
}

void SceneRender::clear()
{
  render_shapes_->clear();
}

void SceneRender::render(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
                         const tesseract::tesseract_ros::EnvStateConstPtr state,
                         const rviz::Color& default_env_color,
                         const rviz::Color& default_attached_color,
                         OctreeVoxelRenderMode octree_voxel_rendering,
                         OctreeVoxelColorMode octree_color_mode, float default_scene_alpha)
{
  if (!env || !state)
    return;

  clear();

  if (state_)
  {
    std_msgs::ColorRGBA color;
    color.r = default_attached_color.r_;
    color.g = default_attached_color.g_;
    color.b = default_attached_color.b_;
    color.a = 1.0f;
    tesseract_ros::ObjectColorMapConstPtr color_map = env->getKnownObjectColors();
    state_->update(env, state, color, color_map);
  }
}
}
