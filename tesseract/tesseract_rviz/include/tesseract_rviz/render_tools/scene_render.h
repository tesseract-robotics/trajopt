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

#ifndef TESSERACT_RVIZ_RENDER
#define TESSERACT_RVIZ_RENDER

#include "tesseract_rviz/render_tools/render_shapes.h"
#include "tesseract_rviz/render_tools/state_visualization.h"
#include <tesseract_ros/ros_basic_env.h>
#include <rviz/helpers/color.h>
#include <OgreMaterial.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class DisplayContext;
}

namespace tesseract_rviz
{

class SceneRender
{
public:
  SceneRender(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const StateVisualizationPtr state);
  ~SceneRender();

  Ogre::SceneNode* getGeometryNode()
  {
    return geometry_node_;
  }

  const StateVisualizationPtr& getRobotVisualization()
  {
    return state_;
  }

  void render(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
              const tesseract::EnvStateConstPtr state,
              const rviz::Color& default_scene_color,
              const rviz::Color& default_attached_color,
              OctreeVoxelRenderMode voxel_render_mode,
              OctreeVoxelColorMode voxel_color_mode, float default_scene_alpha);

  void clear();

private:
  Ogre::SceneNode* geometry_node_;
  rviz::DisplayContext* context_;
  RenderShapesPtr render_shapes_;
  StateVisualizationPtr state_;
};
typedef std::shared_ptr<SceneRender> SceneRenderPtr;
typedef std::shared_ptr<const SceneRender> SceneRenderConstPtr;

}

#endif //TESSERACT_RVIZ_RENDER
