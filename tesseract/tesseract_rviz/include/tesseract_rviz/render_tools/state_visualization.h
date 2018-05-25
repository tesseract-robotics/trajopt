/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef TESSERACT_RVIZ_STATE_VISUALIZATION
#define TESSERACT_RVIZ_STATE_VISUALIZATION

#include "tesseract_rviz/render_tools/env/robot.h"
#include <tesseract_ros/ros_basic_env.h>
#include <std_msgs/ColorRGBA.h>
#include <memory>

namespace tesseract_rviz
{

/** \brief Update the links of an rviz::Robot using a robot_state::RobotState */
class StateVisualization
{
public:
  StateVisualization(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property);

  Robot& getRobot() { return robot_; }

  void load(urdf::ModelInterfaceConstSharedPtr urdf, bool visual = true, bool collision = true, bool show_active = true, bool show_static = true);
  void clear();

  void update(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
              const tesseract::EnvStateConstPtr state);

  void update(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
              const tesseract::EnvStateConstPtr state,
              const std_msgs::ColorRGBA& default_attached_object_color);

  void update(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
              const tesseract::EnvStateConstPtr state,
              const std_msgs::ColorRGBA& default_attached_object_color,
              const tesseract::ObjectColorMapConstPtr color_map);

  void setDefaultAttachedObjectColor(const std_msgs::ColorRGBA& default_attached_object_color);

  /**
   * \brief Set the robot as a whole to be visible or not
   * @param visible Should we be visible?
   */
  void setVisible(bool visible);

  /**
   * \brief Set whether the visual meshes of the robot should be visible
   * @param visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible(bool visible);

  /**
   * \brief Set whether the collision meshes/primitives of the robot should be visible
   * @param visible Whether the collision meshes/primitives should be visible
   */
  void setCollisionVisible(bool visible);

  void setAlpha(float alpha);

private:
  void updateHelper(const tesseract::tesseract_ros::ROSBasicEnvConstPtr env,
                    const tesseract::EnvStateConstPtr state,
                    const std_msgs::ColorRGBA& default_attached_object_color,
                    const tesseract::ObjectColorMapConstPtr color_map);

  Robot robot_;
  tesseract::AttachedBodyInfoMap attached_bodies_;
  tesseract::AttachableObjectConstPtrMap attachable_objects_;
  std_msgs::ColorRGBA default_attached_object_color_;

  bool visible_;
  bool visual_visible_;
  bool collision_visible_;
};
typedef std::shared_ptr<StateVisualization> StateVisualizationPtr;
typedef std::shared_ptr<const StateVisualization> StateVisualizationConstPtr;

}

#endif //TESSERACT_RVIZ_STATE_VISUALIZATION
