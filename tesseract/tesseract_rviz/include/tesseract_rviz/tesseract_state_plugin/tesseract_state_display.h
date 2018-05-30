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

#ifndef TESSERACT_RVIZ_TESSERACT_STATE_DISPLAY_PLUGIN
#define TESSERACT_RVIZ_TESSERACT_STATE_DISPLAY_PLUGIN

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include "tesseract_rviz/render_tools/state_visualization.h"
#include <tesseract_msgs/TesseractState.h>
#include <srdfdom/model.h>
#include <ros/ros.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
}

namespace tesseract_rviz
{
class StateVisualization;
class Robot;

class TesseractStateDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TesseractStateDisplay();
  virtual ~TesseractStateDisplay();

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  const tesseract::tesseract_ros::ROSBasicEnvConstPtr getEnv() const { return env_; }
  void setLinkColor(const std::string& link_name, const QColor& color);
  void unsetLinkColor(const std::string& link_name);

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void changedURDFDescription();
  void changedRootLinkName();
  void changedURDFSceneAlpha();
  void changedAttachedBodyColor();
  void changedTesseractStateTopic();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:
  void loadURDFModel();

  /**
   * \brief Set the scene node's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors);
  void setLinkColor(Robot* robot, const std::string& link_name, const QColor& color);
  void unsetLinkColor(Robot* robot, const std::string& link_name);

  void newTesseractStateCallback(const tesseract_msgs::TesseractStateConstPtr state);

  void setHighlightedLinks(const tesseract_msgs::TesseractState::_highlight_links_type& highlight_links);
  void setHighlightedLink(const std::string& link_name, const std_msgs::ColorRGBA& color);
  void unsetHighlightedLink(const std::string& link_name);

  // overrides from Display
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();

  // render the robot
  ros::NodeHandle nh_;
  ros::Subscriber tesseract_state_subscriber_;

  tesseract::tesseract_ros::ROSBasicEnvPtr env_;
  StateVisualizationPtr state_;
  std::map<std::string, std_msgs::ColorRGBA> highlights_;
  bool update_state_;
  bool load_env_;  // for delayed initialization

  rviz::StringProperty* urdf_description_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::RosTopicProperty* tesseract_state_topic_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::ColorProperty* attached_body_color_property_;
  rviz::BoolProperty* enable_link_highlight_;
  rviz::BoolProperty* enable_visual_visible_;
  rviz::BoolProperty* enable_collision_visible_;
  rviz::BoolProperty* show_all_links_;
};
typedef std::shared_ptr<TesseractStateDisplay> TesseractStateDisplayPtr;
typedef std::shared_ptr<const TesseractStateDisplay> TesseractStateDisplayConstPtr;

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_TESSERACT_STATE_DISPLAY_PLUGIN
