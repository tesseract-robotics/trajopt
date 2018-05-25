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

/* Author: Dave Coleman */

#ifndef TESSERACT_RVIZ_TRAJECTORY_VISUALIZATION
#define TESSERACT_RVIZ_TRAJECTORY_VISUALIZATION

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <boost/thread/mutex.hpp>

#ifndef Q_MOC_RUN
#include <tesseract_rviz/render_tools/state_visualization.h>
#include <tesseract_rviz/render_tools/trajectory_panel.h>
#include <ros/ros.h>
#include <tesseract_msgs/Trajectory.h>
#endif

namespace rviz
{
class Shape;
class Property;
class IntProperty;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EditableEnumProperty;
class EnumProperty;
class ColorProperty;
class MovableText;
}

namespace tesseract_rviz
{
class Robot;

class TrajectoryVisualization : public QObject
{
  Q_OBJECT

public:
  /**
   * \brief Playback a trajectory from a planned path
   * \param widget - either a rviz::Display or rviz::Property
   * \param display - the rviz::Display from the parent
   * \return true on success
   */
  TrajectoryVisualization(rviz::Property* widget, rviz::Display* display);

  virtual ~TrajectoryVisualization();

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  void onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context, ros::NodeHandle update_nh);
  void onEnvLoaded(tesseract::tesseract_ros::ROSBasicEnvPtr env);
  void onEnable();
  void onDisable();
  void setName(const QString& name);

  void dropTrajectory();

public Q_SLOTS:
  void interruptCurrentDisplay();

private Q_SLOTS:

  /**
   * \brief Slot Event Functions
   */
  void changedDisplayPathVisualEnabled();
  void changedDisplayPathCollisionEnabled();
  void changedPathAlpha();
  void changedDisplayMode();
  void changedTrailStepSize();
  void changedTrajectoryTopic();
  void changedStateDisplayTime();
  void changedColor();
  void enabledColor();
  void trajectorySliderPanelVisibilityChange(bool enable);

protected:
  /**
   * \brief ROS callback for an incoming path message
   */
  void incomingDisplayTrajectory(const tesseract_msgs::Trajectory::ConstPtr& msg);
  float getStateDisplayTime();
  void clearTrajectoryTrail();
  void createTrajectoryTrail();

  // Handles actually drawing the robot along motion plans
  StateVisualizationPtr display_path_;

  // Handle colouring of robot
  void setColor(Robot* robot, const QColor& color);
  void unsetColor(Robot* robot);

  tesseract_msgs::TrajectoryPtr displaying_trajectory_message_;
  tesseract_msgs::TrajectoryPtr trajectory_message_to_display_;
  std::vector<tesseract_rviz::StateVisualizationPtr> trajectory_trail_;
  tesseract_rviz::StateVisualizationPtr trajectory_static_;

  ros::Subscriber trajectory_topic_sub_;
  boost::mutex update_trajectory_message_;

  tesseract::tesseract_ros::ROSBasicEnvPtr env_;

  // Pointers from parent display taht we save
  rviz::Property* widget_;
  rviz::Display* display_;  // the parent display that this class populates
  bool animating_path_;
  bool drop_displaying_trajectory_;
  int current_state_;
  float current_state_time_;
  Ogre::SceneNode* scene_node_;
  rviz::DisplayContext* context_;
  ros::NodeHandle update_nh_;
  TrajectoryPanel* trajectory_slider_panel_;
  rviz::PanelDockWidget* trajectory_slider_dock_panel_;

  // Properties
  rviz::BoolProperty* display_path_visual_enabled_property_;
  rviz::BoolProperty* display_path_collision_enabled_property_;
  rviz::EditableEnumProperty* state_display_time_property_;
  rviz::RosTopicProperty* trajectory_topic_property_;
  rviz::FloatProperty* path_alpha_property_;
  rviz::EnumProperty* display_mode_property_;
  rviz::BoolProperty* interrupt_display_property_;
  rviz::ColorProperty* default_color_property_;
  rviz::BoolProperty* enable_default_color_property_;
  rviz::IntProperty* trail_step_size_property_;
};
typedef std::shared_ptr<TrajectoryVisualization> TrajectoryVisualizationPtr;
typedef std::shared_ptr<const TrajectoryVisualization> TrajectoryVisualizationConstPtr;
}  // namespace tesseract_rviz

#endif //TESSERACT_RVIZ_TRAJECTORY_VISUALIZATION
