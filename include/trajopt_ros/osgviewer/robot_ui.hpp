#pragma once
#include "osgviewer.hpp"

class TRAJOPT_API ManipulatorControl {
public:
  ManipulatorControl(OpenRAVE::RobotBase::ManipulatorPtr manip, OSGViewerPtr viewer);
  bool ProcessMouseInput(const osgGA::GUIEventAdapter &ea);
private:
  OpenRAVE::RobotBase::ManipulatorPtr m_manip;
  OSGViewerPtr m_viewer;
  float lastX, lastY;
};

class TRAJOPT_API DriveControl {
public:
  DriveControl(OpenRAVE::RobotBasePtr robot, OSGViewerPtr viewer);
  OpenRAVE::RobotBasePtr m_robot;
  OSGViewerPtr m_viewer;
  void MoveRobot(float dx, float dy, float dtheta);
};

class TRAJOPT_API StatePrinter {
public:
  typedef OpenRAVE::RobotBasePtr RobotPtr;
  StatePrinter(RobotPtr robot) : m_robot(robot) {}
  void PrintAll();
private:
  RobotPtr m_robot;

};
