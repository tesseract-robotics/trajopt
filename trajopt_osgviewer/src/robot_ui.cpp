#include <trajopt_osgviewer/robot_ui.hpp>
using namespace osg;
using namespace std;
namespace OR = OpenRAVE;
#include <boost/foreach.hpp>
#include <trajopt_utils/stl_to_string.hpp>
using namespace util;

namespace {
template <class T>
osg::Vec3 toOsgVec3(const OR::RaveVector<T>& v) {
  return Vec3(v.x, v.y, v.z);
}
template <class T>
osg::Vec4 toOsgVec4(const OR::RaveVector<T>& v) {
  return Vec4(v.x, v.y, v.z, v.w);
}

}

OpenRAVE::Vector toRave(const osg::Vec3& v) {
  return OpenRAVE::Vector(v.x(), v.y(), v.z());
}
vector<double> GetDOFValues(const OR::RobotBase::Manipulator& manip) {
  vector<double> vals;
  manip.GetRobot()->GetDOFValues(vals, manip.GetArmIndices());
  return vals;
}


ManipulatorControl::ManipulatorControl(OpenRAVE::RobotBase::ManipulatorPtr manip, OSGViewerPtr viewer) :
      m_manip(manip), m_viewer(viewer) {
  viewer->AddMouseCallback(boost::bind(&ManipulatorControl::ProcessMouseInput, this, _1));
}

bool ManipulatorControl::ProcessMouseInput(const osgGA::GUIEventAdapter &ea) {

  if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
    lastX = ea.getX();
    lastY = ea.getY();
  }
  else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
    const bool ctrl( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) != 0 );
    if (!ctrl) return false;
    // drag the active manipulator in the plane of view
    // get our current view

    float dx = ea.getX() - lastX;
    lastX = ea.getX();
    float dy = ea.getY() - lastY;
    lastY = ea.getY();




    Vec3d from, to, up;
    m_viewer->m_handler->getTransformation(from, to, up);
    up.normalize();
    Vec3d depthdir = (to - from); depthdir.normalize();
    osg::Vec3 ydir = up;
    osg::Vec3 xdir = depthdir ^ ydir;


    OpenRAVE::Transform T = m_manip->GetEndEffectorTransform();
    float depth = (toOsgVec3(T.trans)-from) * depthdir;
    osg::Vec3 dragvec = xdir*(depth*dx/ea.getWindowWidth()) + ydir*(depth*dy/ea.getWindowHeight());
    cout << toRave(dragvec) << endl;

    T.trans += toRave(dragvec);
    vector<double> iksoln;
    m_manip->FindIKSolution(OR::IkParameterization(T), iksoln, 18);
    if (iksoln.empty()) {
      cerr << "no ik solution found" << endl;
    }
    else {
      cout << "ik succeeded!" << endl;
      m_manip->GetRobot()->SetDOFValues(iksoln, false, m_manip->GetArmIndices());
      m_viewer->UpdateSceneData();
    }
    return true;
  }
  return false;

}

DriveControl::DriveControl(OpenRAVE::RobotBasePtr robot, OSGViewerPtr viewer) :
        m_robot(robot), m_viewer(viewer) {
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(&DriveControl::MoveRobot, this, 0,.05, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(&DriveControl::MoveRobot,this, 0,-.05, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(&DriveControl::MoveRobot, this,.05,0, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(&DriveControl::MoveRobot, this,-.05,0, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Leftbracket, boost::bind(&DriveControl::MoveRobot, this,0,0, .05));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Rightbracket, boost::bind(&DriveControl::MoveRobot, this,0,0, -.05));
}

void DriveControl::MoveRobot(float dx, float dy, float dtheta) {
  OR::Transform T = m_robot->GetTransform();
  T.trans += OpenRAVE::Vector(dx, dy, 0);
  T.rot = OpenRAVE::geometry::quatMultiply(T.rot, OpenRAVE::Vector(1,0,0,dtheta/2));
  T.rot.normalize4();
  cout << "rotation " << T.rot << endl;
  m_robot->SetTransform(T);
  m_viewer->UpdateSceneData();
}

void StatePrinter::PrintAll() {
  cout << "joints: " << endl;
  BOOST_FOREACH(const OR::KinBody::JointPtr& joint, m_robot->GetJoints()) {
    cout << joint->GetName() << ": " << Str(joint->GetValue(0)) << endl;
  }
  vector<double> dofvals; m_robot->GetDOFValues(dofvals);
  cout << "all dof vals: " << Str(dofvals) << endl;
  cout << "transform: " << m_robot->GetTransform() << endl;
  cout << "links: " << endl;
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, m_robot->GetLinks()) {
    cout << link->GetName() << ": " << link->GetTransform() << endl;
  }
}

