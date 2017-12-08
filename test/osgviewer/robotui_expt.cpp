#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"
#include <openrave-core.h>
using namespace OpenRAVE;
using namespace std;


int main() {
  RaveInitialize(true, OpenRAVE::Level_Debug);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  bool success = env->Load("data/pr2test2.env.xml");
  assert(success);

  vector<RobotBasePtr> robots;
  env->GetRobots(robots);
  RobotBasePtr robot = robots[0];
  vector<RobotBase::ManipulatorPtr> manips = robot->GetManipulators();



  OSGViewerPtr viewer(new OSGViewer(env));
  ManipulatorControl mc(manips[manips.size()-1], viewer);
  DriveControl dc(robot, viewer);
  StatePrinter sp(robot);
  viewer->AddKeyCallback('a', boost::bind(&StatePrinter::PrintAll, &sp));

  viewer->Idle();



}
