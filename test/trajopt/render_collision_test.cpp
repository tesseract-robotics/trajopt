#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"
#include <openrave-core.h>
#include "trajopt/collision_checker.hpp"
#include <cmath>
using namespace OpenRAVE;
using namespace std;
using namespace trajopt;

CollisionCheckerPtr cc;
vector<GraphHandlePtr> handles;
EnvironmentBasePtr env;
OSGViewerPtr viewer;
void PlotCollisionGeometry(const osgGA::GUIEventAdapter & ea) {
  if (handles.size() == 0) {

    cc->PlotCollisionGeometry(handles);
    vector<Collision> collisions;
    cc->AllVsAll(collisions);
    PlotCollisions(collisions, *env, handles, .02);
  }

  else
    handles.clear();
}

float alpha = 1;
void AdjustTransparency(float da) {
  alpha += da;
  alpha = fmin(alpha, 1);
  alpha = fmax(alpha, 0);
  viewer->SetAllTransparency(alpha);
}

int main() {
  RaveInitialize(false, OpenRAVE::Level_Debug);
  env = RaveCreateEnvironment();
  env->StopSimulation();
//  bool success = env->Load("data/pr2test2.env.xml");
  {
    bool success = env->Load("/home/joschu/Proj/drc/gfe.xml");
    FAIL_IF_FALSE(success);
  }
  {
    bool success = env->Load("/home/joschu/Proj/trajopt/data/test2.env.xml");
    FAIL_IF_FALSE(success);
  }
  vector<RobotBasePtr> robots;
  env->GetRobots(robots);
  RobotBasePtr robot = robots[0];
  vector<RobotBase::ManipulatorPtr> manips = robot->GetManipulators();


  cc = CollisionChecker::GetOrCreate(*env);
  viewer.reset(new OSGViewer(env));
  env->AddViewer(viewer);



  ManipulatorControl mc(manips[manips.size()-1], viewer);
  DriveControl dc(robot, viewer);
  StatePrinter sp(robot);
  viewer->AddKeyCallback('a', boost::bind(&StatePrinter::PrintAll, &sp));
  viewer->AddKeyCallback('q', &PlotCollisionGeometry);
  viewer->AddKeyCallback('=', boost::bind(&AdjustTransparency, .05));
  viewer->AddKeyCallback('-', boost::bind(&AdjustTransparency, -.05));

  viewer->Idle();

  env.reset();
  viewer.reset();
  RaveDestroy();

}
