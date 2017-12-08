#include "osgviewer/osgviewer.hpp"
#include <openrave-core.h>
using namespace OpenRAVE;
using namespace std;


int main() {
  RaveInitialize(true, OpenRAVE::Level_Debug);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  bool success = env->Load("data/pr2test2.env.xml");

  vector<RobotBasePtr> robots;
  env->GetRobots(robots);
  RobotBasePtr robot = robots[0];
  assert(success);

  OSGViewer* v = new OSGViewer(env);
  printf("1 robot\n");
  v->Idle();


  GraphHandlePtr h0 = v->PlotKinBody(robot);
  SetColor(h0, osg::Vec4(1,0,0,.2));
  robot->SetTransform(Transform(Vector(1,0,0,0), Vector(1,0,0)));
  printf("2 robots\n");
  v->Idle();

  GraphHandlePtr h1 = v->PlotKinBody(robot);
  SetTransparency(h1, .3);
  robot->SetTransform(Transform(Vector(1,0,0,0), Vector(-1,0,0)));
  printf("3 robots\n");
  v->Idle();


  delete v;

}
