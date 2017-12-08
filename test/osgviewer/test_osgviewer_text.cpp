#include "osgviewer/osgviewer.hpp"
#include <openrave-core.h>
using namespace OpenRAVE;
using namespace std;
int main() {
  RaveInitialize(false, Level_Info);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
  vector<GraphHandlePtr> handles;
  handles.push_back(viewer->drawtext("(100,100)", 100,100, 10, Vector(0,0,0,1)));
  handles.push_back(viewer->drawtext("(200,100)", 200,100, 10, Vector(1,0,0,1)));
  handles.push_back(viewer->drawtext("(300,100)", 300,100, 10, Vector(0,1,0,1)));
  handles.push_back(viewer->drawtext("(400,100)", 400,100, 10, Vector(0,0,1,1)));
  viewer->Idle();
  RaveDestroy();
}