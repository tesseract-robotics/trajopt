#include <osg/io_utils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
using namespace osg;

osgViewer::Viewer viewer;

int main(int argc, char* argv[]) {
  ref_ptr<Group> root = new Group;
  viewer.setUpViewInWindow(0, 0, 640, 480);
  viewer.realize();
  ref_ptr<Camera> cam = viewer.getCamera();
  ref_ptr<Geode> geode = new Geode;
  root->addChild(geode.get());



  for (int i=0; i < 10; i++) {
    osg::Sphere* sphere = new osg::Sphere( Vec3f(i+1,10,0), .1*i);
    osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
    geode->addDrawable(sphereDrawable);
  }

  // osg::Sphere* sphere = new osg::Sphere( Vec3f(10,10,0), .2);
  // osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
  // sphereDrawable->setColor(osg::Vec4f(1,0,0,1));
  // geode->addDrawable(sphereDrawable);

  ref_ptr<osgGA::TrackballManipulator> manip = new osgGA::TrackballManipulator();


    viewer.setCameraManipulator(manip);

  viewer.setSceneData(root.get());
  cam->setViewMatrixAsLookAt(Vec3f(10,0,0), Vec3f(10,1,0), Vec3f(0,0,1));
  //manip->setHomePosition(Vec3f(10,0,0), Vec3f(11,1,0), Vec3f(10,0,1));
  //  cam->setProjectionMatrixAsPerspective(49,640/480., .1, 10);

  viewer.run();


}
