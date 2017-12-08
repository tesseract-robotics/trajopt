#pragma once
#include <openrave/openrave.h>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <boost/function.hpp>
#include <osgGA/TrackballManipulator>
#include <trajopt_ros/utils/macros.h>
typedef OpenRAVE::RaveVector<float> RaveVectorf;

/**
Viewer class with some extras for rendering whole robots.
*/
class TRAJOPT_API OSGViewer : public OpenRAVE::ViewerBase {
public:

  static boost::shared_ptr<OSGViewer> GetOrCreate(OpenRAVE::EnvironmentBasePtr env);
  OSGViewer(OpenRAVE::EnvironmentBasePtr env);
  ~OSGViewer();
  virtual int main(bool bShow = true);
  virtual void quitmainloop() {}
  void RemoveKinBody(OpenRAVE::KinBodyPtr pbody);
  void Draw();
  void Idle(); // should be called ToggleIdle
  void UpdateSceneData();
  const std::string& GetName() const {return m_name;}
  void SetBkgndColor(const RaveVectorf& color);
  void TakeScreenshot(const std::string& filename);
  void TakeScreenshot();
  bool GetLastScreenshot(std::vector<unsigned char>& image, unsigned int& height, unsigned int& width);

  // return false if you want to disable the default TrackballManipulator handling
  typedef boost::function<bool(const osgGA::GUIEventAdapter &)> MouseCallback;
  void AddMouseCallback(const MouseCallback& cb);
  typedef boost::function<void(const osgGA::GUIEventAdapter &)> KeyCallback;
  void AddKeyCallback(int key, const KeyCallback& cb, const std::string& help="");
  void PrintHelp();

  OpenRAVE::GraphHandlePtr drawarrow(const RaveVectorf& p1, const RaveVectorf& p2, float fwidth, const RaveVectorf& color);
  OpenRAVE::GraphHandlePtr  drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const RaveVectorf &color);
//  OpenRAVE::GraphHandlePtr drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors);
  OpenRAVE::GraphHandlePtr plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle=0);
  OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float pointsize, const float* colors, int drawstyle=0, bool bhasalpha=false);
  OpenRAVE::GraphHandlePtr  drawlinelist(const float *ppoints, int numPoints, int stride, float fwidth, const RaveVectorf &color);
  OpenRAVE::GraphHandlePtr  drawlinestrip(const float *ppoints, int numPoints, int stride, float fwidth, const RaveVectorf &color);
  OpenRAVE::GraphHandlePtr  _drawlines(osg::PrimitiveSet::Mode mode, const float *ppoints, int numPoints, int stride, float fwidth, const RaveVectorf &color);
  OpenRAVE::GraphHandlePtr drawtext(const std::string& text, float x, float y, float fontsize, const OpenRAVE::Vector& color);

  void SetAllTransparency(float alpha);
  void SetTransparency(OpenRAVE::KinBodyPtr, float alpha);

  OpenRAVE::GraphHandlePtr PlotAxes(const OpenRAVE::Transform& T, float size);
  OpenRAVE::GraphHandlePtr PlotSphere(const OpenRAVE::Vector& x, float radius);

  /** copy current state of kinbody or link */
  OpenRAVE::GraphHandlePtr PlotKinBody(const OpenRAVE::KinBodyPtr);
  OpenRAVE::GraphHandlePtr PlotLink(const OpenRAVE::KinBody::LinkPtr);

  struct EventHandler : public osgGA::TrackballManipulator {
    typedef std::map<int, OSGViewer::KeyCallback> KeyCallbackMap;
    std::vector<MouseCallback> event_cbs;
    KeyCallbackMap key_cbs;
    typedef std::map<int, std::string> Key2Desc;
    Key2Desc descs; // callback descriptions
    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
  };


  osg::ref_ptr<EventHandler> m_handler;
  osg::ref_ptr<osg::Group> m_root;
  private:

//  osg::ref_ptr<osgViewer::Viewer> m_viewer;
  osg::ref_ptr<osg::Camera> m_cam, m_hudcam;
  osgViewer::Viewer m_viewer;
  bool m_idling, m_request_stop_idling;
  std::string m_name;


};
typedef boost::shared_ptr<OSGViewer> OSGViewerPtr;


void TRAJOPT_API SetColor(OpenRAVE::GraphHandlePtr handle, const osg::Vec4& color);
void TRAJOPT_API SetTransparency(OpenRAVE::GraphHandlePtr handle, float alpha);
