#include <trajopt_osgviewer/osgviewer.hpp>
#include <boost/foreach.hpp>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osg/Material>
#include <osg/Point>
#include <osg/LineWidth>
#include <osgUtil/SmoothingVisitor>
#include <cstdio>
#include <algorithm>
#include <osg/Array>
#include <osg/BlendFunc>
#include <osg/io_utils>
#include <iostream>
#include <osg/Camera>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/openrave_userdata_utils.hpp>
#include <osgText/Font>
#include <osgText/Text>

using namespace osg;
using namespace OpenRAVE;
using namespace std;

namespace {

osg::Matrix asOsgMatrix(const OpenRAVE::Transform& T) {
  osg::Matrix m;
  m.setTrans(osg::Vec3(T.trans.x, T.trans.y, T.trans.z));
  m.setRotate(osg::Vec4(T.rot[1], T.rot[2], T.rot[3], T.rot[0]));
  return m;
}
template <class T>
osg::Vec3 toOsgVec3(const RaveVector<T>& v) {
  return Vec3(v.x, v.y, v.z);
}
template <class T>
osg::Vec4 toOsgVec4(const RaveVector<T>& v) {
  return Vec4(v.x, v.y, v.z, v.w);
}

osg::Drawable* toOsgDrawable(const KinBody::Link::TRIMESH& mesh) {

  osg::Vec3Array* vec = new osg::Vec3Array();
  vec->resize( mesh.vertices.size());
  for(int i = 0; i < mesh.vertices.size(); ++i) {
    const Vector& v = mesh.vertices[i];
    (*vec)[i].set( v.x, v.y, v.z );
  }

  osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );
  for(int i = 0; i < mesh.indices.size(); ++i)
    deui->push_back( mesh.indices[ i ] );


  osg::Vec4Array* color = new osg::Vec4Array();
  color->push_back( osg::Vec4( 1., 1., 1., 1. ) );

  osg::Geometry* geom = new osg::Geometry;
  geom->setVertexArray( vec );
  geom->setColorArray( color );
  geom->setColorBinding( osg::Geometry::BIND_OVERALL );

  geom->addPrimitiveSet( deui );
  return geom;
}

osg::Node* osgNodeFromGeom(const KinBody::Link::Geometry& geom) {


  osg::Geode* geode = new osg::Geode;

  switch(geom.GetType()) {

  case KinBody::Link::GEOMPROPERTIES::GeomSphere: {

    osg::Sphere* s = new osg::Sphere();
    s->setRadius(geom.GetSphereRadius());
    osg::ShapeDrawable* sd = new osg::ShapeDrawable(s);
    geode->addDrawable(sd);
    break;
  }
  //  Geometry is defined like a Box
  case KinBody::Link::GEOMPROPERTIES::GeomBox: {

    osg::Box* box = new osg::Box();

    OpenRAVE::Vector v = geom.GetBoxExtents();
    box->setHalfLengths(osg::Vec3(v.x,v.y,v.z));
    osg::ShapeDrawable* sd = new osg::ShapeDrawable(box);
    geode->addDrawable(sd);
    break;
  }
  //  Geometry is defined like a Cylinder
  case KinBody::Link::GEOMPROPERTIES::GeomCylinder: {


    // make SoCylinder point towards z, not y
    osg::Cylinder* cy = new osg::Cylinder();
    cy->setRadius(geom.GetCylinderRadius());
    cy->setHeight(geom.GetCylinderHeight());
    osg::ShapeDrawable* sd = new osg::ShapeDrawable(cy);
    geode->addDrawable(sd);
    break;
  }
  //  Extract geometry from collision Mesh
  case KinBody::Link::GEOMPROPERTIES::GeomTrimesh: {
    // make triangleMesh
    osg::Drawable* mesh_drawable = toOsgDrawable(geom.GetCollisionMesh());
    geode->addDrawable(mesh_drawable);
    break;
  }
  default:
    LOG_ERROR("don't know how to make an osg node for geometry of type %i", geom.GetType());
    break;
  }

  osg::StateSet* state = geode->getOrCreateStateSet();
  osg::Material* mat = new osg::Material;
  OpenRAVE::Vector diffuse = geom.GetDiffuseColor();
  mat->setDiffuse( osg::Material::FRONT_AND_BACK, osg::Vec4(diffuse.x, diffuse.y, diffuse.z, 1) );
  OpenRAVE::Vector amb = geom.GetAmbientColor();
  mat->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4(amb.x,amb.y,amb.z,1)*.5 );
  mat->setTransparency(osg::Material::FRONT_AND_BACK,geom.GetTransparency());
  state->setAttribute(mat);

  osgUtil::SmoothingVisitor sv;
  geode->accept(sv);

  if (!geom.GetRenderFilename().empty()) {
    osg::Node* node = osgDB::readNodeFile(geom.GetRenderFilename());
    if (node) {
#if 0  // show collision mesh and collision mesh
      osg::Group* group = new osg::Group;
      group->addChild(node);
      group->addChild(geode);
      return group;
#else 
      return node;
#endif
    }
    else {
      LOG_ERROR("failed to load graphics mesh %s. Falling back to collision geom", geom.GetRenderFilename().c_str());
      return geode;
    }
  }
  else {
    return geode;
  }



}
MatrixTransform* osgNodeFromLink(const KinBody::Link& link) {
  /* each geom is a child */
  osg::MatrixTransform* link_node = new osg::MatrixTransform;
  const vector<KinBody::Link::GeometryPtr>& geoms = link.GetGeometries();
  for (int i=0; i < geoms.size(); ++i) {

    osg::Node* geom_node = osgNodeFromGeom(*geoms[i]);

    osg::Matrix m = asOsgMatrix( geoms[i]->GetTransform() );
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(m);
    mt->addChild(geom_node);
    link_node->addChild(mt);
  }
  return link_node;
}

class KinBodyGroup : public osg::Group {
public:
  vector<KinBody::LinkPtr> links; // links with geometry
  vector<osg::MatrixTransform*> nodes; // corresponding nodes
  void update() {
    for (int i=0; i < links.size(); ++i) {
      nodes[i]->setMatrix(asOsgMatrix(links[i]->GetTransform()));
    }
  }

};


KinBodyGroup* osgNodeFromKinBody(const KinBody& body) {
  /* each link is a child */
  KinBodyGroup* group = new KinBodyGroup;
  const vector<KinBody::LinkPtr>& links = body.GetLinks();
  for (int i=0; i < links.size(); ++i) {
    if (links[i]->GetGeometries().size() > 0) {
      MatrixTransform* link_node = osgNodeFromLink(*links[i]);
      group->addChild(link_node);
      group->links.push_back(links[i]);
      group->nodes.push_back(link_node);
    }
  }

  return group;
}



void AddLights(osg::Group* group) {
  {
    osg::Light* light = new osg::Light;
    light->setLightNum(0);
    light->setPosition(osg::Vec4(-4,0,4,1));
    osg::LightSource* lightSource = new osg::LightSource;
    lightSource->setLight(light);
    light->setDiffuse(osg::Vec4(1,.9,.9,1)*.5);
    light->setAmbient(osg::Vec4(1,1,1,1)*.3);
    light->setConstantAttenuation(0);
    light->setLinearAttenuation(.15);
    group->addChild(lightSource);
    group->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::ON);
  }

  {
    osg::Light* light = new osg::Light;
    light->setLightNum(1);
    light->setPosition(osg::Vec4(4,0,4,1));
    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
    lightSource->setLight(light);
    light->setDiffuse(osg::Vec4(.9,.9,1,1)*.5);
    light->setConstantAttenuation(0);
    light->setLinearAttenuation(.15);
    group->addChild(lightSource.get());
    group->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
  }

}


// http://forum.openscenegraph.org/viewtopic.php?t=7214
class SnapImageDrawCallback : public osg::Camera::DrawCallback {
public:
  SnapImageDrawCallback() { _snapImageOnNextFrame = false; }
  void setFileName(const std::string& filename) { _filename = filename; }
  const std::string& getFileName() const { return _filename; }
  void setSnapImageOnNextFrame(bool flag) { _snapImageOnNextFrame = flag; }
  bool getSnapImageOnNextFrame() const { return _snapImageOnNextFrame; }
  virtual void operator () (const osg::Camera& camera) const {
    if (!_snapImageOnNextFrame) return;
    int x,y,width,height;
    x = camera.getViewport()->x();
    y = camera.getViewport()->y();
    width = camera.getViewport()->width();
    height = camera.getViewport()->height();
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);
    // make a local copy
    unsigned char * p = image->data();
    unsigned int numBytes = image->computeNumComponents(image->getPixelFormat());
    _image = std::vector<unsigned char>(p, p + height*width*numBytes / sizeof(unsigned char));
    _height = height;
    _width = width;
    // save file
    if (_filename != "" && osgDB::writeImageFile(*image,_filename))
      std::cout << "Saved screenshot to `"<<_filename<<"`"<< std::endl;
    _snapImageOnNextFrame = false;
  }
  std::vector<unsigned char> getImage() const { return _image; }
  unsigned int getHeight() const { return _height; }
  unsigned int getWidth() const { return _width; }
protected:
  std::string _filename;
  mutable bool _snapImageOnNextFrame;
  mutable std::vector<unsigned char> _image;
  mutable unsigned int _height;
  mutable unsigned int _width;
};


// http://forum.openscenegraph.org/viewtopic.php?t=7806
void   AddCylinderBetweenPoints(const osg::Vec3& StartPoint, osg::Vec3 EndPoint, float radius, const osg::Vec4& CylinderColor, osg::Group *pAddToThisGroup, bool use_cone)
{
  osg::Vec3   z = osg::Vec3(0,0,1);
  osg::Vec3 p = (StartPoint - EndPoint);
  if (p.length() == 0) {
//    cerr << "tried to draw a cylinder of length 0" << endl;
    return;
  }
  p.normalize();

  osg::Vec3   t;
  double angle = acos( (z * p) );
  if (angle < 1e-6) t = z;
  else if (M_PI - angle < 1e-6) t = osg::Vec3(1,0,0);
  else {
    t = z ^  p;
    t.normalize();
  }

  if (use_cone) {
    osg::Vec3 pdir = p;
    pdir.normalize();
    EndPoint += pdir * 2*radius;
  }

   float height = (StartPoint- EndPoint).length();
   osg::Vec3 center = (StartPoint + EndPoint)/2;

   // This is the default direction for the cylinders to face in OpenGL
   osg::Cylinder* cylinder = new osg::Cylinder(center,radius,height);
   cylinder->setRotation(osg::Quat(angle, t));

   TessellationHints* hints = new TessellationHints;
   hints->setDetailRatio(.1);

   //   A geode to hold our cylinder
   osg::Geode* geode = new osg::Geode;
   osg::ShapeDrawable* cylinderDrawable = new osg::ShapeDrawable(cylinder, hints );
    geode->addDrawable(cylinderDrawable);

    if (use_cone) {
      osg::Vec3 cone_center = EndPoint;
      float cone_radius = 2*radius;
      float cone_height = -2*radius;
      osg::Cone* cone = new osg::Cone(cone_center, cone_radius, cone_height);
      cone->setRotation(osg::Quat(angle, t));
      osg::ShapeDrawable* coneDrawable = new osg::ShapeDrawable(cone, hints);
      geode->addDrawable(coneDrawable);

      osg::Sphere* sphere = new osg::Sphere(StartPoint, cone_radius);
      osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere, hints);
      geode->addDrawable(sphereDrawable);

    }

   //   Set the color of the cylinder that extends between the two points.
   osg::Material* pMaterial = new osg::Material;
   pMaterial->setDiffuse( osg::Material::FRONT, CylinderColor);
   geode->getOrCreateStateSet()->setAttribute( pMaterial, osg::StateAttribute::OVERRIDE );

   //   Add the cylinder between the two points to an existing group
   pAddToThisGroup->addChild(geode);
}


class SetColorsVisitor : public osg::NodeVisitor
{
public:
  osg::Vec4 color;
  SetColorsVisitor(const osg::Vec4& _color) : color(_color) {
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
  }
  void apply( osg::Geode& geode ) {
    StateSet* ss = geode.getOrCreateStateSet();
    osg::Material* mat = static_cast<Material*>(ss->getAttribute(StateAttribute::MATERIAL));
    if (!mat) {
      mat = new osg::Material;
      ss->setAttribute(mat);
    }
    mat->setAmbient(osg::Material::FRONT_AND_BACK, color);
    mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    if (color[3] < 1) {
      ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
      ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    }
  }
};
class SetTransparencyVisitor : public osg::NodeVisitor
{
public:
  float alpha;
  SetTransparencyVisitor(float _alpha) : alpha(_alpha) {
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
  }
  void apply( osg::Geode& geode ) {
    StateSet* ss = geode.getOrCreateStateSet();
    osg::Material* mat = static_cast<Material*>(ss->getAttribute(StateAttribute::MATERIAL));
    if (!mat) {
      mat = new osg::Material;
      ss->setAttribute(mat);
    }
    // mat->setTransparency(osg::Material::FRONT_AND_BACK, alpha);
    // for some reason setTransparency doesn't work so well
    osg::Vec4 amb = mat->getAmbient(Material::FRONT_AND_BACK);
    amb[3] = alpha;
    mat->setAmbient(Material::FRONT_AND_BACK,amb);
    osg::Vec4 diffuse = mat->getDiffuse(Material::FRONT_AND_BACK);
    diffuse[3] = alpha;
    mat->setDiffuse(Material::FRONT_AND_BACK, diffuse);
    if (alpha < 1) {
      ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
      ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    }
    else ss->setRenderingHint(StateSet::OPAQUE_BIN);

  }
};


class RefPtrHolder : public UserData {
public:
  osg::ref_ptr<osg::Referenced> rp;
  RefPtrHolder(osg::Referenced* x) {
    rp = x;
  }
};


class OsgGraphHandle : public OpenRAVE::GraphHandle {
public:
  osg::Group* parent;
  osg::ref_ptr<osg::Node> node;
  OsgGraphHandle(osg::Node* _node, osg::Group* _parent) : parent(_parent), node(_node) {
    parent->addChild(node);
  }
  ~OsgGraphHandle() {
    parent->removeChild(node);
  }
};

KinBodyGroup* GetOsgGroup(KinBody& body) {
  UserDataPtr rph = trajopt::GetUserData(body, "osg");
  return rph ? static_cast<KinBodyGroup*>(static_cast<RefPtrHolder*>(rph.get())->rp.get())
      : NULL;
}
KinBodyGroup* CreateOsgGroup(KinBody& body) {
//  assert(!GetUserData(body, "osg"));
  LOG_DEBUG("creating graphics for kinbody %s", body.GetName().c_str());
  osg::Node* node = osgNodeFromKinBody(body);
  UserDataPtr rph = UserDataPtr(new RefPtrHolder(node));
  trajopt::SetUserData(body, "osg", rph);
  return static_cast<KinBodyGroup*>(static_cast<RefPtrHolder*>(rph.get())->rp.get());
}

osg::ref_ptr<osg::Camera> createHUDCamera( double left, double right, double bottom, double top ) {
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
  camera->setClearMask( GL_DEPTH_BUFFER_BIT );
  camera->setRenderOrder( osg::Camera::POST_RENDER );
  camera->setAllowEventFocus( false );
  camera->setProjectionMatrix(osg::Matrix::ortho2D(left, right, bottom, top) );
  camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF );
  return camera;
}
osg::ref_ptr<osgText::Text> createText( const osg::Vec3& pos, const std::string& content, float size, const osg::Vec4& color ) {
  // static osg::ref_ptr<osgText::Font> g_font = osgText::readFontFile("fonts/arial.ttf");
  osg::ref_ptr<osgText::Text> text = new osgText::Text;
  // text->setFont( g_font.get() );
  text->setColor(color);
  text->setCharacterSize( size );
  text->setAxisAlignment( osgText::TextBase::XY_PLANE );
  text->setPosition( pos );
  text->setText( content );
  return text;
}

}

bool OSGViewer::EventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    bool suppressDefault = false;
    osgGA::GUIEventAdapter::EventType t = ea.getEventType();
    // keypress handlers
    if (t == osgGA::GUIEventAdapter::KEYDOWN) {
      int key = ea.getKey();
      KeyCallbackMap::iterator it = key_cbs.find(key);
      if (it != key_cbs.end()) (it->second)(ea);
    }

    // general event handlers
    for (int i=0; i < event_cbs.size(); ++i) suppressDefault |= event_cbs[i](ea);


    if (suppressDefault) return false;
    else return osgGA::TrackballManipulator::handle(ea, aa);
}


boost::shared_ptr<OSGViewer> OSGViewer::GetOrCreate(OpenRAVE::EnvironmentBasePtr env) {
  ViewerBasePtr viewer = env->GetViewer("osg");
  if (!viewer) {
    LOG_INFO("creating viewer");
    viewer = ViewerBasePtr(new OSGViewer(env));
    env->AddViewer(viewer);
  }
  else {
    LOG_INFO("already have a viewer for this environment");
  }
  return boost::dynamic_pointer_cast<OSGViewer>(viewer);
}


void throw_runtime_error(const osgGA::GUIEventAdapter&) {
  PRINT_AND_THROW("pressed escape");
}

OSGViewer::OSGViewer(EnvironmentBasePtr env) : ViewerBase(env), m_idling(false) {
  m_name = "osg";
  m_root = new Group;
//  m_viewer = new osgViewer::Viewer();
  m_viewer.setSceneData(m_root.get());
  m_viewer.setUpViewInWindow(0, 0, 640, 480);
  m_viewer.realize();
  m_cam = m_viewer.getCamera();
  m_handler = new EventHandler;
  m_viewer.setCameraManipulator(m_handler.get());
  AddLights(m_root);
  m_cam->setClearColor(osg::Vec4(1,1,1,1));

  osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = new SnapImageDrawCallback();
  m_cam->setPostDrawCallback (snapImageDrawCallback.get());

  AddKeyCallback('h', boost::bind(&OSGViewer::PrintHelp, this), "Display help");
  AddKeyCallback('p', boost::bind(&OSGViewer::Idle, this), "Toggle idle");
  AddKeyCallback('s', boost::bind(&OSGViewer::TakeScreenshot, this), "Take screenshot");
  AddKeyCallback(osgGA::GUIEventAdapter::KEY_Escape, &throw_runtime_error, "Quit (raise exception)");
  PrintHelp();
  m_viewer.setRunFrameScheme(osgViewer::ViewerBase::ON_DEMAND);
}

int OSGViewer::main(bool bShow) {
  UpdateSceneData();
  while (!m_viewer.done()) m_viewer.frame();
  return 0;
}

void OSGViewer::Idle() {
  UpdateSceneData();
  if (m_idling) { // stop idling
    m_request_stop_idling=true;
    return;
  }
  else { // start idling
    m_idling = true;
    m_request_stop_idling=false;
    printf("press p to stop idling\n");
    while (!m_viewer.done() && !m_request_stop_idling) {
      if (m_viewer.checkNeedToDoFrame()) m_viewer.frame();
      usleep(.03*1e6);
    }
    m_idling=false;
  }
}

void OSGViewer::Draw() {
  UpdateSceneData();
  m_viewer.frame();
}

void OSGViewer::RemoveKinBody(OpenRAVE::KinBodyPtr body) {
  KinBodyGroup* node = GetOsgGroup(*body);
  if (node) {
    m_root->removeChild(node);
    trajopt::RemoveUserData(*body, "osg");
  }
  else {
    LOG_ERROR("tried to remove kinbody that does not exist in osg");
  }
}


OSGViewer::~OSGViewer(){
}


void OSGViewer::UpdateSceneData() {
  vector<OpenRAVE::KinBodyPtr> bodies;
  GetEnv()->GetBodies(bodies);
  for (int i=0; i < bodies.size(); ++i) {
    KinBody& body = *bodies[i];
    KinBodyGroup* group = GetOsgGroup(body);
    if (!group) {
      group = CreateOsgGroup(body);
      m_root->addChild(group);
    }
    group->update();
  }
  m_viewer.requestRedraw();
}

void OSGViewer::SetBkgndColor(const RaveVectorf& color) {
  m_cam->setClearColor(toOsgVec4(color));
}

void OSGViewer::TakeScreenshot(const std::string& filename) {
  osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = dynamic_cast<SnapImageDrawCallback*> (m_cam->getPostDrawCallback());
  if(snapImageDrawCallback.get()) {
    snapImageDrawCallback->setFileName(filename);
    snapImageDrawCallback->setSnapImageOnNextFrame(true);
  } else {
    std::cout << "Warning: could not take screenshot" << std::endl;
  }
}

void OSGViewer::TakeScreenshot() {
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer,80,"%Y%m%d-%I%M%S.png",timeinfo);
  std::string filename(buffer);
  TakeScreenshot(filename);
}

bool OSGViewer::GetLastScreenshot(std::vector<unsigned char>& image, unsigned int& height, unsigned int& width)
{
  osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = dynamic_cast<SnapImageDrawCallback*> (m_cam->getPostDrawCallback());
  if(!snapImageDrawCallback.get())
    return false;
  image = snapImageDrawCallback->getImage();
  height = snapImageDrawCallback->getHeight();
  width = snapImageDrawCallback->getWidth();
  return true;
}

void OSGViewer::AddMouseCallback(const MouseCallback& cb) {
  m_handler->event_cbs.push_back(cb);
  // todo: print help
}
void OSGViewer::AddKeyCallback(int key, const KeyCallback& cb, const std::string& help) {
  EventHandler::KeyCallbackMap::iterator it = m_handler->key_cbs.find(key);
  if (it==m_handler->key_cbs.end()) {
    m_handler->key_cbs[key] = cb;
    m_handler->descs[key] = help;
  }
  else LOG_ERROR("error: key %c was already bound!", key);

}


void OSGViewer::PrintHelp() {
  stringstream ss;
  ss << "====== Welcome to the OSG Viewer =======\n"
  "Mouse: \n"
  "- rotate by holding left mouse button and dragging\n"
  "- zoom by holding right mouse button and dragging\n"
  "key bindings:\n"
  "Esc       quit application\n"
  "Space     reset view\n";
  for (EventHandler::Key2Desc::iterator it = m_handler->descs.begin(); it != m_handler->descs.end(); ++it) {
    ss << boost::format("%c:       %s\n")%((char)it->first)%(it->second);
  }
  LOG_INFO("%s", ss.str().c_str());
}


void SetColor(GraphHandlePtr handle, const osg::Vec4& color) {
  if (OsgGraphHandle* osghandle = dynamic_cast<OsgGraphHandle*>(handle.get())) {
    SetColorsVisitor visitor(color);
    osghandle->node->accept(visitor);
  }
}
void SetTransparency(GraphHandlePtr handle, float alpha) {
  if (OsgGraphHandle* osghandle = dynamic_cast<OsgGraphHandle*>(handle.get())) {
    SetTransparencyVisitor visitor(alpha);
    osghandle->node->accept(visitor);
  }
}

void OSGViewer::SetAllTransparency(float alpha) {
  UpdateSceneData();
  SetTransparencyVisitor visitor(alpha);
  m_root->accept(visitor);
}
void OSGViewer::SetTransparency(OpenRAVE::KinBodyPtr body, float alpha) {
  UpdateSceneData();
  KinBodyGroup* node = GetOsgGroup(*body);
  if (node) {
    SetTransparencyVisitor visitor(alpha);
    node->accept(visitor);
  }
  else {
    LOG_ERROR("SetTransparency: body doesn't exist in osg!");
  }
}

OpenRAVE::GraphHandlePtr OSGViewer::drawarrow(const RaveVectorf& p1, const RaveVectorf& p2, float fwidth, const RaveVectorf& color) {
  osg::Group* group = new osg::Group;
  AddCylinderBetweenPoints(toOsgVec3(p1), toOsgVec3(p2), fwidth, toOsgVec4(color), group, true);
  return GraphHandlePtr(new OsgGraphHandle(group, m_root.get()));
}

OpenRAVE::GraphHandlePtr OSGViewer::plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const RaveVector< float > &color, int drawstyle){
  vector<float> colordata(numPoints*3);
  for (int i=0; i < numPoints; ++i) {
    colordata[3*i] = color[0];
    colordata[3*i+1] = color[1];
    colordata[3*i+2] = color[2];
  }
  vector< RaveVectorf > colors(numPoints, color);
  return plot3(ppoints, numPoints, stride, fPointSize, colordata.data(), drawstyle);
}

OpenRAVE::GraphHandlePtr OSGViewer::plot3(const float* ppoints, int numPoints, int stride, float pointsize, const float* colors, int drawstyle, bool bhasalpha) {

  osg::Geometry* geom = new osg::Geometry;


  int floats_per_pt = stride / sizeof(float);

  Vec3Array* osgPts = new Vec3Array;
  osgPts->reserve(numPoints);
  for (int i=0; i < numPoints; ++i) {
    const float* p = ppoints + i*floats_per_pt;
    if (isfinite(p[0]))  osgPts->push_back(osg::Vec3(p[0], p[1], p[2]));
  }
  osg::StateSet* ss = geom->getOrCreateStateSet();
  osg::Point *point = new osg::Point();
  point->setSize(pointsize);
  ss->setAttribute(point);

  ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

  osg::BlendFunc* blendFunc = new osg::BlendFunc;
  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  ss->setAttributeAndModes(blendFunc);
  ss->setMode(GL_BLEND, osg::StateAttribute::ON);

  geom->setVertexArray(osgPts);
  geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,osgPts->size()));
//
  if (colors != NULL) {
    Vec4Array* osgCols = new Vec4Array;
    for (int i=0; i < numPoints; ++i) {
      const float* p = colors + i*3;
      osgCols->push_back(osg::Vec4(p[0], p[1], p[2],1));
    }
    geom->setColorArray(osgCols);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  }

  Geode* geode = new osg::Geode();
  geode->addDrawable(geom);

  return GraphHandlePtr(new OsgGraphHandle(geode, m_root.get()));
}


GraphHandlePtr OSGViewer::PlotKinBody(KinBodyPtr body) {
  UpdateSceneData();
  KinBodyGroup* orig = GetOsgGroup(*body);
  /* Note: we could easily plot a kinbody that's not part of the environment, but
    there would be a problem if you plot something and then add it later
   */
  OPENRAVE_ASSERT_FORMAT0(orig != NULL, "kinbody not part of scene graph", ORE_Assert);
  orig->update();
  osg::Group* copy = new osg::Group(*orig, CopyOp(CopyOp::DEEP_COPY_NODES | CopyOp::DEEP_COPY_STATESETS | CopyOp::DEEP_COPY_STATEATTRIBUTES));
  OsgGraphHandle* handle = new OsgGraphHandle(copy, m_root.get());
  return GraphHandlePtr(handle);
}

GraphHandlePtr OSGViewer::PlotLink(KinBody::LinkPtr link) {
  KinBodyPtr body = link->GetParent();
  KinBodyGroup* orig = GetOsgGroup(*body);
  OPENRAVE_ASSERT_FORMAT0(orig != NULL, "kinbody not part of scene graph", ORE_Assert);
  orig->update();

  vector<KinBody::LinkPtr>::iterator it = std::find(orig->links.begin(), orig->links.end(), link);
  if (it == orig->links.end()) {
    LOG_ERROR("want to plot link %s, can't find it", link->GetName().c_str());
    return GraphHandlePtr();
  }

  osg::Node* orig_node = orig->nodes[it - orig->links.begin()];
  osg::Node* copy = static_cast<Group*>(orig_node->clone(CopyOp(CopyOp::DEEP_COPY_NODES)));
  OsgGraphHandle* handle = new OsgGraphHandle(copy, m_root.get());
  return GraphHandlePtr(handle);
}

GraphHandlePtr OSGViewer::PlotAxes(const OpenRAVE::Transform& T, float size) {
  osg::Matrix m = asOsgMatrix(T);
  osg::Group* group = new osg::Group;
  osg::Vec3 o = toOsgVec3(T.trans);
  osg::Vec3 x = o + Vec3(m(0,0), m(1,0), m(2,0))*size;
  osg::Vec3 y = o + Vec3(m(0,1), m(1,1), m(2,1))*size;
  osg::Vec3 z = o + Vec3(m(0,2), m(1,2), m(2,2))*size;
  AddCylinderBetweenPoints(o, x, size/10, osg::Vec4(1,0,0,1), group, false);
  AddCylinderBetweenPoints(o, y, size/10, osg::Vec4(0,1,0,1), group, false);
  AddCylinderBetweenPoints(o, z, size/10, osg::Vec4(0,0,1,1), group, false);
  return GraphHandlePtr(new OsgGraphHandle(group, m_root.get()));
}
GraphHandlePtr OSGViewer::PlotSphere(const OpenRAVE::Vector& pt, float radius) {
  osg::Geode* geode = new osg::Geode;
  osg::Sphere* sphere = new osg::Sphere(toOsgVec3(pt), radius);
  osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
  geode->addDrawable(sphereDrawable);
  return GraphHandlePtr(new OsgGraphHandle(geode, m_root.get()));
  
}

OpenRAVE::GraphHandlePtr OSGViewer::drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const RaveVectorf &color) {

  osg::DrawElementsUInt* deui = new osg::DrawElementsUInt(GL_TRIANGLES);
  osg::Vec3Array* vec = new osg::Vec3Array();
  osg::Vec3Array& points = *vec;

  if (pIndices == NULL) {
    vec->resize(numTriangles * 3);
    for (int i = 0; i < 3 * numTriangles; ++i) {
      points[i].set(ppoints[0], ppoints[1], ppoints[2]);
      ppoints = (float*) ((char*) ppoints + stride);
      deui->push_back(i);
    }
  }
  else {
    int nverts = *std::max_element(pIndices, pIndices + numTriangles * 3) + 1;
    vec->resize(nverts);
    cout << "number of vertices: " << nverts << endl;
    for (int i = 0; i < nverts; ++i) {
      const float* p = ppoints + i*stride/sizeof(float);
      points[i].set(p[0], p[1], p[2]);
    }
    for (int i = 0; i < numTriangles * 3; ++i) {
      deui->push_back(pIndices[i]);
    }
  }



  osg::Vec4Array* colors = new osg::Vec4Array();
  colors->push_back( osg::Vec4( color[0],color[1], color[2], color[3] ) );

  osg::Geometry* geom = new osg::Geometry;
  geom->setVertexArray( vec );
  geom->setColorArray( colors );
  geom->setColorBinding( osg::Geometry::BIND_OVERALL );

  geom->addPrimitiveSet( deui );

  osg::Geode* geode = new osg::Geode;
  geode->addDrawable(geom);

  osg::StateSet* state = geode->getOrCreateStateSet();
  osg::Material* mat = new osg::Material;
  OpenRAVE::Vector diffuse = color;
  mat->setDiffuse( osg::Material::FRONT_AND_BACK, toOsgVec4(diffuse) );
  OpenRAVE::Vector amb = color;
  mat->setAmbient( osg::Material::FRONT_AND_BACK, toOsgVec4(amb)*.5 );
  mat->setTransparency(osg::Material::FRONT_AND_BACK,color[3]);
  state->setAttribute(mat);
  if (color[3] < 1) {
    state->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
  }

  osgUtil::SmoothingVisitor sv;
  geode->accept(sv);




  return GraphHandlePtr(new OsgGraphHandle(geode, m_root.get()));
}

OpenRAVE::GraphHandlePtr  OSGViewer::_drawlines(osg::PrimitiveSet::Mode mode, const float *ppoints,  int numPoints, int stride, float fwidth, const RaveVectorf &color) {

  osg::Geometry* geom = new osg::Geometry;

  int floats_per_pt = stride / sizeof(float);

  Vec3Array* osgPts = new Vec3Array;
  osgPts->reserve(numPoints);
  for (int i=0; i < numPoints; ++i) {
    const float* p = ppoints + i*floats_per_pt;
    if (isfinite(p[0]))  osgPts->push_back(osg::Vec3(p[0], p[1], p[2]));
  }
  osg::StateSet* ss = geom->getOrCreateStateSet();
  osg::LineWidth *lw= new osg::LineWidth;
  lw->setWidth(fwidth);
  ss->setAttribute(lw);

  ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

  osg::BlendFunc* blendFunc = new osg::BlendFunc;
  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  ss->setAttributeAndModes(blendFunc);
  ss->setMode(GL_BLEND, osg::StateAttribute::ON);

  osg::Vec4Array* osgColor = new osg::Vec4Array();
  osgColor->push_back( osg::Vec4( color[0], color[1], color[2], color[3] ) );
  geom->setColorArray(osgColor);
  geom->setColorBinding(osg::Geometry::BIND_OVERALL);

  geom->setVertexArray(osgPts);
  geom->addPrimitiveSet(new osg::DrawArrays(mode,0,osgPts->size()));


  Geode* geode = new osg::Geode();
  geode->addDrawable(geom);

  return GraphHandlePtr(new OsgGraphHandle(geode, m_root.get()));
}

OpenRAVE::GraphHandlePtr  OSGViewer::drawlinestrip(const float *ppoints,  int numPoints, int stride, float fwidth, const RaveVectorf &color) {
  return _drawlines(osg::PrimitiveSet::LINE_STRIP, ppoints, numPoints, stride, fwidth, color);
}
OpenRAVE::GraphHandlePtr  OSGViewer::drawlinelist(const float *ppoints,  int numPoints, int stride, float fwidth, const RaveVectorf &color) {
  return _drawlines(osg::PrimitiveSet::LINES, ppoints, numPoints, stride, fwidth, color);
}


OpenRAVE::GraphHandlePtr OSGViewer::drawtext(const std::string& text, float x, float y, float fontsize, const OpenRAVE::Vector& color) {
  osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
  textGeode->addDrawable( createText(osg::Vec3(x, y, 0.0f),text,fontsize, toOsgVec4(color)));
  if (!m_hudcam) {
    m_hudcam = createHUDCamera(0, 1024, 0, 768);
    m_root->addChild(m_hudcam);
  }
  return GraphHandlePtr(new OsgGraphHandle(textGeode, m_hudcam.get()));

    
}
