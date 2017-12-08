#include <trajopt_ros/trajopt/collision_checker.hpp>
#include <trajopt_ros/utils/eigen_conversions.hpp>
#include <trajopt_ros/utils/stl_to_string.hpp>
#include <trajopt_ros/utils/logging.hpp>
#include <trajopt_ros/openrave_userdata_utils.hpp>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <openrave-core.h>
#include <boost/foreach.hpp>
#include <vector>
#include <iostream>
#include <LinearMath/btConvexHull.h>

using namespace util;
using namespace std;
using namespace trajopt;
using namespace OpenRAVE;

namespace {

#define METERS 
// there's some scale-dependent parameters. By convention I'll put METERS to mark it
const float MARGIN = 0;

#if 1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
ostream &operator<<(ostream &stream, const btVector3& v) {
  stream << v.x() << " " << v.y() << " " << v.z();
  return stream;
}
ostream &operator<<(ostream &stream, const btQuaternion& v) {
  stream << v.w() << " " << v.x() << " " << v.y() << " " << v.z();
  return stream;
}
ostream &operator<<(ostream &stream, const btTransform& v) {
  stream << v.getOrigin() << " " << v.getRotation();
  return stream;
}
#pragma GCC diagnostic pop
#endif

class CollisionObjectWrapper : public btCollisionObject {
public:
  CollisionObjectWrapper(KinBody::Link* link) : m_link(link), m_index(-1) {}
  vector<boost::shared_ptr<void> > m_data;
  KinBody::Link* m_link;
  int m_index; // index into collision matrix
  template<class T>
  void manage(T* t) { // manage memory of this object
    m_data.push_back(boost::shared_ptr<T>(t));
  }
  template<class T>
  void manage(boost::shared_ptr<T> t) {
    m_data.push_back(t);
  }
};
typedef CollisionObjectWrapper COW;
typedef boost::shared_ptr<CollisionObjectWrapper> COWPtr;

inline const KinBody::Link* getLink(const btCollisionObject* o) {
  return static_cast<const CollisionObjectWrapper*>(o)->m_link;
}


extern void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);




btVector3 toBt(const OR::Vector& v){
  return btVector3(v[0], v[1], v[2]);
}
OR::Vector toOR(const btVector3& v) {
  return OR::Vector(v.x(), v.y(), v.z());
}
btQuaternion toBtQuat(const OR::Vector& q) {
  return btQuaternion(q[1], q[2], q[3], q[0]);
}
btTransform toBt(const OR::Transform& t){
  return btTransform(toBtQuat(t.rot), toBt(t.trans));
}

bool isIdentity(const OpenRAVE::Transform& T) {
  float e = 1e-6;
  return
      fabs(T.trans.x) < e &&
      fabs(T.trans.y) < e &&
      fabs(T.trans.z) < e &&
      fabs(T.rot[0]-1) < e &&
      fabs(T.rot[1]) < e &&
      fabs(T.rot[2]) < e &&
      fabs(T.rot[3]) < e;
}


void GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport, btVector3& outpt) {
  btVector3 ptSum(0,0,0);
  float ptCount = 0;
  float maxSupport=-1000;
  const float EPSILON = 1e-3;
  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape) {
    int nPts = pshape->getNumVertices();

    for (int i=0; i < nPts; ++i) {
      btVector3 pt;
      pshape->getVertex(i, pt);
//      cout << "pt: " << pt << endl;
      float sup  = pt.dot(localNormal);
      if (sup > maxSupport + EPSILON) {
        ptCount=1;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup < maxSupport - EPSILON) {
      }
      else {
        ptCount += 1;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
  }
  else  {
    outpt = shape->localGetSupportingVertexWithoutMargin(localNormal);
    outsupport = localNormal.dot(outpt);
  }
}


btCollisionShape* createShapePrimitive(OR::KinBody::Link::GeometryPtr geom, bool useTrimesh, CollisionObjectWrapper* cow) {

  btCollisionShape* subshape=0;

#if OPENRAVE_VERSION_MINOR <= 8
    #define GT_Box KinBody::Link::GEOMPROPERTIES::GeomBox 
    #define GT_Sphere KinBody::Link::GEOMPROPERTIES::GeomSphere 
    #define GT_Cylinder KinBody::Link::GEOMPROPERTIES::GeomCylinder 
    #define GT_TriMesh KinBody::Link::GEOMPROPERTIES::GeomTrimesh 
    #define TriMesh KinBody::Link::TRIMESH
#endif

  switch (geom->GetType()) {
  case OpenRAVE::GT_Box:
    subshape = new btBoxShape(toBt(geom->GetBoxExtents()));
    break;
  case OpenRAVE::GT_Sphere:
    subshape = new btSphereShape(geom->GetSphereRadius());
    break;
  case OpenRAVE::GT_Cylinder:
    // cylinder axis aligned to Y
  {
    float r = geom->GetCylinderRadius(), h = geom->GetCylinderHeight() / 2;
    subshape = new btCylinderShapeZ(btVector3(r, r, h));
    break;
  }
  case OpenRAVE::GT_TriMesh: {
    const OpenRAVE::TriMesh &mesh = geom->GetCollisionMesh();
    assert(mesh.indices.size() >= 3);
    boost::shared_ptr<btTriangleMesh> ptrimesh(new btTriangleMesh());

    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
      ptrimesh->addTriangle(toBt(mesh.vertices[mesh.indices[i]]), toBt(mesh.vertices[mesh.indices[i + 1]]),
              toBt(mesh.vertices[mesh.indices[i + 2]]));
    }

    if (useTrimesh) {
      subshape = new btBvhTriangleMeshShape(ptrimesh.get(), true);
      cow->manage(ptrimesh);
    }
    else { // CONVEX HULL
      btConvexTriangleMeshShape convexTrimesh(ptrimesh.get());
      convexTrimesh.setMargin(MARGIN); // margin: hull padding
      //Create a hull shape to approximate Trimesh

      bool useShapeHull;

      btShapeHull shapeHull(&convexTrimesh);
      if (mesh.vertices.size() >= 50) {
        bool success = shapeHull.buildHull(-666); // note: margin argument not used
        if (!success) LOG_WARN("shapehull convex hull failed! falling back to original vertices");
        useShapeHull = success;
      }
      else {
        useShapeHull = false;
      }

      btConvexHullShape *convexShape = new btConvexHullShape();
      subshape = convexShape;
      if (useShapeHull) {
        for (int i = 0; i < shapeHull.numVertices(); ++i)
          convexShape->addPoint(shapeHull.getVertexPointer()[i]);
        break;
      }
      else {
        for (int i = 0; i < mesh.vertices.size(); ++i)
          convexShape->addPoint(toBt(mesh.vertices[i]));
        break;
      }
      
    }     
    break;
  }
  default:
    assert(0 && "unrecognized collision shape type");
    break;
  }
  return subshape;
}


COWPtr CollisionObjectFromLink(OR::KinBody::LinkPtr link, bool useTrimesh) {
  LOG_DEBUG("creating bt collision object from from %s",link->GetName().c_str());

  const std::vector<boost::shared_ptr<OpenRAVE::KinBody::Link::Geometry> > & geometries=link->GetGeometries();

  if (geometries.empty()) return COWPtr();

  COWPtr cow(new CollisionObjectWrapper(link.get()));

  if ((link->GetGeometries().size() == 1) && isIdentity(link->GetGeometry(0)->GetTransform())) {
    LOG_DEBUG("using identity for %s", link->GetName().c_str());
    btCollisionShape* shape = createShapePrimitive(link->GetGeometry(0), useTrimesh, cow.get());
    shape->setMargin(MARGIN);
    cow->manage(shape);
    cow->setCollisionShape(shape);

  }
  else {
    LOG_DEBUG("NOT using identity for %s", link->GetName().c_str());
    btCompoundShape* compound = new btCompoundShape(/*dynamicAABBtree=*/false);
    cow->manage(compound);
    compound->setMargin(MARGIN); //margin: compound. seems to have no effect when positive but has an effect when negative
    cow->setCollisionShape(compound);

    BOOST_FOREACH(const boost::shared_ptr<OpenRAVE::KinBody::Link::Geometry>& geom, geometries) {

      btCollisionShape* subshape = createShapePrimitive(geom, useTrimesh, cow.get());
      if (subshape != NULL) {
        cow->manage(subshape);
        subshape->setMargin(MARGIN);
        btTransform geomTrans = toBt(geom->GetTransform());
        compound->addChildShape(geomTrans, subshape);
      }
    }

  }

  cow->setWorldTransform(toBt(link->GetTransform()));

  return cow;
}



void RenderCollisionShape(btCollisionShape* shape, const btTransform& tf,
    OpenRAVE::EnvironmentBase& env, vector<OpenRAVE::GraphHandlePtr>& handles) {

  typedef map<btCollisionShape*, HullResult > Shape2Inds;
  Shape2Inds gHullCache;

  switch (shape->getShapeType()) {
  case COMPOUND_SHAPE_PROXYTYPE: {
    btCompoundShape* compound = static_cast<btCompoundShape*>(shape);
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      RenderCollisionShape(compound->getChildShape(i),
          tf * compound->getChildTransform(i), env, handles);
    }
    break;
  }
  case CONVEX_HULL_SHAPE_PROXYTYPE: {
    btConvexHullShape* convex = static_cast<btConvexHullShape*>(shape);

    Shape2Inds::iterator it = gHullCache.find(convex);

    btAlignedObjectArray<unsigned int> inds;
    HullResult hr;
    if ( it != gHullCache.end() )
      hr = it->second;
    else {

      HullDesc hd;
      hd.mFlags = QF_TRIANGLES;
      hd.mVcount = convex->getNumPoints();
      hd.mVertices = convex->getPoints();
      hd.mVertexStride = sizeof(btVector3);
      HullLibrary hl;

      if (hl.CreateConvexHull(hd, hr) == QE_FAIL) {
        LOG_ERROR("convex hull computation failed on shape with %i vertices", convex->getNumPoints());
        hr.mNumFaces = 0;
      }
      else {
      }
      gHullCache[convex] = hr;
    }

    if (hr.mNumFaces > 0) {
      vector<btVector3> tverts(hr.mNumOutputVertices);
      for (int i=0; i < tverts.size(); ++i) tverts[i] = tf * hr.m_OutputVertices[i];


      handles.push_back(env.drawtrimesh((float*)&tverts[0], 16,
          (int*) &hr.m_Indices[0], hr.mNumFaces, OR::RaveVector<float>(1,1,1,.1)));
    }
    break;


  }

  default:
    LOG_INFO("not rendering shape of type %i", shape->getShapeType());
    break;
  }
}


class BulletCollisionChecker : public CollisionChecker {
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  typedef map<const OR::KinBody::Link*, CollisionObjectWrapper*> Link2Cow;
  Link2Cow m_link2cow;
  double m_contactDistance;
  vector<KinBodyPtr> m_prevbodies;
  typedef std::pair<const KinBody::Link*, const KinBody::Link*> LinkPair;
  set< LinkPair > m_excludedPairs;
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_allowedCollisionMatrix;

public:
  BulletCollisionChecker(OR::EnvironmentBaseConstPtr env);
  ~BulletCollisionChecker();

  ///////// public interface /////////
  virtual void SetContactDistance(float distance);
  virtual double GetContactDistance() {return m_contactDistance;}
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles);
  virtual void ExcludeCollisionPair(const KinBody::Link& link0, const KinBody::Link& link1) {
    m_excludedPairs.insert(LinkPair(&link0, &link1));
    COW *cow0 = GetCow(&link0), *cow1 = GetCow(&link1);
    if (cow0 && cow1) m_allowedCollisionMatrix(cow0->m_index, cow1->m_index) = 0;
  }
  virtual void IncludeCollisionPair(const KinBody::Link& link0, const KinBody::Link& link1) {
    m_excludedPairs.erase(LinkPair(&link0, &link1));
    COW *cow0 = GetCow(&link0), *cow1 = GetCow(&link1);
    if (cow0 && cow1) m_allowedCollisionMatrix(cow0->m_index, cow1->m_index) = 1;
  }
  virtual bool RayCastCollision(const OpenRAVE::Vector& point1, const OpenRAVE::Vector& point2) {
    btVector3 btFrom = toBt(point1);
    btVector3 btTo = toBt(point2);
    btCollisionWorld::ClosestRayResultCallback res(btFrom, btTo);
    UpdateBulletFromRave();
    m_world->updateAabbs();
    m_world->rayTest(btFrom, btTo, res);
    return res.hasHit();
  }
  // collision checking
  virtual void AllVsAll(vector<Collision>& collisions);
  virtual void LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions, short filterMask);
  virtual void LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions, short filterMask);
  virtual void ContinuousCheckTrajectory(const TrajArray& traj, Configuration& rad, vector<Collision>&);
  virtual void CastVsAll(Configuration& rad, const vector<KinBody::LinkPtr>& links, const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions);
  ////
  ///////

  CollisionObjectWrapper* GetCow(const KinBody::Link* link) {
    Link2Cow::iterator it = m_link2cow.find(link);
    return (it == m_link2cow.end()) ? 0 : it->second;
  }
  void SetCow(const KinBody::Link* link, COW* cow) {m_link2cow[link] = cow;}
  void LinkVsAll_NoUpdate(const KinBody::Link& link, vector<Collision>& collisions, short filterMask);
  void UpdateBulletFromRave();
  void AddKinBody(const OR::KinBodyPtr& body);
  void RemoveKinBody(const OR::KinBodyPtr& body);
  void AddAndRemoveBodies(const vector<OR::KinBodyPtr>& curVec, const vector<OR::KinBodyPtr>& prevVec, vector<KinBodyPtr>& addedBodies);
  bool CanCollide(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) {
    return m_allowedCollisionMatrix(cow0->m_index, cow1->m_index);
  }
  void SetLinkIndices();
  void UpdateAllowedCollisionMatrix();
  void CheckShapeCast(btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1,
      CollisionObjectWrapper* cow, btCollisionWorld* world, vector<Collision>& collisions);


};

struct CollisionCollector : public btCollisionWorld::ContactResultCallback {
  std::vector<Collision>& m_collisions;
  const CollisionObjectWrapper* m_cow;
  BulletCollisionChecker* m_cc;

  CollisionCollector(vector<Collision>& collisions, CollisionObjectWrapper* cow, BulletCollisionChecker* cc) :
    m_collisions(collisions), m_cow(cow), m_cc(cc) {}
  virtual btScalar addSingleResult(btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
      const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) {
    if (cp.m_distance1 > m_cc->GetContactDistance()) return 0;
    const KinBody::Link* linkA = getLink(colObj0Wrap->getCollisionObject());
    const KinBody::Link* linkB = getLink(colObj1Wrap->getCollisionObject());
    m_collisions.push_back(Collision(linkA, linkB, toOR(cp.m_positionWorldOnA), toOR(cp.m_positionWorldOnB),
        toOR(cp.m_normalWorldOnB), cp.m_distance1));
    LOG_DEBUG("CollisionCollector: adding collision %s-%s (%.4f)", linkA->GetName().c_str(), linkB->GetName().c_str(), cp.m_distance1);
    return 1;
  }
  bool needsCollision(btBroadphaseProxy* proxy0) const {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask)
        && (m_collisionFilterGroup & proxy0->m_collisionFilterMask)
        && m_cc->CanCollide(m_cow, static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject));
  }
};


// only used for AllVsAll
void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {
  BulletCollisionChecker* cc = static_cast<BulletCollisionChecker*>(dispatcher.m_userData);
  if ( cc->CanCollide(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject),
                      static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)))
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}


BulletCollisionChecker::BulletCollisionChecker(OR::EnvironmentBaseConstPtr env) :
  CollisionChecker(env) {
  m_coll_config = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_coll_config);
  m_broadphase = new btDbvtBroadphase();
  m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
  m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
      m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
  m_dispatcher->setNearCallback(&nearCallback);
  m_dispatcher->m_userData = this;
  SetContactDistance(.05);
  UpdateBulletFromRave();
}

BulletCollisionChecker::~BulletCollisionChecker() {
  delete m_world;
  delete m_broadphase;
  delete m_dispatcher;
  delete m_coll_config;
}


void BulletCollisionChecker::SetContactDistance(float dist) {
  LOG_DEBUG("setting contact distance to %.2f", dist);
  m_contactDistance = dist;
  SHAPE_EXPANSION = btVector3(1,1,1)*dist;
  gContactBreakingThreshold = 2.001*dist; // wtf. when I set it to 2.0 there are no contacts with distance > 0
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  for (int i=0; i < objs.size(); ++i) {
    objs[i]->setContactProcessingThreshold(dist);
  }
  btCollisionDispatcher* dispatcher = static_cast<btCollisionDispatcher*>(m_world->getDispatcher());
  dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
}


void BulletCollisionChecker::AllVsAll(vector<Collision>& collisions) {
  LOG_WARN("WARNING: AllVsAll seems to be broken! (since a8f8da01)");
  UpdateBulletFromRave();
  LOG_DEBUG("AllVsAll");
  m_world->performDiscreteCollisionDetection();
  int numManifolds = m_dispatcher->getNumManifolds();
  LOG_DEBUG("number of manifolds: %i", numManifolds);
  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold = m_dispatcher->getManifoldByIndexInternal(i);
    int numContacts = contactManifold->getNumContacts();
    LOG_DEBUG("number of contacts in manifold %i: %i", i, numContacts);
    const CollisionObjectWrapper* objA = static_cast<const CollisionObjectWrapper*>(contactManifold->getBody0());
    const CollisionObjectWrapper* objB = static_cast<const CollisionObjectWrapper*>(contactManifold->getBody1());
    for (int j = 0; j < numContacts; ++j) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
//      stringstream ss; ss << pt.m_localPointA << " | " << pt.m_localPointB;
//      LOG_DEBUG("local pts: %s\n",ss.str().c_str());
      // adjustContactPoint(pt, objA, objB);

      const KinBody::Link* bodyA = objA->m_link;
      const KinBody::Link* bodyB = objB->m_link;

      if (CanCollide(objA, objB)) {
        collisions.push_back(Collision(bodyA, bodyB, toOR(pt.getPositionWorldOnA()), toOR(pt.getPositionWorldOnB()),
            toOR(pt.m_normalWorldOnB), pt.m_distance1, 1./numContacts));
      }
      else {
        LOG_DEBUG("ignoring collision between %s and %s", bodyA->GetName().c_str(), bodyB->GetName().c_str());
        assert(0 && "this shouldn't happen because we're filtering at narrowphase");
      }
      LOG_DEBUG("%s - %s collided", bodyA->GetName().c_str(), bodyB->GetName().c_str());
    }
    // caching helps performance, but for optimization the cost should not be history-dependent
    contactManifold->clearManifold();
  }
}

void BulletCollisionChecker::LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions, short filterMask) {
//  AllVsAll(collisions);
//  return;

  UpdateBulletFromRave();
  m_world->updateAabbs();
  
  for (int i=0; i < links.size(); ++i) {
    LinkVsAll_NoUpdate(*links[i], collisions, filterMask);
  }
}


void BulletCollisionChecker::LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions, short filterMask) {
  UpdateBulletFromRave();
  LinkVsAll_NoUpdate(link, collisions, filterMask);
}

void BulletCollisionChecker::LinkVsAll_NoUpdate(const KinBody::Link& link, vector<Collision>& collisions, short filterMask) {
  if (link.GetGeometries().empty()) return;
  CollisionObjectWrapper* cow = GetCow(&link);
  CollisionCollector cc(collisions, cow, this);
  cc.m_collisionFilterMask = filterMask;
  m_world->contactTest(cow, cc);
}

struct KinBodyCollisionData;
typedef boost::shared_ptr<KinBodyCollisionData> CDPtr;
struct KinBodyCollisionData : public OpenRAVE::UserData {
  OpenRAVE::KinBodyWeakPtr body;
  std::vector<KinBody::Link*> links;
  std::vector<COWPtr> cows;
  KinBodyCollisionData(OR::KinBodyPtr _body) : body(_body) {}
};

void BulletCollisionChecker::AddKinBody(const OR::KinBodyPtr& body) {
  CDPtr cd(new KinBodyCollisionData(body));

  int filterGroup = body->IsRobot() ? RobotFilter : KinBodyFilter;
  const vector<OR::KinBody::LinkPtr> links = body->GetLinks();

  trajopt::SetUserData(*body, "bt", cd);
  
  bool useTrimesh = trajopt::GetUserData(*body, "bt_use_trimesh");
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, links) {
    if (link->GetGeometries().size() > 0) {
      COWPtr new_cow = CollisionObjectFromLink(link, useTrimesh); 
      if (new_cow) {
        SetCow(link.get(), new_cow.get());
        m_world->addCollisionObject(new_cow.get(), filterGroup);
        new_cow->setContactProcessingThreshold(m_contactDistance);
        LOG_DEBUG("added collision object for  link %s", link->GetName().c_str());
        cd->links.push_back(link.get());
        cd->cows.push_back(new_cow);
      }
      else {
        LOG_WARN("ignoring link %s", link->GetName().c_str());
      }
    }
  }

}
void BulletCollisionChecker::RemoveKinBody(const OR::KinBodyPtr& body) {
  LOG_DEBUG("removing %s", body->GetName().c_str());
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, body->GetLinks()) {
    CollisionObjectWrapper* cow = GetCow(link.get());
    if (cow) {
      m_world->removeCollisionObject(cow);
      m_link2cow.erase(link.get());      
    }
  }
  trajopt::RemoveUserData(*body, "bt");
}

template <typename T>
void SetDifferences(const vector<T>& A, const vector<T>& B, vector<T>& AMinusB, vector<T>& BMinusA) {
  set<T> Aset, Bset;
  AMinusB.clear();
  BMinusA.clear();
  BOOST_FOREACH(const T& a, A) {
    Aset.insert(a);
  }
  BOOST_FOREACH(const T& b, B) {
    Bset.insert(b);
  }
  BOOST_FOREACH(const T& a, A) {
    if (Bset.count(a) == 0) AMinusB.push_back(a);
  }
  BOOST_FOREACH(const T& b, B) {
    if (Aset.count(b) == 0) BMinusA.push_back(b);
  }
}

void BulletCollisionChecker::AddAndRemoveBodies(const vector<KinBodyPtr>& curVec, const vector<KinBodyPtr>& prevVec, vector<KinBodyPtr>& toAdd) {
  vector<KinBodyPtr> toRemove;
  SetDifferences(curVec, prevVec, toAdd, toRemove);
  BOOST_FOREACH(const KinBodyPtr& body, toAdd) {
    assert(!trajopt::GetUserData(*body, "bt"));
    AddKinBody(body);
  }
  BOOST_FOREACH(const KinBodyPtr& body, toRemove) {
    RemoveKinBody(body);
  }
  SetLinkIndices();
}

void BulletCollisionChecker::SetLinkIndices() {
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  for (int i=0; i < objs.size(); ++i) {
    CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
    cow->m_index = i;
  }
  m_allowedCollisionMatrix.resize(objs.size(), objs.size());
  m_allowedCollisionMatrix.setOnes();
}

void BulletCollisionChecker::UpdateAllowedCollisionMatrix() {
  BOOST_FOREACH(const LinkPair& pair, m_excludedPairs) {
    const KinBody::Link* linkA = pair.first;
    const KinBody::Link* linkB = pair.second;
    const CollisionObjectWrapper* cowA = GetCow(linkA);
    const CollisionObjectWrapper* cowB = GetCow(linkB);
    if (cowA != NULL && cowB != NULL) {
      m_allowedCollisionMatrix(cowA->m_index, cowB->m_index) = 0;
      m_allowedCollisionMatrix(cowB->m_index, cowA->m_index) = 0;
    }
  }
}

void BulletCollisionChecker::UpdateBulletFromRave() {
  vector<OR::KinBodyPtr> bodies, addedBodies;
  m_env->GetBodies(bodies);
  if (bodies.size() != m_prevbodies.size() || !std::equal(bodies.begin(), bodies.end(), m_prevbodies.begin())) {
    LOG_DEBUG("need to add and remove stuff");
    AddAndRemoveBodies(bodies, m_prevbodies, addedBodies);
    m_prevbodies=bodies;
    float contactDistanceOld = GetContactDistance();
    SetContactDistance(.1 METERS);
    BOOST_FOREACH(const KinBodyPtr& body, addedBodies) {
      IgnoreZeroStateSelfCollisions(body);
    }
    SetContactDistance(contactDistanceOld);
    UpdateAllowedCollisionMatrix();
  }
  else {
    LOG_DEBUG("don't need to add or remove stuff");
  }

  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  LOG_DEBUG("%i objects in bullet world", objs.size());
  for (int i=0; i < objs.size(); ++i) {
    CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
    cow->setWorldTransform(toBt(cow->m_link->GetTransform()));
  }

}


void BulletCollisionChecker::PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles) {
  UpdateBulletFromRave();
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  LOG_DEBUG("%i objects in bullet world", objs.size());
  for (int i=0; i < objs.size(); ++i) {
    RenderCollisionShape(objs[i]->getCollisionShape(), objs[i]->getWorldTransform(), *boost::const_pointer_cast<OpenRAVE::EnvironmentBase>(m_env), handles);
  }
}




////////// Continuous collisions ////////////////////////

namespace {

vector<btTransform> rightMultiplyAll(const vector<btTransform>& xs, const btTransform& y) {
  vector<btTransform> out(xs.size());
  for (int i=0; i < xs.size(); ++i) out[i] = xs[i]*y;
  return out;
}


}

void ContinuousCheckShape(btCollisionShape* shape, const vector<btTransform>& transforms,
    KinBody::Link* link, btCollisionWorld* world, vector<Collision>& collisions) {
  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {
    for (int i=0; i < transforms.size()-1; ++i) {
      btCollisionWorld::ClosestConvexResultCallback ccc(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN));
      ccc.m_collisionFilterMask = KinBodyFilter;
      world->convexSweepTest(convex, transforms[i], transforms[i+1], ccc, 0);
      if (ccc.hasHit()) {
        collisions.push_back(Collision(link, getLink(ccc.m_hitCollisionObject),
            toOR(ccc.m_hitPointWorld), toOR(ccc.m_hitPointWorld), toOR(ccc.m_hitNormalWorld), 0, 1, i+ccc.m_closestHitFraction));
      }
    }
  }
  else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape)) {
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      ContinuousCheckShape(compound->getChildShape(i), rightMultiplyAll(transforms, compound->getChildTransform(i)),  link, world, collisions);
    }
  }
  else {
    throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
  }

}


void BulletCollisionChecker::ContinuousCheckTrajectory(const TrajArray& traj, Configuration& rad, vector<Collision>& collisions) {
  UpdateBulletFromRave();
  m_world->updateAabbs();

  // first calculate transforms of all the relevant links at each step
  vector<KinBody::LinkPtr> links;
  vector<int> link_inds;
  rad.GetAffectedLinks(links, true, link_inds);


  // don't need to remove them anymore because now I only check collisions
  // against KinBodyFilter stuff
  // remove them, because we can't check moving stuff against each other
  vector<CollisionObjectWrapper*> cows;
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    CollisionObjectWrapper* cow = GetCow(link.get());
    assert(cow != NULL);
    cows.push_back(cow);
#if 0
    m_world->removeCollisionObject(cow);
#endif
  }


  typedef vector<btTransform> TransformVec;
  vector<TransformVec> link2transforms(links.size(), TransformVec(traj.rows()));
  Configuration::SaverPtr save = rad.Save();

  for (int iStep=0; iStep < traj.rows(); ++iStep) {
    rad.SetDOFValues(toDblVec(traj.row(iStep)));
    for (int iLink = 0; iLink < links.size(); ++iLink) {
      link2transforms[iLink][iStep] = toBt(links[iLink]->GetTransform());
    }
  }

  for (int iLink = 0; iLink < links.size(); ++iLink) {
    ContinuousCheckShape(cows[iLink]->getCollisionShape(), link2transforms[iLink], links[iLink].get(), m_world, collisions);
  }

#if 0
  // add them back
  BOOST_FOREACH(CollisionObjectWrapper* cow, cows) {
    m_world->addCollisionObject(cow);
  }
#endif
}

#if 0
class CompoundHullShape : public btConvexShape {
  std::vector<btConvexHullShape*> m_children;
  btVector3   localGetSupportingVertex(const btVector3& vec)const {
    btVector3 sv = m_children[0]->localGetSupportingVertex(vec);
    float support = sv.dot(vec);
    for (int i=1; i < m_children.size(); ++i) {
      btVector3 newsv = m_children[i]->localGetSupportingVertex(vec);
      float newsupport = vec.dot(newsv);
      if (newsupport > support) {
        support = newsupport;
        sv = newsv;
      }
    }
  }
#if 0
  void project(const btTransform& trans, const btVector3& dir, btScalar& min, btScalar& max) const {
    m_children[0]->project(trans, dir, min, max);
    for (int i=1; i < m_children.size(); ++i) {
      btScalar newmin, newmax;
      m_children[i]->project(trans, dir, newmin, newmax);
      btSetMin(min, newmin);
      btSetMax(max, newmax);
    }
  }
#endif

  //notice that the vectors should be unit length
  void    batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const {
    throw std::runtime_error("not implemented");
  }

  ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    m_children[0]->getAabb(t, aabbMin, aabbMax);
    for (int i=1; i < m_children.size(); ++i) {
      btVector3 newmin, newmax;
      m_children[i]->getAabb(t, newmin, newmax);
      aabbMin.setMin(newmin);
      aabbMax.setMax(newmax);
    }
  }

  virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void    setLocalScaling(const btVector3& scaling) {}
  virtual const btVector3& getLocalScaling() const {return btVector3(1,1,1);}

  virtual void    setMargin(btScalar margin) {}
  virtual btScalar    getMargin() const {return 0;}

  virtual int     getNumPreferredPenetrationDirections() const {return 0;}
  virtual void    getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const=0;


};
#endif


struct CastHullShape : public btConvexShape {
public:
  btConvexShape* m_shape;
  btTransform m_t01, m_t10; // T_0_1 = T_w_0^-1 * T_w_1
  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_t01(t01) {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;



  }
  btVector3   localGetSupportingVertex(const btVector3& vec)const {
    btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
    btVector3 sv1 = m_t01*m_shape->localGetSupportingVertex(vec*m_t01.getBasis());
    return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
  }
#if 0
  void project(const btTransform& trans, const btVector3& dir, btScalar& min, btScalar& max) const {
    m_children[0]->project(trans, dir, min, max);
    for (int i=1; i < m_children.size(); ++i) {
      btScalar newmin, newmax;
      m_children[i]->project(trans, dir, newmin, newmax);
      btSetMin(min, newmin);
      btSetMax(max, newmax);
    }
  }
#endif

  //notice that the vectors should be unit length
  void    batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const {
    throw std::runtime_error("not implemented");
  }

  ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0,btVector3& aabbMin,btVector3& aabbMax) const {
    m_shape->getAabb(t_w0, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(t_w0*m_t01, min1, max1 );
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void    setLocalScaling(const btVector3& scaling) {}
  virtual const btVector3& getLocalScaling() const {
    static btVector3 out(1,1,1);
    return out;
  }

  virtual void    setMargin(btScalar margin) {}
  virtual btScalar    getMargin() const {return 0;}

  virtual int     getNumPreferredPenetrationDirections() const {return 0;}
  virtual void    getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const {throw std::runtime_error("not implemented");}


  virtual void calculateLocalInertia(btScalar, btVector3&) const {throw std::runtime_error("not implemented");}
  virtual const char* getName() const {return "CastHull";}
  virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const {return localGetSupportingVertex(v);}

  void calculateContactTime(Collision& col) {    
    // float support0 = localGetSupportingVertex(col.)
  }

};


struct CastCollisionCollector : public CollisionCollector {
  CastCollisionCollector(vector<Collision>& collisions, CollisionObjectWrapper* cow, BulletCollisionChecker* cc) :
    CollisionCollector(collisions, cow, cc) {}  
  virtual btScalar addSingleResult(btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
      const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1);
};


btScalar CastCollisionCollector::addSingleResult(btManifoldPoint& cp,
    const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
    const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) {      
      float retval = CollisionCollector::addSingleResult(cp, colObj0Wrap,partId0,index0, colObj1Wrap,partId1,index1); // call base class func
      if (retval == 1) { // if contact was added
        bool castShapeIsFirst =  (colObj0Wrap->getCollisionObject() == m_cow);
        btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
        const CastHullShape* shape = dynamic_cast<const CastHullShape*>((castShapeIsFirst ? colObj0Wrap : colObj1Wrap)->getCollisionObject()->getCollisionShape());
        assert(!!shape);
        btTransform tfWorld0 = m_cow->getWorldTransform();
        btTransform tfWorld1 = m_cow->getWorldTransform() * shape->m_t01;
        btVector3 normalLocal0 = normalWorldFromCast * tfWorld0.getBasis();
        btVector3 normalLocal1 = normalWorldFromCast * tfWorld1.getBasis();

        Collision& col = m_collisions.back();
        const float SUPPORT_FUNC_TOLERANCE = .01 METERS;

//        cout << normalWorldFromCast << endl;

        if (castShapeIsFirst) {
          swap(col.ptA, col.ptB);
          swap(col.linkA, col.linkB);
          col.normalB2A *= -1;
        }

#if 0
        btVector3 ptWorld0 = tfWorld0*shape->m_shape->localGetSupportingVertex(normalLocal0);
        btVector3 ptWorld1 = tfWorld1*shape->m_shape->localGetSupportingVertex(normalLocal1);
#else
        btVector3 ptLocal0;
        float localsup0;
        GetAverageSupport(shape->m_shape, normalLocal0, localsup0, ptLocal0);
        btVector3 ptWorld0 = tfWorld0 * ptLocal0;
        btVector3 ptLocal1;
        float localsup1;
        GetAverageSupport(shape->m_shape, normalLocal1, localsup1, ptLocal1);
        btVector3 ptWorld1 = tfWorld1 * ptLocal1;



#endif
        float sup0 = normalWorldFromCast.dot(ptWorld0);
        float sup1 = normalWorldFromCast.dot(ptWorld1);



        // TODO: this section is potentially problematic. think hard about the math
        if (sup0 - sup1 > SUPPORT_FUNC_TOLERANCE) {
          col.time = 0;
          col.cctype = CCType_Time0;
        }
        else if (sup1 - sup0 > SUPPORT_FUNC_TOLERANCE) {
          col.time = 1;
          col.cctype = CCType_Time1;
        }
        else {
          const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
          float l0c = (ptOnCast - ptWorld0).length(), 
                l1c = (ptOnCast - ptWorld1).length();

          col.ptB = toOR(ptWorld0);
          col.ptB1 = toOR(ptWorld1);
          col.cctype = CCType_Between;

          const float LENGTH_TOLERANCE = .001 METERS;

          if ( l0c + l1c < LENGTH_TOLERANCE) {

            col.time = .5;
          }
          else {
            col.time = l0c/(l0c + l1c); 
          }

        }
          
      }
      return retval;          
}

void BulletCollisionChecker::CheckShapeCast(btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1,
    CollisionObjectWrapper* cow, btCollisionWorld* world, vector<Collision>& collisions) {
  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {
    CastHullShape* shape = new CastHullShape(convex, tf0.inverseTimes(tf1));
    CollisionObjectWrapper* obj = new CollisionObjectWrapper(cow->m_link);
    obj->setCollisionShape(shape);
    obj->setWorldTransform(tf0);
    obj->m_index = cow->m_index;
    CastCollisionCollector cc(collisions, obj, this);
    cc.m_collisionFilterMask = KinBodyFilter;
    // cc.m_collisionFilterGroup = cow->m_collisionFilterGroup;
    world->contactTest(obj, cc);
    delete obj;
    delete shape;
  }
  else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape)) {
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      CheckShapeCast(compound->getChildShape(i), tf0*compound->getChildTransform(i), tf1*compound->getChildTransform(i), cow, world, collisions);
    }
  }
  else {
    throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
  }

}

void BulletCollisionChecker::CastVsAll(Configuration& rad, const vector<KinBody::LinkPtr>& links,
    const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) {
  Configuration::SaverPtr saver = rad.Save();
  rad.SetDOFValues(startjoints);
  int nlinks = links.size();
  vector<btTransform> tbefore(nlinks), tafter(nlinks);
  for (int i=0; i < nlinks; ++i) {
    tbefore[i] = toBt(links[i]->GetTransform());
  }
  rad.SetDOFValues(endjoints);
  for (int i=0; i < nlinks; ++i) {
    tafter[i] = toBt(links[i]->GetTransform());
  }
  rad.SetDOFValues(startjoints);
  UpdateBulletFromRave();
  m_world->updateAabbs();

  for (int i=0; i < nlinks; ++i) {
    assert(m_link2cow[links[i].get()] != NULL);
    CollisionObjectWrapper* cow = m_link2cow[links[i].get()];
    CheckShapeCast(cow->getCollisionShape(), tbefore[i], tafter[i], cow, m_world, collisions);
  }
  LOG_DEBUG("CastVsAll checked %li links and found %li collisions", links.size(), collisions.size());
}

}






namespace trajopt {



CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env) {
  CollisionCheckerPtr checker(new BulletCollisionChecker(env));
  return checker;
}
}
