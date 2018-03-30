#ifndef ROS_BULLET_UTILS_H
#define ROS_BULLET_UTILS_H

//#include "trajopt_scene/common.hpp"
#include "trajopt_scene/basic_env.h"
#include <geometric_shapes/mesh_operations.h>
#include <urdf_model/link.h>
#include <ros/console.h>
#include <btBulletCollisionCommon.h>

namespace trajopt_scene
{
#define METERS

const float BULLET_MARGIN = 0;
const float BULLET_SUPPORT_FUNC_TOLERANCE = .01 METERS;
const float BULLET_LENGTH_TOLERANCE = .001 METERS;
const float BULLET_EPSILON = 1e-3;
const double BULLET_DEFAULT_CONTACT_DISTANCE = 0.05;

inline
btVector3 convertEigenToBt(const Eigen::Vector3d& v)
{
  return btVector3(v[0], v[1], v[2]);
}

inline
Eigen::Vector3d convertBtToEigen(const btVector3& v)
{
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

inline
btQuaternion convertEigenToBt(const Eigen::Quaterniond& q)
{
  return btQuaternion(q.x(), q.y(), q.z(), q.w());
}

inline
btTransform convertEigenToBt(const Eigen::Affine3d& t)
{
  Eigen::Quaterniond q(t.rotation());
  return btTransform(convertEigenToBt(q), convertEigenToBt(t.translation()));
}

struct AttachedBodyInfo
{
  std::string name;
  std::string parent_link_name;
  std::string object_name;
  std::vector<std::string> touch_links;
};

struct AttachableObject
{
  std::string name;
  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Affine3d shapes_trans;
};
typedef boost::shared_ptr<AttachableObject> AttachableObjectPtr;
typedef boost::shared_ptr<const AttachableObject> AttachableObjectConstPtr;

struct AttachedBody
{
   AttachedBodyInfo info;
   AttachableObjectConstPtr obj;
};
typedef boost::shared_ptr<AttachedBody> AttachedBodyPtr;
typedef boost::shared_ptr<const AttachedBody> AttachedBodyConstPtr;

/// Destance query results information
typedef std::map<const std::pair<std::string, std::string>, DistanceResultVector> BulletDistanceMap;
struct BulletDistanceData
{
  BulletDistanceData(const DistanceRequest* req, BulletDistanceMap* res) : req(req), res(res), done(false)
  {
  }

  /// Distance query request information
  const DistanceRequest* req;

  /// Destance query results information
  BulletDistanceMap* res;

  /// Indicate if search is finished
  bool done;

};

class CollisionObjectWrapper : public btCollisionObject
{
public:
  CollisionObjectWrapper(const urdf::Link* link);

  CollisionObjectWrapper(const AttachedBody* ab);

  std::vector<boost::shared_ptr<void>> m_data;

  short int	m_collisionFilterGroup;
  short int	m_collisionFilterMask;

  int m_index; // index into collision matrix
  BodyType m_type;
  union
  {
    const urdf::Link* m_link;
    const AttachedBody* m_ab;
    const void* raw;
  } ptr;

  /**
   * @brief getID Returns the ID which is key when storing in link2cow
   * @return Collision object ID
   */
  const std::string& getID() const
  {
    if (m_type == BodyType::ROBOT_ATTACHED)
      return ptr.m_ab->info.name;
    else
      return ptr.m_link->name;
  }

  std::string getTypeString() const
  {
    if (m_type == BodyType::ROBOT_ATTACHED)
      return "Robot attached";
    else
      return "Robot link";
  }

  const std::string& getLinkName() const
  {
    if (m_type == BodyType::ROBOT_ATTACHED)
      return ptr.m_ab->info.parent_link_name;
    else
      return ptr.m_link->name;
  }

  boost::shared_ptr<CollisionObjectWrapper> clone()
  {
    if (m_type == BodyType::ROBOT_ATTACHED)
    {
      boost::shared_ptr<CollisionObjectWrapper> cow(new CollisionObjectWrapper(ptr.m_ab));
      cow->m_collisionFilterGroup = m_collisionFilterGroup;
      cow->m_collisionFilterMask = m_collisionFilterMask;
      return cow;
    }
    else
    {
      boost::shared_ptr<CollisionObjectWrapper> cow(new CollisionObjectWrapper(ptr.m_link));
      cow->m_collisionFilterGroup = m_collisionFilterGroup;
      cow->m_collisionFilterMask = m_collisionFilterMask;
      return cow;
    }
  }

  /** \brief Check if two CollisionObjectWrapper objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return m_type == other.m_type && ptr.raw == other.ptr.raw;
  }

  template<class T>
  void manage(T* t)
  { // manage memory of this object
    m_data.push_back(boost::shared_ptr<T>(t));
  }
  template<class T>
  void manage(boost::shared_ptr<T> t)
  {
    m_data.push_back(t);
  }

private:
  void initialize(const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &transforms);
};

typedef CollisionObjectWrapper COW;
typedef boost::shared_ptr<CollisionObjectWrapper> COWPtr;
typedef boost::shared_ptr<const CollisionObjectWrapper> COWConstPtr;
typedef std::map<std::string, COWPtr> Link2Cow;
typedef std::map<std::string, COWConstPtr> Link2ConstCow;

inline void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
    ROS_ERROR("error");
//  // only used for AllVsAll
//    BulletCollisionChecker* cc = static_cast<BulletCollisionChecker*>(dispatcher.m_userData);
//    if ( cc->CanCollide(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject),
//                        static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)))
//      dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
//  }
}

inline bool isCollisionAllowed(const COW* cow0, const COW* cow1, const AllowedCollisionMatrixConstPtr acm, bool verbose = false)
{
  // do not distance check geoms part of the same object / link / attached body
  if (cow0->sameObject(*cow1))
    return false;

  bool always_in_collision = false;
  if (acm != nullptr)
  {
    std::pair<std::string, std::string> pc = getObjectPairKey(cow0->getID(), cow1->getID());

    const auto& it = acm->find(pc);
    if (it != acm->end())
    {
      always_in_collision = true;
      if (verbose)
      {
        ROS_DEBUG("Collision between '%s' and '%s' is always allowed. No contacts are computed.", cow0->getID().c_str(), cow1->getID().c_str());
      }
    }
  }

  // check if a link is touching an attached object
  if (cow0->m_type == BodyType::ROBOT_LINK && cow1->m_type == BodyType::ROBOT_ATTACHED)
  {
    const std::vector<std::string>& tl = cow1->ptr.m_ab->info.touch_links;
    if (std::find(tl.begin(), tl.end(), cow0->getID()) != tl.end())
    {
      always_in_collision = true;
      if (verbose)
        ROS_DEBUG("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                  cow0->getID().c_str(), cow1->getID().c_str());
    }
  }
  else
  {
    if (cow1->m_type == BodyType::ROBOT_LINK && cow0->m_type == BodyType::ROBOT_ATTACHED)
    {
      const std::vector<std::string>& tl = cow0->ptr.m_ab->info.touch_links;
      if (std::find(tl.begin(), tl.end(), cow1->getID()) != tl.end())
      {
        always_in_collision = true;
        if (verbose)
          ROS_DEBUG("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                    cow1->getID().c_str(), cow0->getID().c_str());
      }
    }
  }

  if (verbose && !always_in_collision)
    ROS_DEBUG("Actually checking collisions between %s and %s", cow0->getID().c_str(), cow1->getID().c_str());

  return !always_in_collision;
}

inline
DistanceResult* processResult(BulletDistanceData& cdata, DistanceResult &contact, const std::pair<std::string, std::string>& key, bool found)
{
  if (!found)
  {
    DistanceResultVector data;
    data.reserve(100); // TODO: Need better way to initialize this
    data.emplace_back(contact);
    return &(cdata.res->insert(std::make_pair(key, data)).first->second.back());
  }
  else
  {
    DistanceResultVector &dr = cdata.res->at(key);
    if (cdata.req->type == DistanceRequestType::ALL)
    {
      dr.emplace_back(contact);
      return &(dr.back());
    }
    else if (cdata.req->type == DistanceRequestType::SINGLE)
    {
      if (contact.distance < dr[0].distance)
      {
        dr[0] = contact;
        return &(dr[0]);
      }
    }
//    else if (cdata.req->type == DistanceRequestType::LIMITED)
//    {
//      assert(dr.size() < cdata.req->max_contacts_per_body);
//      dr.emplace_back(contact);
//      return &(dr.back());
//    }
  }


  return nullptr;
}

struct CollisionCollector : public btCollisionWorld::ContactResultCallback
{
  BulletDistanceData& m_collisions;
  const COWPtr m_cow;
  double m_contact_distance;

  bool m_verbose;

  CollisionCollector(BulletDistanceData& collisions, const COWPtr cow, double contact_distance, bool verbose = false) :
    m_collisions(collisions), m_cow(cow), m_contact_distance(contact_distance), m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0, const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
  {
    if (cp.m_distance1 > m_contact_distance) return 0;

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

    std::pair<std::string, std::string> pc = getObjectPairKey(cd0->getID(), cd1->getID());

    const auto& it = m_collisions.res->find(pc);
    bool found = (it != m_collisions.res->end());

//    size_t l = 0;
//    if (found)
//    {
//      l = it->second.size();
//      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
//          return 0;

//    }

    DistanceResult contact;
    contact.link_names[0] = cd0->getID();
    contact.link_names[1] = cd1->getID();
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.body_types[0] = cd0->m_type;
    contact.body_types[1] = cd1->m_type;
    contact.distance = cp.m_distance1;
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

    if (!processResult(m_collisions, contact, pc, found))
    {
      return 0;
    }

    return 1;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask)
        && (m_collisionFilterGroup & proxy0->m_collisionFilterMask)
        && isCollisionAllowed(m_cow.get(), static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject), m_collisions.req->acm, m_verbose);
  }
};

struct SweepCollisionCollector : public btCollisionWorld::ClosestConvexResultCallback
{
  BulletDistanceData& m_collisions;
  const COWPtr m_cow;
  bool m_verbose;

  SweepCollisionCollector(BulletDistanceData& collisions, const COWPtr cow, bool verbose = false) :
    ClosestConvexResultCallback(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN)), m_collisions(collisions), m_cow(cow), m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
  {
    ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(m_cow.get());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(m_hitCollisionObject);

    std::pair<std::string, std::string> pc = getObjectPairKey(cd0->getID(), cd1->getID());

    BulletDistanceMap::iterator it = m_collisions.res->find(pc);
    bool found = it != m_collisions.res->end();

    size_t l = 0;
    if (found)
    {
      l = it->second.size();
//      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
//          return 0;
    }

    DistanceResult contact;
    contact.link_names[0] = cd0->getID();
    contact.link_names[1] = cd1->getID();
    contact.nearest_points[0] = convertBtToEigen(m_hitPointWorld);
    contact.nearest_points[1] = convertBtToEigen(m_hitPointWorld);
    contact.body_types[0] = cd0->m_type;
    contact.body_types[1] = cd1->m_type;
    contact.distance = 0;
    contact.normal = convertBtToEigen(-1 * m_hitNormalWorld);
    contact.cc_time = l + m_closestHitFraction;

    if (!processResult(m_collisions, contact, pc, found))
    {
      return 0;
    }

    return 1;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask)
        && (m_collisionFilterGroup & proxy0->m_collisionFilterMask)
        && isCollisionAllowed(m_cow.get(), static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject), m_collisions.req->acm, m_verbose);
  }
};



struct CastHullShape : public btConvexShape
{
public:
  btConvexShape* m_shape;
  btTransform m_t01, m_t10; // T_0_1 = T_w_0^-1 * T_w_1

  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_t01(t01)
  {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
  }

  btVector3 localGetSupportingVertex(const btVector3& vec) const override
  {
    btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
    btVector3 sv1 = m_t01*m_shape->localGetSupportingVertex(vec*m_t01.getBasis());
    return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
  }

  //notice that the vectors should be unit length
  void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const override
  {
    throw std::runtime_error("not implemented");
  }

  ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0,btVector3& aabbMin,btVector3& aabbMax) const override
  {
    m_shape->getAabb(t_w0, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(t_w0*m_t01, min1, max1 );
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const override
  {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void setLocalScaling(const btVector3& scaling) override {}

  virtual const btVector3& getLocalScaling() const override
  {
    static btVector3 out(1,1,1);
    return out;
  }

  virtual void setMargin(btScalar margin) override {}

  virtual btScalar getMargin() const override {return 0;}

  virtual int getNumPreferredPenetrationDirections() const override { return 0; }

  virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const override { throw std::runtime_error("not implemented"); }

  virtual void calculateLocalInertia(btScalar, btVector3&) const { throw std::runtime_error("not implemented"); }

  virtual const char* getName() const { return "CastHull"; }

  virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const {return localGetSupportingVertex(v);}

};


inline void GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport, btVector3& outpt)
{
  btVector3 ptSum(0,0,0);
  float ptCount = 0;
  float maxSupport=-1000;

  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape)
  {
    int nPts = pshape->getNumVertices();

    for (int i=0; i < nPts; ++i) {
      btVector3 pt;
      pshape->getVertex(i, pt);

      float sup  = pt.dot(localNormal);
      if (sup > maxSupport + BULLET_EPSILON)
      {
        ptCount=1;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup < maxSupport - BULLET_EPSILON)
      {
      }
      else
      {
        ptCount += 1;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
  }
  else
  {
    outpt = shape->localGetSupportingVertexWithoutMargin(localNormal);
    outsupport = localNormal.dot(outpt);
  }
}


struct CastCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  BulletDistanceData& m_collisions;
  const COWPtr m_cow;
  double m_contact_distance;

  bool m_verbose;

  CastCollisionCollector(BulletDistanceData& collisions, const COWPtr cow, double contact_distance, bool verbose = false) :
    m_collisions(collisions), m_cow(cow), m_contact_distance(contact_distance), m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0, const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
  {
    if (cp.m_distance1 > m_contact_distance) return 0;

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

    const std::pair<std::string, std::string>& pc = cd0->getID() < cd1->getID() ?
                                                        std::make_pair(cd0->getID(), cd1->getID()) :
                                                        std::make_pair(cd1->getID(), cd0->getID());

    BulletDistanceMap::iterator it = m_collisions.res->find(pc);
    bool found = it != m_collisions.res->end();

//    size_t l = 0;
//    if (found)
//    {
//      l = it->second.size();
//      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
//          return 0;
//    }

    DistanceResult contact;
    contact.link_names[0] = cd0->getID();
    contact.link_names[1] = cd1->getID();
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.body_types[0] = cd0->m_type;
    contact.body_types[1] = cd1->m_type;
    contact.distance = cp.m_distance1;
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

    DistanceResult* col = processResult(m_collisions, contact, pc, found);
    if (!col)
    {
      return 0;
    }

    bool castShapeIsFirst = (colObj0Wrap->getCollisionObject() == m_cow.get());
    btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
    const btCollisionObjectWrapper* firstColObjWrap = (castShapeIsFirst ? colObj0Wrap : colObj1Wrap);
    int shapeIndex = (castShapeIsFirst ? index0 : index1);

    if (castShapeIsFirst)
    {
      std::swap(col->nearest_points[0], col->nearest_points[1]);
      std::swap(col->link_names[0], col->link_names[1]);
      std::swap(col->body_types[0], col->body_types[1]);
      col->normal *= -1;
    }

    btTransform tfWorld0, tfWorld1;
    const CastHullShape* shape;
    if (btBroadphaseProxy::isConvex(firstColObjWrap->getCollisionObject()->getCollisionShape()->getShapeType()))
    {
      shape = static_cast<const CastHullShape*>(firstColObjWrap->getCollisionObject()->getCollisionShape());
      tfWorld0 = m_cow->getWorldTransform();
      tfWorld1 = m_cow->getWorldTransform() * shape->m_t01;
    }
    else if (btBroadphaseProxy::isCompound(firstColObjWrap->getCollisionObject()->getCollisionShape()->getShapeType()))
    {
      const btCompoundShape* compound = static_cast<const btCompoundShape*>(firstColObjWrap->getCollisionObject()->getCollisionShape());
      shape = static_cast<const CastHullShape*>(compound->getChildShape(shapeIndex));
      tfWorld0 = m_cow->getWorldTransform() *  compound->getChildTransform(shapeIndex);
      tfWorld1 = tfWorld0 * shape->m_t01;
    }
    else
    {
      throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
    }
    assert(!!shape);

    btVector3 normalLocal0 = normalWorldFromCast * tfWorld0.getBasis();
    btVector3 normalLocal1 = normalWorldFromCast * tfWorld1.getBasis();

    btVector3 ptLocal0;
    float localsup0;
    GetAverageSupport(shape->m_shape, normalLocal0, localsup0, ptLocal0);
    btVector3 ptWorld0 = tfWorld0 * ptLocal0;
    btVector3 ptLocal1;
    float localsup1;
    GetAverageSupport(shape->m_shape, normalLocal1, localsup1, ptLocal1);
    btVector3 ptWorld1 = tfWorld1 * ptLocal1;

    float sup0 = normalWorldFromCast.dot(ptWorld0);
    float sup1 = normalWorldFromCast.dot(ptWorld1);

    // TODO: this section is potentially problematic. think hard about the math
    if (sup0 - sup1 > BULLET_SUPPORT_FUNC_TOLERANCE)
    {
      col->cc_time = 0;
      col->cc_type = ContinouseCollisionType::CCType_Time0;
    }
    else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
    {
      col->cc_time = 1;
      col->cc_type = ContinouseCollisionType::CCType_Time1;
    }
    else
    {
      const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
      float l0c = (ptOnCast - ptWorld0).length(),
            l1c = (ptOnCast - ptWorld1).length();

      // DEBUG: SHould be removed
      Eigen::Vector3d diff = convertBtToEigen(ptWorld0) - col->nearest_points[1];
      double distDiff = diff.norm();

      // DEBUG: Store the original contact point for debug, remove after integration and testing and uncommnet: col->cc_nearest_points[0] = col->nearest_points[1];
      col->cc_nearest_points[0] = col->nearest_points[1];
      col->nearest_points[1] = convertBtToEigen(ptWorld0);
//        col->cc_nearest_points[0] = col->nearest_points[1];
      col->cc_nearest_points[1] = convertBtToEigen(ptWorld1);
      col->cc_type = ContinouseCollisionType::CCType_Between;

      if ( l0c + l1c < BULLET_LENGTH_TOLERANCE)
      {
        col->cc_time = .5;
      }
      else
      {
        col->cc_time = l0c/(l0c + l1c);
      }
    }

    return 1;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask)
        && (m_collisionFilterGroup & proxy0->m_collisionFilterMask)
        && isCollisionAllowed(m_cow.get(), static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject), m_collisions.req->acm, m_verbose);
  }
};

struct CastCollisionCollectorOriginal : public btCollisionWorld::ContactResultCallback
{
  BulletDistanceData& m_collisions;
  const COWPtr m_cow;
  double m_contact_distance;

  bool m_verbose;

  CastCollisionCollectorOriginal(BulletDistanceData& collisions, const COWPtr cow, double contact_distance, bool verbose = false) :
    m_collisions(collisions), m_cow(cow), m_contact_distance(contact_distance), m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0, const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
  {
    if (cp.m_distance1 > m_contact_distance) return 0;

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

    const std::pair<std::string, std::string>& pc = cd0->getID() < cd1->getID() ?
                                                        std::make_pair(cd0->getID(), cd1->getID()) :
                                                        std::make_pair(cd1->getID(), cd0->getID());

    BulletDistanceMap::iterator it = m_collisions.res->find(pc);
    bool found = it != m_collisions.res->end();

//    size_t l = 0;
//    if (found)
//    {
//      l = it->second.size();
//      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
//          return 0;
//    }

    DistanceResult contact;
    contact.link_names[0] = cd0->getID();
    contact.link_names[1] = cd1->getID();
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.body_types[0] = cd0->m_type;
    contact.body_types[1] = cd1->m_type;
    contact.distance = cp.m_distance1;
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

    DistanceResult* col = processResult(m_collisions, contact, pc, found);
    if (!col)
    {
      return 0;
    }

    bool castShapeIsFirst = (colObj0Wrap->getCollisionObject() == m_cow.get());
    btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
    const CastHullShape* shape = dynamic_cast<const CastHullShape*>((castShapeIsFirst ? colObj0Wrap : colObj1Wrap)->getCollisionObject()->getCollisionShape());
    assert(!!shape);
    btTransform tfWorld0 = m_cow->getWorldTransform();
    btTransform tfWorld1 = m_cow->getWorldTransform() * shape->m_t01;
    btVector3 normalLocal0 = normalWorldFromCast * tfWorld0.getBasis();
    btVector3 normalLocal1 = normalWorldFromCast * tfWorld1.getBasis();

    if (castShapeIsFirst)
    {
      std::swap(col->nearest_points[0], col->nearest_points[1]);
      std::swap(col->link_names[0], col->link_names[1]);
      std::swap(col->body_types[0], col->body_types[1]);
      col->normal *= -1;
    }

    btVector3 ptLocal0;
    float localsup0;
    GetAverageSupport(shape->m_shape, normalLocal0, localsup0, ptLocal0);
    btVector3 ptWorld0 = tfWorld0 * ptLocal0;
    btVector3 ptLocal1;
    float localsup1;
    GetAverageSupport(shape->m_shape, normalLocal1, localsup1, ptLocal1);
    btVector3 ptWorld1 = tfWorld1 * ptLocal1;

    float sup0 = normalWorldFromCast.dot(ptWorld0);
    float sup1 = normalWorldFromCast.dot(ptWorld1);

    // TODO: this section is potentially problematic. think hard about the math
    if (sup0 - sup1 > BULLET_SUPPORT_FUNC_TOLERANCE)
    {
      col->cc_time = 0;
      col->cc_type = ContinouseCollisionType::CCType_Time0;
    }
    else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
    {
      col->cc_time = 1;
      col->cc_type = ContinouseCollisionType::CCType_Time1;
    }
    else
    {
      const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
      float l0c = (ptOnCast - ptWorld0).length(),
            l1c = (ptOnCast - ptWorld1).length();

      col->nearest_points[1] = convertBtToEigen(ptWorld0);
      col->cc_nearest_points[0] = col->nearest_points[1];
      col->cc_nearest_points[1] = convertBtToEigen(ptWorld1);
      col->cc_type = ContinouseCollisionType::CCType_Between;

      if ( l0c + l1c < BULLET_LENGTH_TOLERANCE)
      {
        col->cc_time = .5;
      }
      else
      {
        col->cc_time = l0c/(l0c + l1c);
      }

    }

    return 1;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask)
        && (m_collisionFilterGroup & proxy0->m_collisionFilterMask)
        && isCollisionAllowed(m_cow.get(), static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject), m_collisions.req->acm, m_verbose);
  }
};

struct BulletManager
{
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  Link2Cow m_link2cow;

  BulletManager()
  {
    m_coll_config = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_coll_config);
    m_broadphase = new btDbvtBroadphase();
    m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
    m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
    m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
    m_dispatcher->setNearCallback(&nearCallback);

    btCollisionDispatcher* dispatcher = static_cast<btCollisionDispatcher*>(m_world->getDispatcher());
    dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
  }

  ~BulletManager()
  {
    delete m_world;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_coll_config;
  }

  void processCollisionObjects()
  {
    for (auto element : m_link2cow)
    {
      m_world->addCollisionObject(element.second.get(), element.second->m_collisionFilterGroup, element.second->m_collisionFilterMask);
    }
  }

  void contactDiscreteTest(const COWPtr cow, BulletDistanceData& collisions)
  {
    CollisionCollector cc(collisions, cow, cow->getContactProcessingThreshold());
    m_world->contactTest(cow.get(), cc);
  }

  void contactCastTest(const COWPtr cow, BulletDistanceData& collisions)
  {
    CastCollisionCollector cc(collisions, cow, cow->getContactProcessingThreshold());
    m_world->contactTest(cow.get(), cc);
  }

  void contactCastTestOriginal(const std::string &link_name, const btTransform &tf1, const btTransform &tf2, BulletDistanceData& collisions)
  {
    COWPtr cow = m_link2cow[link_name];
    convexCastTestHelper(cow, cow->getCollisionShape(), tf1, tf2, collisions);
  }

  void convexSweepTest(const COWPtr cow, const btTransform &tf1, const btTransform &tf2, BulletDistanceData& collisions)
  {
    SweepCollisionCollector cc(collisions, cow);
    convexSweepTestHelper(cow->getCollisionShape(), tf1, tf2, cc);
  }

private:

  void convexCastTestHelper(COWPtr cow, btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1, BulletDistanceData& collisions)
  {
    if (btBroadphaseProxy::isConvex(shape->getShapeType()))
    {
      btConvexShape* convex = dynamic_cast<btConvexShape*>(shape);

      CastHullShape* shape = new CastHullShape(convex, tf0.inverseTimes(tf1));
      COWPtr obj = cow->clone();
      obj->setCollisionShape(shape);
      obj->setWorldTransform(tf0);

      CastCollisionCollectorOriginal cc(collisions, obj, cow->getContactProcessingThreshold());
      m_world->contactTest(obj.get(), cc);

      delete shape;
    }
    else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape))
    {
      for (int i = 0; i < compound->getNumChildShapes(); ++i)
      {
        convexCastTestHelper(cow, compound->getChildShape(i), tf0*compound->getChildTransform(i), tf1*compound->getChildTransform(i), collisions);
      }
    }
    else {
      throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
    }
  }

  void convexSweepTestHelper(const btCollisionShape* shape, const btTransform &tf1, const btTransform &tf2, btCollisionWorld::ConvexResultCallback &cc)
  {
    if (btBroadphaseProxy::isConvex(shape->getShapeType()))
    {
      const btConvexShape* convex = static_cast<const btConvexShape*>(shape);
      m_world->convexSweepTest(convex, tf1, tf2, cc, 0);
    }
    else if (btBroadphaseProxy::isCompound(shape->getShapeType()))
    {
      const btCompoundShape* compound = static_cast<const btCompoundShape*>(shape);
      for (int i = 0; i < compound->getNumChildShapes(); ++i)
      {
        convexSweepTestHelper(compound->getChildShape(i), tf1 * compound->getChildTransform(i), tf1 * compound->getChildTransform(i), cc);
      }
    }
    else
    {
      throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
    }
  }
};
typedef boost::shared_ptr<BulletManager> BulletManagerPtr;

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom, bool useTrimesh, CollisionObjectWrapper* cow);
COWPtr CollisionObjectFromLink(const urdf::Link* link, bool useTrimesh);

inline
void setContactDistance(COWPtr cow, double contact_distance)
{
  SHAPE_EXPANSION = btVector3(1,1,1) * contact_distance;
  gContactBreakingThreshold = 2.001 * contact_distance; // wtf. when I set it to 2.0 there are no contacts with distance > 0
  cow->setContactProcessingThreshold(contact_distance);
}

inline
shapes::ShapePtr constructShape(const urdf::Geometry* geom)
{
  shapes::Shape* result = NULL;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
      result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
    case urdf::Geometry::CYLINDER:
      result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                    static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        result = m;
      }
    }
    break;
    default:
      ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
      break;
  }

  return shapes::ShapePtr(result);
}

inline Eigen::Affine3d urdfPose2Affine3d(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Affine3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q.toRotationMatrix());
  return af;
}

}
#endif // ROS_BULLET_UTILS_H
