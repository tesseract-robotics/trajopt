/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_COMMON_
#define MOVEIT_COLLISION_DETECTION_BULLET_COLLISION_COMMON_

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/macros/class_forward.h>
#include <memory>
#include <set>
#include <moveit/macros/deprecation.h>
#include <btBulletCollisionCommon.h>

namespace collision_detection
{
#define METERS

const float BULLET_MARGIN = 0;
const float BULLET_SUPPORT_FUNC_TOLERANCE = .01 METERS;
const float BULLET_LENGTH_TOLERANCE = .001 METERS;
const float BULLET_EPSILON = 1e-3;
const double BULLET_DEFAULT_CONTACT_DISTANCE = 0.05;

inline btVector3 convertEigenToBt(const Eigen::Vector3d& v) { return btVector3(v[0], v[1], v[2]); }
inline Eigen::Vector3d convertBtToEigen(const btVector3& v) { return Eigen::Vector3d(v.x(), v.y(), v.z()); }
inline btQuaternion convertEigenToBt(const Eigen::Quaterniond& q) { return btQuaternion(q.x(), q.y(), q.z(), q.w()); }
inline btTransform convertEigenToBt(const Eigen::Isometry3d& t)
{
  Eigen::Quaterniond q(t.rotation());
  return btTransform(convertEigenToBt(q), convertEigenToBt(t.translation()));
}

struct BulletDistanceData
{
  BulletDistanceData(const DistanceRequest* req, DistanceResult* res) : req(req), res(res), done(false) {}
  /// Distance query request information
  const DistanceRequest* req;

  /// Destance query results information
  DistanceResult* res;

  /// Indicate if search is finished
  bool done;
};

inline void convertBulletCollisions(collision_detection::CollisionResult& moveit_cr,
                                    const BulletDistanceData& collisions)
{
  if (collisions.req->type == DistanceRequestType::GLOBAL)
  {
    if (collisions.res->collision)
    {
      DistanceResultsData& collision = collisions.res->minimum_distance;
      moveit_cr.collision = true;
      collision_detection::Contact c;

      c.depth = collision.distance;
      c.normal = collision.normal;
      c.body_name_1 = collision.link_names[0];
      c.body_name_2 = collision.link_names[1];
      c.body_type_1 = collision.body_types[0];
      c.body_type_2 = collision.body_types[1];

      const std::pair<std::string, std::string>& key =
          collision.link_names[0] < collision.link_names[1] ?
              std::make_pair(collision.link_names[0], collision.link_names[1]) :
              std::make_pair(collision.link_names[1], collision.link_names[0]);

      moveit_cr.contacts[key].push_back(c);
      moveit_cr.contact_count++;
    }
  }
  else
  {
    for (auto it = collisions.res->distances.begin(); it != collisions.res->distances.end(); ++it)
    {
      for (auto collision : it->second)
      {
        if (collision.distance <= 0.0)
        {
          moveit_cr.collision = true;
          collision_detection::Contact c;

          c.depth = collision.distance;
          c.normal = collision.normal;
          c.body_name_1 = collision.link_names[0];
          c.body_name_2 = collision.link_names[1];
          c.body_type_1 = collision.body_types[0];
          c.body_type_2 = collision.body_types[1];

          moveit_cr.contacts[it->first].push_back(c);
          moveit_cr.contact_count++;
        }
      }
    }
  }
}

class CollisionObjectWrapper : public btCollisionObject
{
public:
  CollisionObjectWrapper(const robot_model::LinkModel* link);

  CollisionObjectWrapper(const robot_state::AttachedBody* ab);

  CollisionObjectWrapper(const World::Object* obj);

  std::vector<std::shared_ptr<void>> m_data;

  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;

  int m_index;  // index into collision matrix
  BodyType m_type;
  union
  {
    const robot_model::LinkModel* m_link;
    const robot_state::AttachedBody* m_ab;
    const World::Object* m_obj;
    const void* raw;
  } ptr;

  /**
   * @brief getID Returns the ID which is key when storing in link2cow
   * @return Collision object ID
   */
  const std::string& getID() const
  {
    switch (m_type)
    {
      case BodyTypes::ROBOT_LINK:
        return ptr.m_link->getName();
      case BodyTypes::ROBOT_ATTACHED:
        return ptr.m_ab->getName();
      default:
        break;
    }
    return ptr.m_obj->id_;
  }

  std::string getTypeString() const
  {
    switch (m_type)
    {
      case BodyTypes::ROBOT_LINK:
        return "Robot link";
      case BodyTypes::ROBOT_ATTACHED:
        return "Robot attached";
      default:
        break;
    }
    return "Object";
  }

  const std::string& getLinkName() const
  {
    switch (m_type)
    {
      case BodyTypes::ROBOT_LINK:
        return ptr.m_link->getName();
      case BodyTypes::ROBOT_ATTACHED:
        return ptr.m_ab->getAttachedLinkName();
      default:
        break;
    }
    return ptr.m_obj->id_;
  }

  std::shared_ptr<CollisionObjectWrapper> clone()
  {
    switch (m_type)
    {
      case BodyTypes::ROBOT_LINK:
      {
        std::shared_ptr<CollisionObjectWrapper> cow(new CollisionObjectWrapper(ptr.m_link));
        cow->m_collisionFilterGroup = m_collisionFilterGroup;
        cow->m_collisionFilterMask = m_collisionFilterMask;
        return cow;
      }
      case BodyTypes::ROBOT_ATTACHED:
      {
        std::shared_ptr<CollisionObjectWrapper> cow(new CollisionObjectWrapper(ptr.m_ab));
        cow->m_collisionFilterGroup = m_collisionFilterGroup;
        cow->m_collisionFilterMask = m_collisionFilterMask;
        return cow;
      }
      default:
      {
        std::shared_ptr<CollisionObjectWrapper> cow(new CollisionObjectWrapper(ptr.m_obj));
        cow->m_collisionFilterGroup = m_collisionFilterGroup;
        cow->m_collisionFilterMask = m_collisionFilterMask;
        return cow;
      }
    }
  }

  /** \brief Check if two CollisionObjectWrapper objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return m_type == other.m_type && ptr.raw == other.ptr.raw;
  }

  template <class T>
  void manage(T* t)
  {  // manage memory of this object
    m_data.push_back(std::shared_ptr<T>(t));
  }
  template <class T>
  void manage(std::shared_ptr<T> t)
  {
    m_data.push_back(t);
  }

private:
  void initialize(const std::vector<shapes::ShapeConstPtr>& shapes, const tesseract::VectorIsometry3d& transforms);
};

typedef CollisionObjectWrapper COW;
typedef std::shared_ptr<CollisionObjectWrapper> COWPtr;
typedef std::shared_ptr<const CollisionObjectWrapper> COWConstPtr;
typedef std::map<std::string, COWPtr> Link2Cow;
typedef std::map<std::string, COWConstPtr> Link2ConstCow;

inline void nearCallback(btBroadphasePair& collisionPair,
                         btCollisionDispatcher& dispatcher,
                         const btDispatcherInfo& dispatchInfo)
{
  logError("error");
  //  // only used for AllVsAll
  //    BulletCollisionChecker* cc = static_cast<BulletCollisionChecker*>(dispatcher.m_userData);
  //    if ( cc->CanCollide(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject),
  //                        static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)))
  //      dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
  //  }
}

inline bool
isCollisionAllowed(const COW* cow0, const COW* cow1, const AllowedCollisionMatrix* acm, bool verbose = false)
{
  // do not distance check geoms part of the same object / link / attached body
  if (cow0->sameObject(*cow1))
    return false;

  // use the collision matrix (if any) to avoid certain collision checks
  DecideContactFn dcf;
  bool always_in_collision = false;
  if (acm)
  {
    AllowedCollision::Type type;
    bool found = acm->getAllowedCollision(cow0->getID(), cow1->getID(), type);
    if (found)
    {
      // if we have an entry in the collision matrix, we read it
      if (type == AllowedCollision::ALWAYS)
      {
        always_in_collision = true;
        if (verbose)
        {
          CONSOLE_BRIDGE_logDebug("Collision between '%s' and '%s' is always allowed. No contacts are computed.",
                                  cow0->getID().c_str(),
                                  cow1->getID().c_str());
        }
      }
      else if (type == AllowedCollision::CONDITIONAL)
      {
        acm->getAllowedCollision(cow0->getID(), cow1->getID(), dcf);
        if (verbose)
          CONSOLE_BRIDGE_logDebug(
              "Collision between '%s' and '%s' is conditionally allowed", cow0->getID().c_str(), cow1->getID().c_str());
      }
    }
  }

  // check if a link is touching an attached object
  if (cow0->m_type == BodyTypes::ROBOT_LINK && cow1->m_type == BodyTypes::ROBOT_ATTACHED)
  {
    const std::set<std::string>& tl = cow1->ptr.m_ab->getTouchLinks();
    if (tl.find(cow0->getID()) != tl.end())
    {
      always_in_collision = true;
      if (verbose)
        logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                 cow0->getID().c_str(),
                 cow1->getID().c_str());
    }
  }
  else
  {
    if (cow1->m_type == BodyTypes::ROBOT_LINK && cow0->m_type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string>& tl = cow0->ptr.m_ab->getTouchLinks();
      if (tl.find(cow1->getID()) != tl.end())
      {
        always_in_collision = true;
        if (verbose)
          logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                   cow1->getID().c_str(),
                   cow0->getID().c_str());
      }
    }
  }

  if (verbose && !always_in_collision)
    logDebug("Actually checking collisions between %s and %s", cow0->getID().c_str(), cow1->getID().c_str());

  return !always_in_collision;
}

inline collision_detection::DistanceResultsData* processResult(BulletDistanceData& cdata,
                                                               collision_detection::DistanceResultsData& contact,
                                                               const std::pair<std::string, std::string>& key,
                                                               bool found)
{
  if (cdata.req->type == DistanceRequestType::GLOBAL)
  {
    if (contact.distance <= 0 && !cdata.res->collision)
    {
      cdata.res->collision = true;
      if (!cdata.req->enable_signed_distance)
        cdata.done = true;
    }

    if (contact.distance < cdata.res->minimum_distance.distance)
    {
      cdata.res->minimum_distance = contact;
      return &(cdata.res->minimum_distance);
    }
  }
  else
  {
    if (contact.distance <= 0 && !cdata.res->collision)
    {
      cdata.res->collision = true;
    }

    if (contact.distance < cdata.res->minimum_distance.distance)
    {
      cdata.res->minimum_distance = contact;
    }

    if (!found)
    {
      std::vector<DistanceResultsData> data;
      data.reserve(cdata.req->max_contacts_per_body);
      data.emplace_back(contact);
      return &(cdata.res->distances.insert(std::make_pair(key, data)).first->second.back());
    }
    else
    {
      std::vector<DistanceResultsData>& dr = cdata.res->distances[key];
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
      else if (cdata.req->type == DistanceRequestType::LIMITED)
      {
        assert(dr.size() < cdata.req->max_contacts_per_body);
        dr.emplace_back(contact);
        return &(dr.back());
      }
    }
  }

  return nullptr;
}

struct CollisionCollector : public btCollisionWorld::ContactResultCallback
{
  BulletDistanceData& m_collisions;
  const COWPtr m_cow;
  double m_contact_distance;

  bool m_verbose;

  CollisionCollector(BulletDistanceData& collisions, const COWPtr cow, double contact_distance, bool verbose = false)
    : m_collisions(collisions), m_cow(cow), m_contact_distance(contact_distance), m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int partId0,
                                   int index0,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int partId1,
                                   int index1)
  {
    if (cp.m_distance1 > m_contact_distance)
      return 0;

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

    const std::pair<std::string, std::string>& pc = cd0->getID() < cd1->getID() ?
                                                        std::make_pair(cd0->getID(), cd1->getID()) :
                                                        std::make_pair(cd1->getID(), cd0->getID());

    DistanceMap::iterator it = m_collisions.res->distances.find(pc);
    size_t l = 0;
    bool found = (it != m_collisions.res->distances.end());

    if (found)
    {
      l = it->second.size();
      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
        return 0;
    }

    DistanceResultsData contact;
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
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask) &&
           (m_collisionFilterGroup & proxy0->m_collisionFilterMask) &&
           isCollisionAllowed(m_cow.get(),
                              static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject),
                              m_collisions.req->acm,
                              m_verbose);
  }
};

struct SweepCollisionCollector : public btCollisionWorld::ClosestConvexResultCallback
{
  BulletDistanceData& m_collisions;
  const COWPtr m_cow;
  bool m_verbose;

  SweepCollisionCollector(BulletDistanceData& collisions, const COWPtr cow, bool verbose = false)
    : ClosestConvexResultCallback(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN))
    , m_collisions(collisions)
    , m_cow(cow)
    , m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
  {
    ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(m_cow.get());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(m_hitCollisionObject);

    const std::pair<std::string, std::string>& pc = cd0->getID() < cd1->getID() ?
                                                        std::make_pair(cd0->getID(), cd1->getID()) :
                                                        std::make_pair(cd1->getID(), cd0->getID());

    DistanceMap::iterator it = m_collisions.res->distances.find(pc);
    size_t l = 0;
    bool found = it != m_collisions.res->distances.end();
    if (found)
    {
      l = it->second.size();
      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
        return 0;
    }

    collision_detection::DistanceResultsData contact;
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
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask) &&
           (m_collisionFilterGroup & proxy0->m_collisionFilterMask) &&
           isCollisionAllowed(m_cow.get(),
                              static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject),
                              m_collisions.req->acm,
                              m_verbose);
  }
};

struct CastHullShape : public btConvexShape
{
public:
  btConvexShape* m_shape;
  btTransform m_t01, m_t10;  // T_0_1 = T_w_0^-1 * T_w_1

  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_t01(t01)
  {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
  }

  btVector3 localGetSupportingVertex(const btVector3& vec) const override
  {
    btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
    btVector3 sv1 = m_t01 * m_shape->localGetSupportingVertex(vec * m_t01.getBasis());
    return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
  }

  // notice that the vectors should be unit length
  void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,
                                                         btVector3* supportVerticesOut,
                                                         int numVectors) const override
  {
    throw std::runtime_error("not implemented");
  }

  /// getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0, btVector3& aabbMin, btVector3& aabbMax) const override
  {
    m_shape->getAabb(t_w0, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(t_w0 * m_t01, min1, max1);
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  virtual void getAabbSlow(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const override
  {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void setLocalScaling(const btVector3& scaling) override {}
  virtual const btVector3& getLocalScaling() const override
  {
    static btVector3 out(1, 1, 1);
    return out;
  }

  virtual void setMargin(btScalar margin) override {}
  virtual btScalar getMargin() const override { return 0; }
  virtual int getNumPreferredPenetrationDirections() const override { return 0; }
  virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const override
  {
    throw std::runtime_error("not implemented");
  }

  virtual void calculateLocalInertia(btScalar, btVector3&) const { throw std::runtime_error("not implemented"); }
  virtual const char* getName() const { return "CastHull"; }
  virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const
  {
    return localGetSupportingVertex(v);
  }
};

inline void
GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport, btVector3& outpt)
{
  btVector3 ptSum(0, 0, 0);
  float ptCount = 0;
  float maxSupport = -1000;

  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape)
  {
    int nPts = pshape->getNumVertices();

    for (int i = 0; i < nPts; ++i)
    {
      btVector3 pt;
      pshape->getVertex(i, pt);

      float sup = pt.dot(localNormal);
      if (sup > maxSupport + BULLET_EPSILON)
      {
        ptCount = 1;
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

  CastCollisionCollector(BulletDistanceData& collisions,
                         const COWPtr cow,
                         double contact_distance,
                         bool verbose = false)
    : m_collisions(collisions), m_cow(cow), m_contact_distance(contact_distance), m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int partId0,
                                   int index0,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int partId1,
                                   int index1)
  {
    if (cp.m_distance1 > m_contact_distance)
      return 0;

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

    const std::pair<std::string, std::string>& pc = cd0->getID() < cd1->getID() ?
                                                        std::make_pair(cd0->getID(), cd1->getID()) :
                                                        std::make_pair(cd1->getID(), cd0->getID());

    DistanceMap::iterator it = m_collisions.res->distances.find(pc);
    size_t l = 0;
    bool found = it != m_collisions.res->distances.end();
    if (found)
    {
      l = it->second.size();
      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
        return 0;
    }

    collision_detection::DistanceResultsData contact;
    contact.link_names[0] = cd0->getID();
    contact.link_names[1] = cd1->getID();
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.body_types[0] = cd0->m_type;
    contact.body_types[1] = cd1->m_type;
    contact.distance = cp.m_distance1;
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

    collision_detection::DistanceResultsData* col = processResult(m_collisions, contact, pc, found);
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
      const btCompoundShape* compound =
          static_cast<const btCompoundShape*>(firstColObjWrap->getCollisionObject()->getCollisionShape());
      shape = static_cast<const CastHullShape*>(compound->getChildShape(shapeIndex));
      tfWorld0 = m_cow->getWorldTransform() * compound->getChildTransform(shapeIndex);
      tfWorld1 = tfWorld0 * shape->m_t01;
    }
    else
    {
      throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex "
                               "shapes");
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
      col->cc_type = CCType_Time0;
    }
    else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
    {
      col->cc_time = 1;
      col->cc_type = CCType_Time1;
    }
    else
    {
      const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
      float l0c = (ptOnCast - ptWorld0).length(), l1c = (ptOnCast - ptWorld1).length();

      // DEBUG: SHould be removed
      Eigen::Vector3d diff = convertBtToEigen(ptWorld0) - col->nearest_points[1];
      double distDiff = diff.norm();

      // DEBUG: Store the original contact point for debug, remove after integration and testing and uncommnet:
      // col->cc_nearest_points[0] = col->nearest_points[1];
      col->cc_nearest_points[0] = col->nearest_points[1];
      col->nearest_points[1] = convertBtToEigen(ptWorld0);
      //        col->cc_nearest_points[0] = col->nearest_points[1];
      col->cc_nearest_points[1] = convertBtToEigen(ptWorld1);
      col->cc_type = CCType_Between;

      if (l0c + l1c < BULLET_LENGTH_TOLERANCE)
      {
        col->cc_time = .5;
      }
      else
      {
        col->cc_time = l0c / (l0c + l1c);
      }
    }

    return 1;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask) &&
           (m_collisionFilterGroup & proxy0->m_collisionFilterMask) &&
           isCollisionAllowed(m_cow.get(),
                              static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject),
                              m_collisions.req->acm,
                              m_verbose);
  }
};

struct CastCollisionCollectorOriginal : public btCollisionWorld::ContactResultCallback
{
  BulletDistanceData& m_collisions;
  const COWPtr m_cow;
  double m_contact_distance;

  bool m_verbose;

  CastCollisionCollectorOriginal(BulletDistanceData& collisions,
                                 const COWPtr cow,
                                 double contact_distance,
                                 bool verbose = false)
    : m_collisions(collisions), m_cow(cow), m_contact_distance(contact_distance), m_verbose(verbose)
  {
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int partId0,
                                   int index0,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int partId1,
                                   int index1)
  {
    if (cp.m_distance1 > m_contact_distance)
      return 0;

    const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
    const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

    const std::pair<std::string, std::string>& pc = cd0->getID() < cd1->getID() ?
                                                        std::make_pair(cd0->getID(), cd1->getID()) :
                                                        std::make_pair(cd1->getID(), cd0->getID());

    DistanceMap::iterator it = m_collisions.res->distances.find(pc);
    size_t l = 0;
    bool found = it != m_collisions.res->distances.end();
    if (found)
    {
      l = it->second.size();
      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
        return 0;
    }

    collision_detection::DistanceResultsData contact;
    contact.link_names[0] = cd0->getID();
    contact.link_names[1] = cd1->getID();
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.body_types[0] = cd0->m_type;
    contact.body_types[1] = cd1->m_type;
    contact.distance = cp.m_distance1;
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

    collision_detection::DistanceResultsData* col = processResult(m_collisions, contact, pc, found);
    if (!col)
    {
      return 0;
    }

    bool castShapeIsFirst = (colObj0Wrap->getCollisionObject() == m_cow.get());
    btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
    const CastHullShape* shape = dynamic_cast<const CastHullShape*>(
        (castShapeIsFirst ? colObj0Wrap : colObj1Wrap)->getCollisionObject()->getCollisionShape());
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
      col->cc_type = CCType_Time0;
    }
    else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
    {
      col->cc_time = 1;
      col->cc_type = CCType_Time1;
    }
    else
    {
      const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
      float l0c = (ptOnCast - ptWorld0).length(), l1c = (ptOnCast - ptWorld1).length();

      col->nearest_points[1] = convertBtToEigen(ptWorld0);
      col->cc_nearest_points[0] = col->nearest_points[1];
      col->cc_nearest_points[1] = convertBtToEigen(ptWorld1);
      col->cc_type = CCType_Between;

      if (l0c + l1c < BULLET_LENGTH_TOLERANCE)
      {
        col->cc_time = .5;
      }
      else
      {
        col->cc_time = l0c / (l0c + l1c);
      }
    }

    return 1;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask) &&
           (m_collisionFilterGroup & proxy0->m_collisionFilterMask) &&
           isCollisionAllowed(m_cow.get(),
                              static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject),
                              m_collisions.req->acm,
                              m_verbose);
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
    m_dispatcher->registerCollisionCreateFunc(
        BOX_SHAPE_PROXYTYPE,
        BOX_SHAPE_PROXYTYPE,
        m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
    m_dispatcher->setNearCallback(&nearCallback);

    btCollisionDispatcher* dispatcher = static_cast<btCollisionDispatcher*>(m_world->getDispatcher());
    dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() &
                                   ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
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
      m_world->addCollisionObject(
          element.second.get(), element.second->m_collisionFilterGroup, element.second->m_collisionFilterMask);
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

  void contactCastTestOriginal(const std::string& link_name,
                               const btTransform& tf1,
                               const btTransform& tf2,
                               BulletDistanceData& collisions)
  {
    COWPtr cow = m_link2cow[link_name];
    convexCastTestHelper(cow, cow->getCollisionShape(), tf1, tf2, collisions);
  }

  void convexSweepTest(const COWPtr cow, const btTransform& tf1, const btTransform& tf2, BulletDistanceData& collisions)
  {
    SweepCollisionCollector cc(collisions, cow);
    convexSweepTestHelper(cow->getCollisionShape(), tf1, tf2, cc);
  }

private:
  void convexCastTestHelper(COWPtr cow,
                            btCollisionShape* shape,
                            const btTransform& tf0,
                            const btTransform& tf1,
                            BulletDistanceData& collisions)
  {
    if (btBroadphaseProxy::isConvex(shape->getShapeType()))
    {
      btConvexShape* convex = dynamic_cast<btConvexShape*>(shape);

      collision_detection::CastHullShape* shape = new collision_detection::CastHullShape(convex, tf0.inverseTimes(tf1));
      collision_detection::COWPtr obj = cow->clone();
      obj->setCollisionShape(shape);
      obj->setWorldTransform(tf0);

      collision_detection::CastCollisionCollectorOriginal cc(collisions, obj, cow->getContactProcessingThreshold());
      m_world->contactTest(obj.get(), cc);

      delete shape;
    }
    else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape))
    {
      for (int i = 0; i < compound->getNumChildShapes(); ++i)
      {
        convexCastTestHelper(cow,
                             compound->getChildShape(i),
                             tf0 * compound->getChildTransform(i),
                             tf1 * compound->getChildTransform(i),
                             collisions);
      }
    }
    else
    {
      throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex "
                               "shapes");
    }
  }

  void convexSweepTestHelper(const btCollisionShape* shape,
                             const btTransform& tf1,
                             const btTransform& tf2,
                             btCollisionWorld::ConvexResultCallback& cc)
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
        convexSweepTestHelper(
            compound->getChildShape(i), tf1 * compound->getChildTransform(i), tf1 * compound->getChildTransform(i), cc);
      }
    }
    else
    {
      throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex "
                               "shapes");
    }
  }
};
typedef std::shared_ptr<BulletManager> BulletManagerPtr;

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom, bool useTrimesh, CollisionObjectWrapper* cow);
COWPtr CollisionObjectFromLink(const robot_model::LinkModel* link, bool useTrimesh);

inline void setContactDistance(COWPtr cow, double contact_distance)
{
  SHAPE_EXPANSION = btVector3(1, 1, 1) * contact_distance;
  gContactBreakingThreshold =
      2.001 * contact_distance;  // wtf. when I set it to 2.0 there are no contacts with distance > 0
  cow->setContactProcessingThreshold(contact_distance);
}
}

#endif
