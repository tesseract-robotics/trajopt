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

class CollisionObjectWrapper : public btCollisionObject {
public:
  CollisionObjectWrapper(const robot_model::LinkModel* link) : m_link(link), m_index(-1) {}
  std::vector<boost::shared_ptr<void>> m_data;
  const robot_model::LinkModel* m_link;
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
typedef boost::shared_ptr<const CollisionObjectWrapper> COWConstPtr;
typedef std::map<const robot_model::LinkModel*, COWPtr> Link2Cow;
typedef std::map<const robot_model::LinkModel*, COWConstPtr> Link2ConstCow;

inline const robot_model::LinkModel* getLink(const btCollisionObject* o) {
  return static_cast<const CollisionObjectWrapper*>(o)->m_link;
}

inline void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
    logError("error");
//  // only used for AllVsAll
//    BulletCollisionChecker* cc = static_cast<BulletCollisionChecker*>(dispatcher.m_userData);
//    if ( cc->CanCollide(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject),
//                        static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)))
//      dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
//  }
}

struct CollisionCollector : public btCollisionWorld::ContactResultCallback
{
  std::vector<collision_detection::DistanceResultsData>& m_collisions;
  const COWPtr m_cow;
  const AllowedCollisionMatrix* m_acm;
  bool m_verbose;

  CollisionCollector(std::vector<collision_detection::DistanceResultsData>& collisions, const COWPtr cow, const AllowedCollisionMatrix* acm, bool verbose = false) :
    m_collisions(collisions), m_cow(cow), m_acm(acm), m_verbose(verbose) {}

  virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0, const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
  {
    if (cp.m_distance1 > 0.075) return 0;
    const robot_model::LinkModel* linkA = getLink(colObj0Wrap->getCollisionObject());
    const robot_model::LinkModel* linkB = getLink(colObj1Wrap->getCollisionObject());
    collision_detection::DistanceResultsData contact;
    contact.link_names[0] = linkA->getName();
    contact.link_names[1] = linkB->getName();
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.distance = cp.m_distance1;
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);
    contact.hasNearestPoints = true;
    m_collisions.push_back(contact);

    CONSOLE_BRIDGE_logDebug("CollisionCollector: adding collision %s-%s (%.4f)", linkA->getName().c_str(), linkB->getName().c_str(), cp.m_distance1);
    return 1;
  }
  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    bool check1 = (proxy0->m_collisionFilterGroup & m_collisionFilterMask);
    bool check2 = (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
    bool check3 = canCollide(m_cow.get(), static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject));

    return check1 && check2 && check3;

//    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask)
//        && (m_collisionFilterGroup & proxy0->m_collisionFilterMask)
//        && canCollide(m_cow.get(), static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject));
  }

private:
  bool canCollide(const COW* cow0, const COW* cow1) const
  {
    const robot_model::LinkModel* linkA = cow0->m_link;
    const robot_model::LinkModel* linkB = cow1->m_link;

    // use the collision matrix (if any) to avoid certain collision checks
    DecideContactFn dcf;
    bool always_allow_collision = false;
    if (m_acm)
    {
      AllowedCollision::Type type;
      bool found = m_acm->getAllowedCollision(linkA->getName(), linkB->getName(), type);
      if (found)
      {
        // if we have an entry in the collision matrix, we read it
        if (type == AllowedCollision::ALWAYS)
        {
          always_allow_collision = true;
          if (m_verbose)
          {
            CONSOLE_BRIDGE_logDebug(
                "Collision between '%s' and '%s' is always allowed. No contacts are computed.",
                linkA->getName().c_str(), linkB->getName().c_str());
          }
        }
        else if (type == AllowedCollision::CONDITIONAL)
        {
          m_acm->getAllowedCollision(linkA->getName(), linkB->getName(), dcf);
          if (m_verbose)
            CONSOLE_BRIDGE_logDebug("Collision between '%s' and '%s' is conditionally allowed", linkA->getName().c_str(),
                                    linkB->getName().c_str());
        }
      }
    }

//    // check if a link is touching an attached object
//    if (cd1->type == BodyTypes::ROBOT_LINK && cd2->type == BodyTypes::ROBOT_ATTACHED)
//    {
//      const std::set<std::string>& tl = cd2->ptr.ab->getTouchLinks();
//      if (tl.find(cd1->getID()) != tl.end())
//      {
//        always_allow_collision = true;
//        if (cdata->req_->verbose)
//          CONSOLE_BRIDGE_logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
//                                  cd1->getID().c_str(), cd2->getID().c_str());
//      }
//    }
//    else if (cd2->type == BodyTypes::ROBOT_LINK && cd1->type == BodyTypes::ROBOT_ATTACHED)
//    {
//      const std::set<std::string>& tl = cd1->ptr.ab->getTouchLinks();
//      if (tl.find(cd2->getID()) != tl.end())
//      {
//        always_allow_collision = true;
//        if (cdata->req_->verbose)
//          CONSOLE_BRIDGE_logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
//                                  cd2->getID().c_str(), cd1->getID().c_str());
//      }
//    }
//    // bodies attached to the same link should not collide
//    if (cd1->type == BodyTypes::ROBOT_ATTACHED && cd2->type == BodyTypes::ROBOT_ATTACHED)
//    {
//      if (cd1->ptr.ab->getAttachedLink() == cd2->ptr.ab->getAttachedLink())
//        always_allow_collision = true;
//    }

    // if collisions are always allowed, we are done
    if (always_allow_collision)
      return false;

    if (m_verbose)
      CONSOLE_BRIDGE_logDebug("Actually checking collisions between %s and %s", linkA->getName().c_str(),
                              linkB->getName().c_str());

    return true;
  }
};

struct BulletManager
{
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  Link2Cow m_link2cow;


  BulletManager(const AllowedCollisionMatrix* acm) : m_acm(acm)
  {
    m_coll_config = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_coll_config);
    m_broadphase = new btDbvtBroadphase();
    m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
    m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
    m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
    m_dispatcher->setNearCallback(&nearCallback);
  }

  ~BulletManager()
  {
    delete m_world;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_coll_config;
  }

  void contactTest(const robot_model::LinkModel* link, std::vector<collision_detection::DistanceResultsData>& collisions, short filterMask)
  {
    COWPtr cow = m_link2cow[link];
    contactTest(cow, collisions, filterMask);
  }

  void contactTest(const COWPtr cow, std::vector<collision_detection::DistanceResultsData>& collisions, short filterMask)
  {
    CollisionCollector cc(collisions, cow, m_acm);
    cc.m_collisionFilterMask = filterMask;
    m_world->contactTest(cow.get(), cc);
  }

  void setContactDistance(float dist) {
    CONSOLE_BRIDGE_logDebug("setting contact distance to %.2f", dist);
    SHAPE_EXPANSION = btVector3(1,1,1) * dist;
    gContactBreakingThreshold = 2.001 * dist; // wtf. when I set it to 2.0 there are no contacts with distance > 0
    btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
    for (int i=0; i < objs.size(); ++i)
    {
      objs[i]->setContactProcessingThreshold(dist);
    }
    btCollisionDispatcher* dispatcher = static_cast<btCollisionDispatcher*>(m_world->getDispatcher());
    dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
  }

private:
  const AllowedCollisionMatrix* m_acm;
};

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom, bool useTrimesh, CollisionObjectWrapper* cow);
COWPtr CollisionObjectFromLink(const robot_model::LinkModel* link, bool useTrimesh);

}

#endif
