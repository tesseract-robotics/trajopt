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

#include <trajopt_moveit/collision_robot_bullet.h>

const bool BULLET_DEFAULT_USE_ORIGINAL_CAST = false;

collision_detection::CollisionRobotBullet::CollisionRobotBullet(const robot_model::RobotModelConstPtr& model,
                                                                double padding,
                                                                double scale)
  : CollisionRobot(model, padding, scale)
{
  ros::NodeHandle nh("~");

  nh.param<bool>("bullet/use_original_cast", m_use_original_cast, BULLET_DEFAULT_USE_ORIGINAL_CAST);

  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  // we keep the same order of objects as what RobotState *::getLinkState()
  // returns
  for (auto link : links)
  {
    if (link->getShapes().size() > 0)
    {
      COWPtr new_cow(new COW(link));
      if (new_cow)
      {
        setContactDistance(new_cow, BULLET_DEFAULT_CONTACT_DISTANCE);
        m_link2cow[new_cow->getID()] = new_cow;
        logDebug("Added collision object for link %s", link->getName().c_str());
      }
      else
      {
        logWarn("ignoring link %s", link->getName().c_str());
      }
    }
  }
}

collision_detection::CollisionRobotBullet::CollisionRobotBullet(const CollisionRobotBullet& other)
  : CollisionRobot(other)
{
  m_link2cow = other.m_link2cow;
  m_use_original_cast = other.m_use_original_cast;
}

void collision_detection::CollisionRobotBullet::constructBulletObject(
    Link2Cow& collision_objects,
    std::vector<std::string>& active_objects,
    double contact_distance,
    const robot_state::RobotState& state,
    const std::set<const robot_model::LinkModel*>* active_links,
    bool continuous) const
{
  for (std::pair<std::string, COWConstPtr> element : m_link2cow)
  {
    COWPtr new_cow(new COW(*(element.second.get())));
    assert(new_cow->getCollisionShape());

    Eigen::Isometry3d tf = state.getGlobalLinkTransform(element.first);
    new_cow->setWorldTransform(convertEigenToBt(tf));

    // For descrete checks we can check static to kinematic and kinematic to
    // kinematic
    new_cow->m_collisionFilterGroup = (active_links && (std::find_if(active_links->begin(),
                                                                     active_links->end(),
                                                                     [&](const robot_model::LinkModel* link) {
                                                                       return link->getName() == element.first;
                                                                     }) == active_links->end())) ?
                                          btBroadphaseProxy::StaticFilter :
                                          btBroadphaseProxy::KinematicFilter;
    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(element.first);
      (continuous) ?
          (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter) :
          (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[element.first] = new_cow;
  }

  // TODO: Implement a method for caching fcl::CollisionObject's for
  // robot_state::AttachedBody's
  std::vector<const robot_state::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (auto& body : ab)
  {
    COWPtr new_cow(new COW(body));

    btTransform tf = convertEigenToBt(state.getGlobalLinkTransform(body->getAttachedLinkName()));
    new_cow->setWorldTransform(tf);

    // For descrete checks we can check static to kinematic and kinematic to
    // kinematic
    new_cow->m_collisionFilterGroup =
        (active_links && (std::find_if(active_links->begin(),
                                       active_links->end(),
                                       [&](const robot_model::LinkModel* link) {
                                         return link->getName() == body->getAttachedLinkName();
                                       }) == active_links->end())) ?
            btBroadphaseProxy::StaticFilter :
            btBroadphaseProxy::KinematicFilter;
    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(new_cow->getID());
      (continuous) ?
          (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter) :
          (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[new_cow->getID()] = new_cow;
  }
}

void collision_detection::CollisionRobotBullet::constructBulletObject(
    Link2Cow& collision_objects,
    std::vector<std::string>& active_objects,
    double contact_distance,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const std::set<const robot_model::LinkModel*>* active_links) const
{
  for (std::pair<std::string, COWConstPtr> element : m_link2cow)
  {
    COWPtr new_cow(new COW(*(element.second.get())));

    new_cow->m_collisionFilterGroup = (active_links && (std::find_if(active_links->begin(),
                                                                     active_links->end(),
                                                                     [&](const robot_model::LinkModel* link) {
                                                                       return link->getName() == element.first;
                                                                     }) == active_links->end())) ?
                                          btBroadphaseProxy::StaticFilter :
                                          btBroadphaseProxy::KinematicFilter;

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(element.first));

      new_cow->setWorldTransform(tf1);
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(element.first);

      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
        assert(convex != nullptr);

        btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(element.first));
        btTransform tf2 = convertEigenToBt(state2.getGlobalLinkTransform(element.first));

        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
        assert(shape != nullptr);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape);
        new_cow->setWorldTransform(tf1);
      }
      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
        Eigen::Isometry3d tf1 = state1.getGlobalLinkTransform(element.first);
        Eigen::Isometry3d tf2 = state2.getGlobalLinkTransform(element.first);

        btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
          assert(convex != nullptr);

          btTransform geomTrans = compound->getChildTransform(i);
          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

          btCollisionShape* subshape = new CastHullShape(convex, child_tf1.inverseTimes(child_tf2));
          assert(subshape != nullptr);

          if (subshape != nullptr)
          {
            new_cow->manage(subshape);
            subshape->setMargin(BULLET_MARGIN);
            new_compound->addChildShape(geomTrans, subshape);
          }
        }

        new_compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to
                                                 // have no effect when positive
                                                 // but has an effect when
                                                 // negative
        new_cow->manage(new_compound);
        new_cow->setCollisionShape(new_compound);
        new_cow->setWorldTransform(convertEigenToBt(tf1));
      }
      else
      {
        CONSOLE_BRIDGE_logError("I can only continuous collision check convex "
                                "shapes and compound shapes made of convex "
                                "shapes");
      }

      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[element.first] = new_cow;
  }

  std::vector<const robot_state::AttachedBody*> ab;
  state1.getAttachedBodies(ab);
  for (auto& body : ab)
  {
    COWPtr new_cow(new COW(body));

    new_cow->m_collisionFilterGroup =
        (active_links && (std::find_if(active_links->begin(),
                                       active_links->end(),
                                       [&](const robot_model::LinkModel* link) {
                                         return link->getName() == body->getAttachedLinkName();
                                       }) == active_links->end())) ?
            btBroadphaseProxy::StaticFilter :
            btBroadphaseProxy::KinematicFilter;

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      btTransform tf = convertEigenToBt(state1.getGlobalLinkTransform(body->getAttachedLinkName()));

      new_cow->setWorldTransform(tf);
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(new_cow->getID());

      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
        assert(convex != nullptr);

        btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(body->getAttachedLinkName()));
        btTransform tf2 = convertEigenToBt(state2.getGlobalLinkTransform(body->getAttachedLinkName()));

        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
        assert(shape != nullptr);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape);
        new_cow->setWorldTransform(tf1);
      }
      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
        Eigen::Isometry3d tf1 = state1.getGlobalLinkTransform(body->getAttachedLinkName());
        Eigen::Isometry3d tf2 = state2.getGlobalLinkTransform(body->getAttachedLinkName());

        btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
          assert(convex != nullptr);

          btTransform geomTrans = compound->getChildTransform(i);
          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

          btCollisionShape* subshape = new CastHullShape(convex, child_tf1.inverseTimes(child_tf2));
          assert(subshape != nullptr);

          if (subshape != nullptr)
          {
            new_cow->manage(subshape);
            subshape->setMargin(BULLET_MARGIN);
            new_compound->addChildShape(geomTrans, subshape);
          }
        }

        new_compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to
                                                 // have no effect when positive
                                                 // but has an effect when
                                                 // negative
        new_cow->manage(new_compound);
        new_cow->setCollisionShape(new_compound);
        new_cow->setWorldTransform(convertEigenToBt(tf1));
      }
      else
      {
        CONSOLE_BRIDGE_logError("I can only continuous collision check convex "
                                "shapes and compound shapes made of convex "
                                "shapes");
      }

      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[new_cow->getID()] = new_cow;
  }
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req,
                                                                   CollisionResult& res,
                                                                   const robot_state::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req,
                                                                   CollisionResult& res,
                                                                   const robot_state::RobotState& state,
                                                                   const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req,
                                                                   CollisionResult& res,
                                                                   const robot_state::RobotState& state1,
                                                                   const robot_state::RobotState& state2) const
{
  checkSelfCollisionHelper(req, res, state1, state2, nullptr);
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req,
                                                                   CollisionResult& res,
                                                                   const robot_state::RobotState& state1,
                                                                   const robot_state::RobotState& state2,
                                                                   const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state1, state2, &acm);
}

void collision_detection::CollisionRobotBullet::checkSelfCollisionHelper(const CollisionRequest& req,
                                                                         CollisionResult& res,
                                                                         const robot_state::RobotState& state,
                                                                         const AllowedCollisionMatrix* acm) const
{
  BulletManager manager;
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(getRobotModel());
  dreq.enable_signed_distance = false;

  // Right now it will get distance information within 1.0 meter
  // Need to figure out the best way to expose this or see if bullet
  // has a true distance request.
  double contact_distance = 0.0;
  if (req.distance)
  {
    contact_distance = BULLET_DEFAULT_CONTACT_DISTANCE;
  }

  std::vector<std::string> active_objects;
  constructBulletObject(manager.m_link2cow, active_objects, contact_distance, state, dreq.active_components_only);
  manager.processCollisionObjects();

  BulletDistanceData collisions(&dreq, &dres);
  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    manager.contactDiscreteTest(cow, collisions);

    if (collisions.done)
      break;
  }

  convertBulletCollisions(res, collisions);
}

void collision_detection::CollisionRobotBullet::checkSelfCollisionHelper(const CollisionRequest& req,
                                                                         CollisionResult& res,
                                                                         const robot_state::RobotState& state1,
                                                                         const robot_state::RobotState& state2,
                                                                         const AllowedCollisionMatrix* acm) const
{
  BulletManager manager;
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(getRobotModel());
  dreq.enable_signed_distance = false;

  // Right now it will get distance information within 1.0 meter
  // Need to figure out the best way to expose this or see if bullet
  // has a true distance request.
  double contact_distance = 0.0;
  if (req.distance)
  {
    contact_distance = 1.0;
  }

  std::vector<std::string> active_objects;
  constructBulletObject(
      manager.m_link2cow, active_objects, contact_distance, state1, dreq.active_components_only, true);
  manager.processCollisionObjects();

  BulletDistanceData collisions(&dreq, &dres);
  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    Eigen::Isometry3d tf1 = state1.getGlobalLinkTransform(cow->getLinkName());
    Eigen::Isometry3d tf2 = state2.getGlobalLinkTransform(cow->getLinkName());
    manager.convexSweepTest(cow, convertEigenToBt(tf1), convertEigenToBt(tf2), collisions);

    if (collisions.done)
      break;
  }

  convertBulletCollisions(res, collisions);
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const robot_state::RobotState& state,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, nullptr);
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const robot_state::RobotState& state,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state,
                                                                    const AllowedCollisionMatrix& acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const robot_state::RobotState& state1,
                                                                    const robot_state::RobotState& state2,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state1,
                                                                    const robot_state::RobotState& other_state2) const
{
  CONSOLE_BRIDGE_logError("Bullet continuous collision checking not yet "
                          "implemented for robot to robot.");
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const robot_state::RobotState& state1,
                                                                    const robot_state::RobotState& state2,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state1,
                                                                    const robot_state::RobotState& other_state2,
                                                                    const AllowedCollisionMatrix& acm) const
{
  CONSOLE_BRIDGE_logError("Bullet continuous collision checking not yet "
                          "implemented for robot to robot.");
}

void collision_detection::CollisionRobotBullet::checkOtherCollisionHelper(const CollisionRequest& req,
                                                                          CollisionResult& res,
                                                                          const robot_state::RobotState& state,
                                                                          const CollisionRobot& other_robot,
                                                                          const robot_state::RobotState& other_state,
                                                                          const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotBullet& other_bullet_robot = dynamic_cast<const CollisionRobotBullet&>(other_robot);
  BulletManager other_robot_manager;
  Link2Cow robot_objects;
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(getRobotModel());
  dreq.enable_signed_distance = false;

  // Right now it will get distance information within 1.0 meter
  // Need to figure out the best way to expose this or see if bullet
  // has a true distance request.
  double contact_distance = 0.0;
  if (req.distance)
  {
    contact_distance = BULLET_DEFAULT_CONTACT_DISTANCE;
  }

  std::vector<std::string> active_objects, other_active_objects;
  other_bullet_robot.constructBulletObject(
      other_robot_manager.m_link2cow, other_active_objects, contact_distance, other_state, nullptr);
  other_robot_manager.processCollisionObjects();

  constructBulletObject(robot_objects, active_objects, contact_distance, state, dreq.active_components_only);

  BulletDistanceData collisions(&dreq, &dres);
  for (auto& obj : active_objects)
  {
    COWPtr cow = robot_objects[obj];
    assert(cow);

    other_robot_manager.contactDiscreteTest(cow, collisions);

    if (collisions.done)
      break;
  }

  convertBulletCollisions(res, collisions);
}

void collision_detection::CollisionRobotBullet::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  CONSOLE_BRIDGE_logError("Bullet updatedPaddingOrScaling not implemented");
}

double collision_detection::CollisionRobotBullet::distanceSelf(const robot_state::RobotState& state) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.enableGroup(getRobotModel());
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  distanceSelfHelper(dreq, dres, state);
  dreq.enable_signed_distance = true;
  return dres.minimum_distance.distance;
}

double collision_detection::CollisionRobotBullet::distanceSelf(const robot_state::RobotState& state,
                                                               const AllowedCollisionMatrix& acm) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.acm = &acm;
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  dreq.enableGroup(getRobotModel());
  dreq.enable_signed_distance = true;
  distanceSelfHelper(dreq, dres, state);
  return dres.minimum_distance.distance;
}

void collision_detection::CollisionRobotBullet::distanceSelf(const DistanceRequest& req,
                                                             DistanceResult& res,
                                                             const robot_state::RobotState& state) const
{
  distanceSelfHelper(req, res, state);
}

void collision_detection::CollisionRobotBullet::distanceSelf(const DistanceRequest& req,
                                                             DistanceResult& res,
                                                             const robot_state::RobotState& state1,
                                                             const robot_state::RobotState& state2) const
{
  distanceSelfHelper(req, res, state1, state2);
}

void collision_detection::CollisionRobotBullet::distanceSelfHelper(const DistanceRequest& req,
                                                                   DistanceResult& res,
                                                                   const robot_state::RobotState& state) const
{
  BulletManager manager;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> active_objects;
  constructBulletObject(manager.m_link2cow, active_objects, req.distance_threshold, state, req.active_components_only);
  manager.processCollisionObjects();

  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    manager.contactDiscreteTest(cow, collisions);

    if (collisions.done)
      break;
  }
}

void collision_detection::CollisionRobotBullet::distanceSelfHelper(const DistanceRequest& req,
                                                                   DistanceResult& res,
                                                                   const robot_state::RobotState& state1,
                                                                   const robot_state::RobotState& state2) const
{
  BulletManager manager;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> active_objects;
  constructBulletObject(
      manager.m_link2cow, active_objects, req.distance_threshold, state1, state2, req.active_components_only);
  manager.processCollisionObjects();

  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    manager.contactCastTest(cow, collisions);

    if (collisions.done)
      break;
  }
}

void collision_detection::CollisionRobotBullet::distanceSelfHelperOriginal(const DistanceRequest& req,
                                                                           DistanceResult& res,
                                                                           const robot_state::RobotState& state1,
                                                                           const robot_state::RobotState& state2) const
{
  BulletManager manager;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> active_objects;
  constructBulletObject(
      manager.m_link2cow, active_objects, req.distance_threshold, state1, req.active_components_only, true);
  manager.processCollisionObjects();

  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(cow->getLinkName()));
    btTransform tf2 = convertEigenToBt(state2.getGlobalLinkTransform(cow->getLinkName()));
    manager.contactCastTestOriginal(obj, tf1, tf2, collisions);

    if (collisions.done)
      break;
  }
}

double collision_detection::CollisionRobotBullet::distanceOther(const robot_state::RobotState& state,
                                                                const CollisionRobot& other_robot,
                                                                const robot_state::RobotState& other_state) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.enableGroup(getRobotModel());
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  dreq.enable_signed_distance = true;
  distanceOtherHelper(dreq, dres, state, other_robot, other_state);
  return dres.minimum_distance.distance;
}

double collision_detection::CollisionRobotBullet::distanceOther(const robot_state::RobotState& state,
                                                                const CollisionRobot& other_robot,
                                                                const robot_state::RobotState& other_state,
                                                                const AllowedCollisionMatrix& acm) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.acm = &acm;
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  dreq.enableGroup(getRobotModel());
  dreq.enable_signed_distance = true;
  distanceOtherHelper(dreq, dres, state, other_robot, other_state);
  return dres.minimum_distance.distance;
}

void collision_detection::CollisionRobotBullet::distanceOther(const DistanceRequest& req,
                                                              DistanceResult& res,
                                                              const robot_state::RobotState& state,
                                                              const CollisionRobot& other_robot,
                                                              const robot_state::RobotState& other_state) const
{
  distanceOtherHelper(req, res, state, other_robot, other_state);
}

void collision_detection::CollisionRobotBullet::distanceOtherHelper(const DistanceRequest& req,
                                                                    DistanceResult& res,
                                                                    const robot_state::RobotState& state,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state) const
{
  const CollisionRobotBullet& other_bullet_robot = dynamic_cast<const CollisionRobotBullet&>(other_robot);
  BulletManager other_robot_manager;
  Link2Cow robot_objects;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> active_objects, other_active_objects;
  other_bullet_robot.constructBulletObject(
      other_robot_manager.m_link2cow, other_active_objects, req.distance_threshold, other_state, nullptr);
  other_robot_manager.processCollisionObjects();

  constructBulletObject(robot_objects, active_objects, req.distance_threshold, state, req.active_components_only);

  for (auto& obj : active_objects)
  {
    COWPtr cow = robot_objects[obj];
    assert(cow);

    other_robot_manager.contactDiscreteTest(cow, collisions);

    if (collisions.done)
      break;
  }
}
