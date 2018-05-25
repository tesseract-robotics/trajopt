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

#include <boost/bind.hpp>
#include <trajopt_moveit/collision_world_bullet.h>

void collision_detection::CollisionWorldBullet::constructBulletObject(Link2Cow& collision_objects,
                                                                      double contact_distance,
                                                                      bool allow_static2static) const
{
  for (std::pair<std::string, COWConstPtr> element : m_link2cow)
  {
    COWPtr new_cow(new COW(*(element.second.get())));
    assert(new_cow->getCollisionShape());

    new_cow->setWorldTransform(element.second->getWorldTransform());

    new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
    (allow_static2static) ?
        new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter | btBroadphaseProxy::StaticFilter :
        new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;

    setContactDistance(new_cow, contact_distance);
    collision_objects[element.first] = new_cow;
  }
}

collision_detection::CollisionWorldBullet::CollisionWorldBullet() : CollisionWorld()
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));
}

collision_detection::CollisionWorldBullet::CollisionWorldBullet(const WorldPtr& world) : CollisionWorld(world)
{
  // TODO: Need to loop through objects and add them
  m_link2cow.clear();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

collision_detection::CollisionWorldBullet::CollisionWorldBullet(const CollisionWorldBullet& other,
                                                                const WorldPtr& world)
  : CollisionWorld(other, world)
{
  m_link2cow = other.m_link2cow;

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));
}

collision_detection::CollisionWorldBullet::~CollisionWorldBullet() { getWorld()->removeObserver(observer_handle_); }
void collision_detection::CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const CollisionRobot& robot,
                                                                    const robot_state::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, robot, state, nullptr);
}

void collision_detection::CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const CollisionRobot& robot,
                                                                    const robot_state::RobotState& state,
                                                                    const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void collision_detection::CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const CollisionRobot& robot,
                                                                    const robot_state::RobotState& state1,
                                                                    const robot_state::RobotState& state2) const
{
  checkRobotCollisionHelper(req, res, robot, state1, state2, nullptr);
}

void collision_detection::CollisionWorldBullet::checkRobotCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const CollisionRobot& robot,
                                                                    const robot_state::RobotState& state1,
                                                                    const robot_state::RobotState& state2,
                                                                    const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, robot, state1, state2, &acm);
}

void collision_detection::CollisionWorldBullet::checkRobotCollisionHelper(const CollisionRequest& req,
                                                                          CollisionResult& res,
                                                                          const CollisionRobot& robot,
                                                                          const robot_state::RobotState& state,
                                                                          const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotBullet& robot_bullet = dynamic_cast<const CollisionRobotBullet&>(robot);
  BulletManager manager;
  Link2Cow robot_collision_objects;
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(robot_bullet.getRobotModel());
  dreq.enable_signed_distance = false;

  // Right now it will get distance information within 1.0 meter
  // Need to figure out the best way to expose this or see if bullet
  // has a true distance request.
  double contact_distance = 0.0;
  if (req.distance)
  {
    contact_distance = BULLET_DEFAULT_CONTACT_DISTANCE;
  }

  std::vector<std::string> robot_active_objects;
  robot_bullet.constructBulletObject(
      robot_collision_objects, robot_active_objects, contact_distance, state, dreq.active_components_only, false);

  constructBulletObject(manager.m_link2cow, contact_distance, false);
  manager.processCollisionObjects();

  BulletDistanceData collisions(&dreq, &dres);
  for (auto& obj : robot_active_objects)
  {
    COWPtr cow = robot_collision_objects[obj];
    assert(cow);

    manager.contactDiscreteTest(cow, collisions);

    if (collisions.done)
      break;
  }

  convertBulletCollisions(res, collisions);
}

void collision_detection::CollisionWorldBullet::checkRobotCollisionHelper(const CollisionRequest& req,
                                                                          CollisionResult& res,
                                                                          const CollisionRobot& robot,
                                                                          const robot_state::RobotState& state1,
                                                                          const robot_state::RobotState& state2,
                                                                          const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotBullet& robot_bullet = dynamic_cast<const CollisionRobotBullet&>(robot);
  BulletManager manager;
  Link2Cow robot_collision_objects;
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(robot_bullet.getRobotModel());
  dreq.enable_signed_distance = false;

  // Right now it will get distance information within 1.0 meter
  // Need to figure out the best way to expose this or see if bullet
  // has a true distance request.
  double contact_distance = 0.0;
  if (req.distance)
  {
    contact_distance = BULLET_DEFAULT_CONTACT_DISTANCE;
  }

  std::vector<std::string> robot_active_objects;
  robot_bullet.constructBulletObject(
      robot_collision_objects, robot_active_objects, contact_distance, state1, dreq.active_components_only, true);

  constructBulletObject(manager.m_link2cow, contact_distance, false);
  manager.processCollisionObjects();

  BulletDistanceData collisions(&dreq, &dres);
  for (auto& obj : robot_active_objects)
  {
    COWPtr cow = robot_collision_objects[obj];
    assert(cow);

    btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(cow->getLinkName()));
    btTransform tf2 = convertEigenToBt(state2.getGlobalLinkTransform(cow->getLinkName()));

    manager.convexSweepTest(cow, tf1, tf2, collisions);

    if (collisions.done)
      break;
  }

  convertBulletCollisions(res, collisions);
}

void collision_detection::CollisionWorldBullet::checkWorldCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const CollisionWorld& other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, nullptr);
}

void collision_detection::CollisionWorldBullet::checkWorldCollision(const CollisionRequest& req,
                                                                    CollisionResult& res,
                                                                    const CollisionWorld& other_world,
                                                                    const AllowedCollisionMatrix& acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void collision_detection::CollisionWorldBullet::checkWorldCollisionHelper(const CollisionRequest& req,
                                                                          CollisionResult& res,
                                                                          const CollisionWorld& other_world,
                                                                          const AllowedCollisionMatrix* acm) const
{
  const CollisionWorldBullet& other_bullet_world = dynamic_cast<const CollisionWorldBullet&>(other_world);
  BulletManager manager;
  Link2Cow other_world_objects;
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.acm = acm;
  dreq.group_name = req.group_name;
  dreq.enable_signed_distance = false;

  // Right now it will get distance information within 1.0 meter
  // Need to figure out the best way to expose this or see if bullet
  // has a true distance request.
  double contact_distance = 0.0;
  if (req.distance)
  {
    contact_distance = BULLET_DEFAULT_CONTACT_DISTANCE;
  }

  other_bullet_world.constructBulletObject(other_world_objects, contact_distance, true);

  constructBulletObject(manager.m_link2cow, contact_distance, true);
  manager.processCollisionObjects();

  BulletDistanceData collisions(&dreq, &dres);
  for (auto element : other_world_objects)
  {
    manager.contactDiscreteTest(element.second, collisions);
    if (collisions.done)
      break;
  }

  convertBulletCollisions(res, collisions);
}

void collision_detection::CollisionWorldBullet::updateBulletObject(const std::string& id)
{
  // check to see if we have this object
  auto it = getWorld()->find(id);
  if (it != getWorld()->end())
  {
    m_link2cow[id] = COWPtr(new COW(it->second.get()));
  }
  else
  {
    auto jt = m_link2cow.find(id);
    if (jt != m_link2cow.end())
      m_link2cow.erase(jt);
  }
}

void collision_detection::CollisionWorldBullet::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  m_link2cow.clear();

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBullet::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void collision_detection::CollisionWorldBullet::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {
    auto it = m_link2cow.find(obj->id_);
    if (it != m_link2cow.end())
    {
      m_link2cow.erase(it);
    }
  }
  else
  {
    updateBulletObject(obj->id_);
  }
}

void collision_detection::CollisionWorldBullet::distanceRobotHelper(const DistanceRequest& req,
                                                                    DistanceResult& res,
                                                                    const CollisionRobot& robot,
                                                                    const robot_state::RobotState& state) const
{
  const CollisionRobotBullet& robot_bullet = dynamic_cast<const CollisionRobotBullet&>(robot);
  BulletManager manager;
  Link2Cow robot_collision_objects;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> robot_active_objects;
  robot_bullet.constructBulletObject(
      robot_collision_objects, robot_active_objects, req.distance_threshold, state, req.active_components_only);

  constructBulletObject(manager.m_link2cow, req.distance_threshold, false);
  manager.processCollisionObjects();

  for (auto& obj : robot_active_objects)
  {
    COWPtr cow = robot_collision_objects[obj];
    assert(cow);

    manager.contactDiscreteTest(cow, collisions);

    if (collisions.done)
      break;
  }
}

void collision_detection::CollisionWorldBullet::distanceRobotHelper(const DistanceRequest& req,
                                                                    DistanceResult& res,
                                                                    const CollisionRobot& robot,
                                                                    const robot_state::RobotState& state1,
                                                                    const robot_state::RobotState& state2) const
{
  const CollisionRobotBullet& robot_bullet = dynamic_cast<const CollisionRobotBullet&>(robot);
  BulletManager manager;
  Link2Cow robot_collision_objects;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> robot_active_objects;
  robot_bullet.constructBulletObject(robot_collision_objects,
                                     robot_active_objects,
                                     req.distance_threshold,
                                     state1,
                                     state2,
                                     req.active_components_only);

  constructBulletObject(manager.m_link2cow, req.distance_threshold, false);
  manager.processCollisionObjects();

  for (auto& obj : robot_active_objects)
  {
    COWPtr cow = robot_collision_objects[obj];
    assert(cow);

    manager.contactCastTest(cow, collisions);

    if (collisions.done)
      break;
  }
}

double collision_detection::CollisionWorldBullet::distanceRobot(const CollisionRobot& robot,
                                                                const robot_state::RobotState& state,
                                                                bool verbose) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.verbose = verbose;
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  dreq.enableGroup(robot.getRobotModel());
  dreq.enable_signed_distance = true;
  distanceRobotHelper(dreq, dres, robot, state);

  return dres.minimum_distance.distance;
}

double collision_detection::CollisionWorldBullet::distanceRobot(const CollisionRobot& robot,
                                                                const robot_state::RobotState& state,
                                                                const AllowedCollisionMatrix& acm,
                                                                bool verbose) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.acm = &acm;
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  dreq.verbose = verbose;
  dreq.enableGroup(robot.getRobotModel());
  dreq.enable_signed_distance = true;
  distanceRobotHelper(dreq, dres, robot, state);

  return dres.minimum_distance.distance;
}

void collision_detection::CollisionWorldBullet::distanceRobot(const DistanceRequest& req,
                                                              DistanceResult& res,
                                                              const CollisionRobot& robot,
                                                              const robot_state::RobotState& state) const
{
  distanceRobotHelper(req, res, robot, state);
}

void collision_detection::CollisionWorldBullet::distanceRobot(const DistanceRequest& req,
                                                              DistanceResult& res,
                                                              const CollisionRobot& robot,
                                                              const robot_state::RobotState& state1,
                                                              const robot_state::RobotState& state2) const
{
  distanceRobotHelper(req, res, robot, state1, state2);
}

double collision_detection::CollisionWorldBullet::distanceWorld(const CollisionWorld& world, bool verbose) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.verbose = verbose;
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  dreq.enable_signed_distance = true;
  distanceWorldHelper(dreq, dres, world);

  return dres.minimum_distance.distance;
}

double collision_detection::CollisionWorldBullet::distanceWorld(const CollisionWorld& world,
                                                                const AllowedCollisionMatrix& acm,
                                                                bool verbose) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.acm = &acm;
  dreq.distance_threshold = BULLET_DEFAULT_CONTACT_DISTANCE;
  dreq.verbose = verbose;
  dreq.enable_signed_distance = true;
  distanceWorldHelper(dreq, dres, world);

  return dres.minimum_distance.distance;
}

void collision_detection::CollisionWorldBullet::distanceWorld(const DistanceRequest& req,
                                                              DistanceResult& res,
                                                              const CollisionWorld& world) const
{
  distanceWorldHelper(req, res, world);
}

void collision_detection::CollisionWorldBullet::distanceWorldHelper(const DistanceRequest& req,
                                                                    DistanceResult& res,
                                                                    const CollisionWorld& world) const
{
  const CollisionWorldBullet& other_bullet_world = dynamic_cast<const CollisionWorldBullet&>(world);
  BulletManager manager;
  Link2Cow other_world_objects;
  BulletDistanceData collisions(&req, &res);

  other_bullet_world.constructBulletObject(other_world_objects, req.distance_threshold, true);

  constructBulletObject(manager.m_link2cow, req.distance_threshold, true);
  manager.processCollisionObjects();

  for (auto element : other_world_objects)
  {
    manager.contactDiscreteTest(element.second, collisions);
    if (collisions.done)
      break;
  }
}
