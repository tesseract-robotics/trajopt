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

#include <trajopt/moveit/collision_robot_bullet.h>

collision_detection::CollisionRobotBullet::CollisionRobotBullet(const robot_model::RobotModelConstPtr& model, double padding, double scale)
  : CollisionRobot(model, padding, scale)
{
  m_contactDistance = 0.05;

  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  // we keep the same order of objects as what RobotState *::getLinkState() returns
  bool useTrimesh;
  for (auto link : links)
  {
    if (link->getShapes().size() > 0)
    {
      useTrimesh = false;

      // TODO: Need to figure out how to determine if a link is rigidly fix to the world
      // This is added to test the planning unit test
//      if (link->getName() == "table_link")
//      {
//        useTrimesh = true;
//      }

      COWPtr new_cow = CollisionObjectFromLink(link, useTrimesh);
      if (new_cow)
      {
        new_cow->setContactProcessingThreshold(m_contactDistance);
        m_link2cow[link] = new_cow;
        logDebug("Added collision object for link %s", link->getName().c_str());
      }
      else
      {
        logWarn("ignoring link %s", link->getName().c_str());
      }
    }
  }
}

collision_detection::CollisionRobotBullet::CollisionRobotBullet(const CollisionRobotBullet& other) : CollisionRobot(other)
{
  m_link2cow = other.m_link2cow;
  m_contactDistance = other.m_contactDistance;
}

//void collision_detection::CollisionRobotBullet::getAttachedBodyObjects(const robot_state::AttachedBody* ab,
//                                                                       std::vector<FCLGeometryConstPtr>& geoms) const
//{
//  const std::vector<shapes::ShapeConstPtr>& shapes = ab->getShapes();
//  for (std::size_t i = 0; i < shapes.size(); ++i)
//  {
//    FCLGeometryConstPtr co = createCollisionGeometry(shapes[i], ab, i);
//    if (co)
//      geoms.push_back(co);
//  }
//}

void collision_detection::CollisionRobotBullet::constructBulletObject(BulletManager& manager, const robot_state::RobotState& state, const std::set<const robot_model::LinkModel*> *active_links) const
{

  for (std::pair<const robot_model::LinkModel*, COWConstPtr> element : m_link2cow)
  {
    COWPtr new_cow(new COW(*(element.second.get())));
    manager.m_link2cow[element.first] = new_cow;

    Eigen::Affine3d tf = state.getGlobalLinkTransform(element.first);
    new_cow->setWorldTransform(convertEigenToBt(tf));

    // For descrete checks we can check static to kinematic and kinematic to kinematic
    new_cow->m_collisionFilterGroup = (active_links && (active_links->find(element.first) == active_links->end())) ? btBroadphaseProxy::StaticFilter : btBroadphaseProxy::KinematicFilter;
    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
    }

    manager.m_world->addCollisionObject(new_cow.get(), new_cow->m_collisionFilterGroup, new_cow->m_collisionFilterMask);
  }

//  // TODO: Implement a method for caching fcl::CollisionObject's for robot_state::AttachedBody's
//  std::vector<const robot_state::AttachedBody*> ab;
//  state.getAttachedBodies(ab);
//  for (auto& body : ab)
//  {
//    std::vector<FCLGeometryConstPtr> objs;
//    getAttachedBodyObjects(body, objs);
//    const EigenSTL::vector_Affine3d& ab_t = body->getGlobalCollisionBodyTransforms();
//    for (std::size_t k = 0; k < objs.size(); ++k)
//      if (objs[k]->collision_geometry_)
//      {
//        transform2fcl(ab_t[k], fcl_tf);
//        fcl_obj.collision_objects_.push_back(
//            FCLCollisionObjectPtr(new fcl::CollisionObject(objs[k]->collision_geometry_, fcl_tf)));
//        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
//        // and would be destroyed when objs goes out of scope.
//        fcl_obj.collision_geometry_.push_back(objs[k]);
//      }
//  }

  // TODO: This should probably be moved
  manager.setContactDistance(0.05);
}

void collision_detection::CollisionRobotBullet::constructBulletObject(BulletManager& manager,
                                                                      const robot_state::RobotState& state1,
                                                                      const robot_state::RobotState& state2,
                                                                      const std::set<const robot_model::LinkModel*> *active_links) const
{
  for (std::pair<const robot_model::LinkModel*, COWConstPtr> element : m_link2cow)
  {
    COWPtr new_cow(new COW(element.second->m_link));
    new_cow->m_index = element.second->m_index;

    if (const btConvexShape* convex = dynamic_cast<const btConvexShape*>(element.second->getCollisionShape()))
    {
      Eigen::Affine3d tf1 = state1.getGlobalLinkTransform(element.first);
      Eigen::Affine3d tf2 = state2.getGlobalLinkTransform(element.first);

      CastHullShape* shape = new CastHullShape(convex, convertEigenToBt(tf1.inverse() * tf2));

      new_cow->manage(shape);
      new_cow->setCollisionShape(shape);
      new_cow->setWorldTransform(convertEigenToBt(tf1));
    }
    else if (const btCompoundShape* compound = dynamic_cast<const btCompoundShape*>(new_cow->getCollisionShape()))
    {
      Eigen::Affine3d tf1 = state1.getGlobalLinkTransform(element.first);
      Eigen::Affine3d tf2 = state2.getGlobalLinkTransform(element.first);

      btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);
      new_cow->manage(new_compound);
      new_compound->setMargin(BULLET_MARGIN); //margin: compound. seems to have no effect when positive but has an effect when negative
      new_cow->setCollisionShape(new_compound);
      new_cow->setWorldTransform(convertEigenToBt(tf1));

      for (int i = 0; i < compound->getNumChildShapes(); ++i)
      {
        btCollisionShape* subshape = new CastHullShape(static_cast<const btConvexShape*>(compound->getChildShape(i)), convertEigenToBt(tf1.inverse() * tf2));
        if (subshape != NULL)
        {
          new_cow->manage(subshape);
          subshape->setMargin(BULLET_MARGIN);
          btTransform geomTrans = compound->getChildTransform(i);
          new_compound->addChildShape(geomTrans, subshape);
        }
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
    }

    new_cow->m_collisionFilterGroup = (active_links && (active_links->find(element.first) == active_links->end())) ? btBroadphaseProxy::StaticFilter : btBroadphaseProxy::KinematicFilter;
    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    manager.m_link2cow[element.first] = new_cow;
    manager.m_world->addCollisionObject(new_cow.get(), new_cow->m_collisionFilterGroup, new_cow->m_collisionFilterMask);
  }

//  // TODO: Implement a method for caching fcl::CollisionObject's for robot_state::AttachedBody's
//  std::vector<const robot_state::AttachedBody*> ab;
//  state.getAttachedBodies(ab);
//  for (auto& body : ab)
//  {
//    std::vector<FCLGeometryConstPtr> objs;
//    getAttachedBodyObjects(body, objs);
//    const EigenSTL::vector_Affine3d& ab_t = body->getGlobalCollisionBodyTransforms();
//    for (std::size_t k = 0; k < objs.size(); ++k)
//      if (objs[k]->collision_geometry_)
//      {
//        transform2fcl(ab_t[k], fcl_tf);
//        fcl_obj.collision_objects_.push_back(
//            FCLCollisionObjectPtr(new fcl::CollisionObject(objs[k]->collision_geometry_, fcl_tf)));
//        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
//        // and would be destroyed when objs goes out of scope.
//        fcl_obj.collision_geometry_.push_back(objs[k]);
//      }
//  }

  // TODO: This should probably be moved
  manager.setContactDistance(0.05);
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                const robot_state::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                const robot_state::RobotState& state,
                                                                const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                const robot_state::RobotState& state1,
                                                                const robot_state::RobotState& state2) const
{
  checkSelfCollisionHelper(req, res, state1, state2, nullptr);
}

void collision_detection::CollisionRobotBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                                                const robot_state::RobotState& state1,
                                                                const robot_state::RobotState& state2,
                                                                const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state1, state2, &acm);
}

void collision_detection::CollisionRobotBullet::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                                         const robot_state::RobotState& state,
                                                                         const AllowedCollisionMatrix* acm) const
{
  BulletManager manager(acm);
  DistanceRequest dreq;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(getRobotModel());

  manager.setContactDistance(0.0);
  constructBulletObject(manager, state, dreq.active_components_only);

  std::vector<collision_detection::DistanceResultsData> collisions;
  if (dreq.active_components_only->size() > 0)
  {
    for (auto element: *dreq.active_components_only)
    {
      manager.contactTest(element, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      manager.contactTest(element.second, collisions);
    }
  }


  if (!collisions.empty())
  {
    res.collision = true;

    for (auto collision: collisions)
    {
      collision_detection::Contact c;
      const std::pair<std::string, std::string>& pc = collision.link_names[0] < collision.link_names[1] ?
                                                          std::make_pair(collision.link_names[0], collision.link_names[1]) :
                                                          std::make_pair(collision.link_names[1], collision.link_names[0]);

      c.depth = collision.distance;
      c.normal = collision.normal;
      c.body_name_1 = collision.link_names[0];
      c.body_name_2 = collision.link_names[1];

      // TODO: This needs to be update for attached objects.
      c.body_type_1 = BodyTypes::ROBOT_LINK;
      c.body_type_2 = BodyTypes::ROBOT_LINK;


      res.contacts[pc].push_back(c);
      res.contact_count++;
    }
  }

}

void collision_detection::CollisionRobotBullet::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                                         const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                                                         const AllowedCollisionMatrix* acm) const
{
  BulletManager manager(acm);
  DistanceRequest dreq;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(getRobotModel());

  manager.setContactDistance(0.0);
  constructBulletObject(manager, state1, state2, dreq.active_components_only);

  std::vector<collision_detection::DistanceResultsData> collisions;
  if (dreq.active_components_only->size() > 0)
  {
    for (auto element: *dreq.active_components_only)
    {
      manager.contactTest(element, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      manager.contactTest(element.second, collisions);
    }
  }


  if (!collisions.empty())
  {
    res.collision = true;

    for (auto collision: collisions)
    {
      collision_detection::Contact c;
      const std::pair<std::string, std::string>& pc = collision.link_names[0] < collision.link_names[1] ?
                                                          std::make_pair(collision.link_names[0], collision.link_names[1]) :
                                                          std::make_pair(collision.link_names[1], collision.link_names[0]);

      c.depth = collision.distance;
      c.normal = collision.normal;
      c.body_name_1 = collision.link_names[0];
      c.body_name_2 = collision.link_names[1];

      // TODO: This needs to be update for attached objects.
      c.body_type_1 = BodyTypes::ROBOT_LINK;
      c.body_type_2 = BodyTypes::ROBOT_LINK;


      res.contacts[pc].push_back(c);
      res.contact_count++;
    }
  }
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const robot_state::RobotState& state,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, nullptr);
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const robot_state::RobotState& state,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state,
                                                                    const AllowedCollisionMatrix& acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const robot_state::RobotState& state1,
                                                                    const robot_state::RobotState& state2,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state1,
                                                                    const robot_state::RobotState& other_state2) const
{
  CONSOLE_BRIDGE_logError("Bullet continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotBullet::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                                                    const robot_state::RobotState& state1,
                                                                    const robot_state::RobotState& state2,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state1,
                                                                    const robot_state::RobotState& other_state2,
                                                                    const AllowedCollisionMatrix& acm) const
{
  CONSOLE_BRIDGE_logError("Bullet continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotBullet::checkOtherCollisionHelper(const CollisionRequest& req,
                                                                          CollisionResult& res,
                                                                          const robot_state::RobotState& state,
                                                                          const CollisionRobot& other_robot,
                                                                          const robot_state::RobotState& other_state,
                                                                          const AllowedCollisionMatrix* acm) const
{
  CONSOLE_BRIDGE_logError("Bullet collision other checking not yet implemented");
//  FCLManager manager;
//  allocSelfCollisionBroadPhase(state, manager);

//  const CollisionRobotFCL& fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
//  FCLObject other_fcl_obj;
//  fcl_rob.constructFCLObject(other_state, other_fcl_obj);

//  CollisionData cd(&req, &res, acm);
//  cd.enableGroup(getRobotModel());
//  for (std::size_t i = 0; !cd.done_ && i < other_fcl_obj.collision_objects_.size(); ++i)
//    manager.manager_->collide(other_fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);

//  if (req.distance)
//  {
//    DistanceRequest dreq;
//    DistanceResult dres;

//    dreq.group_name = req.group_name;
//    dreq.acm = acm;
//    dreq.enableGroup(getRobotModel());
//    distanceOtherHelper(dreq, dres, state, other_robot, other_state);
//    res.distance = dres.minimum_distance.distance;
//  }
}

void collision_detection::CollisionRobotBullet::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
   CONSOLE_BRIDGE_logError("Bullet updatedPaddingOrScaling implemented");

//  std::size_t index;
//  for (const auto& link : links)
//  {
//    const robot_model::LinkModel* lmodel = robot_model_->getLinkModel(link);
//    if (lmodel)
//    {
//      for (std::size_t j = 0; j < lmodel->getShapes().size(); ++j)
//      {
//        FCLGeometryConstPtr g = createCollisionGeometry(lmodel->getShapes()[j], getLinkScale(lmodel->getName()),
//                                                        getLinkPadding(lmodel->getName()), lmodel, j);
//        if (g)
//        {
//          index = lmodel->getFirstCollisionBodyTransformIndex() + j;
//          geoms_[index] = g;
//          fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObject(g->collision_geometry_));
//        }
//      }
//    }
//    else
//      CONSOLE_BRIDGE_logError("Updating padding or scaling for unknown link: '%s'", link.c_str());
//  }
}

double collision_detection::CollisionRobotBullet::distanceSelf(const robot_state::RobotState& state) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.enableGroup(getRobotModel());
  distanceSelfHelper(dreq, dres, state);
  return dres.minimum_distance.distance;
}

double collision_detection::CollisionRobotBullet::distanceSelf(const robot_state::RobotState& state,
                                                            const AllowedCollisionMatrix& acm) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.acm = &acm;
  dreq.enableGroup(getRobotModel());
  distanceSelfHelper(dreq, dres, state);
  return dres.minimum_distance.distance;
}

void collision_detection::CollisionRobotBullet::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                                             const robot_state::RobotState& state) const
{
  distanceSelfHelper(req, res, state);
}

void collision_detection::CollisionRobotBullet::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                                             const robot_state::RobotState& state1, const robot_state::RobotState& state2) const
{
  distanceSelfHelper(req, res, state1, state2);
}

void collision_detection::CollisionRobotBullet::distanceSelfHelper(const DistanceRequest& req, DistanceResult& res,
                                                                   const robot_state::RobotState& state) const
{
  BulletManager manager(req.acm);
  constructBulletObject(manager, state, req.active_components_only);

  std::vector<collision_detection::DistanceResultsData> collisions;
  if (req.active_components_only->size() > 0)
  {
    for (auto element: *req.active_components_only)
    {
      manager.contactTest(element, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      manager.contactTest(element.second, collisions);
    }
  }

  for (auto collision: collisions)
  {
    if (collision.distance < res.minimum_distance.distance)
    {
      res.minimum_distance = collision;
    }

    if (collision.distance <= 0.0)
    {
      res.collision = true;
    }

    if (req.active_components_only->size() > 0)
    {
      if (req.active_components_only->find(state.getLinkModel(collision.link_names[0])) != req.active_components_only->end())
      {
        res.distances[collision.link_names[0]] = collision;
      }

      if (req.active_components_only->find(state.getLinkModel(collision.link_names[1])) != req.active_components_only->end())
      {
        res.distances[collision.link_names[1]] = collision;
      }
    }
    else
    {
      res.distances[collision.link_names[0]] = collision;
      res.distances[collision.link_names[1]] = collision;
    }
  }
}

void collision_detection::CollisionRobotBullet::distanceSelfHelper(const DistanceRequest& req, DistanceResult& res,
                                                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2) const
{
  BulletManager manager(req.acm);
  constructBulletObject(manager, state1, state2, req.active_components_only);

  std::vector<collision_detection::DistanceResultsData> collisions;
  if (req.active_components_only->size() > 0)
  {
    for (auto element: *req.active_components_only)
    {
      manager.contactTest(element, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      manager.contactTest(element.second, collisions);
    }
  }

  for (auto collision: collisions)
  {
    if (collision.distance < res.minimum_distance.distance)
    {
      res.minimum_distance = collision;
    }

    if (collision.distance <= 0.0)
    {
      res.collision = true;
    }

    if (req.active_components_only->size() > 0)
    {
      if (req.active_components_only->find(state1.getLinkModel(collision.link_names[0])) != req.active_components_only->end())
      {
        res.distances[collision.link_names[0]] = collision;
      }

      if (req.active_components_only->find(state1.getLinkModel(collision.link_names[1])) != req.active_components_only->end())
      {
        res.distances[collision.link_names[1]] = collision;
      }
    }
    else
    {
      res.distances[collision.link_names[0]] = collision;
      res.distances[collision.link_names[1]] = collision;
    }
  }
}

double collision_detection::CollisionRobotBullet::distanceOther(const robot_state::RobotState& state,
                                                                const CollisionRobot& other_robot,
                                                                const robot_state::RobotState& other_state) const
{
  DistanceRequest dreq;
  DistanceResult dres;

  dreq.enableGroup(getRobotModel());
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
  dreq.enableGroup(getRobotModel());
  distanceOtherHelper(dreq, dres, state, other_robot, other_state);
  return dres.minimum_distance.distance;
}

void collision_detection::CollisionRobotBullet::distanceOther(const DistanceRequest& req, DistanceResult& res,
                                                              const robot_state::RobotState& state,
                                                              const CollisionRobot& other_robot,
                                                              const robot_state::RobotState& other_state) const
{
  distanceOtherHelper(req, res, state, other_robot, other_state);
}

void collision_detection::CollisionRobotBullet::distanceOtherHelper(const DistanceRequest& req, DistanceResult& res,
                                                                    const robot_state::RobotState& state,
                                                                    const CollisionRobot& other_robot,
                                                                    const robot_state::RobotState& other_state) const
{
  CONSOLE_BRIDGE_logError("Bullet distance other checking not yet implemented");
//  FCLManager manager;
//  allocSelfCollisionBroadPhase(state, manager);

//  const CollisionRobotFCL& fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
//  FCLObject other_fcl_obj;
//  fcl_rob.constructFCLObject(other_state, other_fcl_obj);

//  DistanceData drd(&req, &res);
//  for (std::size_t i = 0; !drd.done && i < other_fcl_obj.collision_objects_.size(); ++i)
//    manager.manager_->distance(other_fcl_obj.collision_objects_[i].get(), &drd, &distanceDetailedCallback);
}
