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

const double BULLET_DEFAULT_CONTACT_DISTANCE = 0.05;
const bool   BULLET_DEFAULT_USE_ORIGINAL_CAST = false;

collision_detection::CollisionRobotBullet::CollisionRobotBullet(const robot_model::RobotModelConstPtr& model, double padding, double scale)
  : CollisionRobot(model, padding, scale)
{
  ros::NodeHandle nh("~");

  nh.param<bool>("bullet/use_original_cast", m_use_original_cast, BULLET_DEFAULT_USE_ORIGINAL_CAST);

  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  // we keep the same order of objects as what RobotState *::getLinkState() returns
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

collision_detection::CollisionRobotBullet::CollisionRobotBullet(const CollisionRobotBullet& other) : CollisionRobot(other)
{
  m_link2cow = other.m_link2cow;
  m_use_original_cast = other.m_use_original_cast;
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

void collision_detection::CollisionRobotBullet::constructBulletObject(Link2Cow &collision_objects, double contact_distance, const robot_state::RobotState& state, const std::set<const robot_model::LinkModel*> *active_links, bool continuous) const
{

  for (std::pair<std::string, COWConstPtr> element : m_link2cow)
  {
    COWPtr new_cow(new COW(*(element.second.get())));
    assert(new_cow->getCollisionShape());

    Eigen::Affine3d tf = state.getGlobalLinkTransform(element.first);
    new_cow->setWorldTransform(convertEigenToBt(tf));

    // For descrete checks we can check static to kinematic and kinematic to kinematic
    new_cow->m_collisionFilterGroup = (active_links && (std::find_if(active_links->begin(), active_links->end(), [&](const robot_model::LinkModel* link) { return link->getName() == element.first; }) == active_links->end())) ? btBroadphaseProxy::StaticFilter : btBroadphaseProxy::KinematicFilter;
    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      (continuous) ? (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter) : (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[element.first] = new_cow;
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
}

void collision_detection::CollisionRobotBullet::constructBulletObject(Link2Cow& collision_objects,
                                                                      double contact_distance,
                                                                      const robot_state::RobotState& state1,
                                                                      const robot_state::RobotState& state2,
                                                                      const std::set<const robot_model::LinkModel*> *active_links) const
{
  for (std::pair<std::string, COWConstPtr> element : m_link2cow)
  {
    COWPtr new_cow(new COW(*(element.second.get())));
//    COWPtr new_cow(new COW(element.second->m_link));
//    new_cow->m_index = element.second->m_index;

    new_cow->m_collisionFilterGroup = (active_links && (std::find_if(active_links->begin(), active_links->end(), [&](const robot_model::LinkModel* link) { return link->getName() == element.first; }) == active_links->end())) ? btBroadphaseProxy::StaticFilter : btBroadphaseProxy::KinematicFilter;

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(element.first));

      new_cow->setWorldTransform(tf1);
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
        assert(convex != NULL);

        btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(element.first));
        btTransform tf2 = convertEigenToBt(state2.getGlobalLinkTransform(element.first));

        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
        assert(shape != NULL);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape);
        new_cow->setWorldTransform(tf1);
      }
      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
        Eigen::Affine3d tf1 = state1.getGlobalLinkTransform(element.first);
        Eigen::Affine3d tf2 = state2.getGlobalLinkTransform(element.first);

        btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
          assert(convex != NULL);

          btTransform geomTrans = compound->getChildTransform(i);
          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

          Eigen::Affine3d tmp = tf1.inverse() * tf2;
          btTransform tmp2 = child_tf1.inverseTimes(child_tf2);

          btCollisionShape* subshape = new CastHullShape(convex, child_tf1.inverseTimes(child_tf2));
          assert(subshape != NULL);

          if (subshape != NULL)
          {
            new_cow->manage(subshape);
            subshape->setMargin(BULLET_MARGIN);
            new_compound->addChildShape(geomTrans, subshape);
          }
        }

        new_compound->setMargin(BULLET_MARGIN); //margin: compound. seems to have no effect when positive but has an effect when negative
        new_cow->manage(new_compound);
        new_cow->setCollisionShape(new_compound);
        new_cow->setWorldTransform(convertEigenToBt(tf1));
      }
      else
      {
        CONSOLE_BRIDGE_logError("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
      }

      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[element.first] = new_cow;
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
  BulletManager manager;
  DistanceRequest dreq;
  std::vector<collision_detection::DistanceResultsData> collisions;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(getRobotModel());

  constructBulletObject(manager.m_link2cow, 0.0, state, dreq.active_components_only);
  manager.processCollisionObjects();

  if (dreq.active_components_only->size() > 0)
  {
    for (auto element: *dreq.active_components_only)
    {
      COWPtr cow = manager.m_link2cow[element->getName()];
      manager.contactDiscreteTest(cow, acm, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      manager.contactDiscreteTest(element.second, acm, collisions);
    }
  }

  convertBulletCollisions(res, collisions);
}

void collision_detection::CollisionRobotBullet::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                                         const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                                                         const AllowedCollisionMatrix* acm) const
{
  BulletManager manager;
  std::vector<collision_detection::DistanceResultsData> collisions;
  DistanceRequest dreq;

  dreq.group_name = req.group_name;
  dreq.acm = acm;
  dreq.enableGroup(getRobotModel());

  constructBulletObject(manager.m_link2cow, 0.0, state1, dreq.active_components_only, true);
  manager.processCollisionObjects();

  if (dreq.active_components_only->size() > 0)
  {
    for (auto element: *dreq.active_components_only)
    {
      Eigen::Affine3d tf1 = state1.getGlobalLinkTransform(element);
      Eigen::Affine3d tf2 = state2.getGlobalLinkTransform(element);
      COWPtr cow = manager.m_link2cow[element->getName()];
      manager.convexSweepTest(cow, convertEigenToBt(tf1), convertEigenToBt(tf2), acm, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      Eigen::Affine3d tf1 = state1.getGlobalLinkTransform(element.first);
      Eigen::Affine3d tf2 = state2.getGlobalLinkTransform(element.first);
      manager.convexSweepTest(element.second, convertEigenToBt(tf1), convertEigenToBt(tf2), acm, collisions);
    }
  }

  convertBulletCollisions(res, collisions);
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
   CONSOLE_BRIDGE_logError("Bullet updatedPaddingOrScaling not implemented");

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
  BulletManager manager;
  std::vector<collision_detection::DistanceResultsData> collisions;

  constructBulletObject(manager.m_link2cow, req.distance_threshold, state, req.active_components_only);
  manager.processCollisionObjects();

  if (req.active_components_only->size() > 0)
  {
    for (auto element: *req.active_components_only)
    {
      COWPtr cow = manager.m_link2cow[element->getName()];
      manager.contactDiscreteTest(cow, req.acm, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      manager.contactDiscreteTest(element.second, req.acm, collisions);
    }
  }

  convertBulletCollisions(res, collisions, state, req.active_components_only);
}

void collision_detection::CollisionRobotBullet::distanceSelfHelper(const DistanceRequest& req, DistanceResult& res,
                                                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2) const
{
  BulletManager manager;
  std::vector<collision_detection::DistanceResultsData> collisions;

  constructBulletObject(manager.m_link2cow, req.distance_threshold, state1, state2, req.active_components_only);
  manager.processCollisionObjects();

  if (req.active_components_only->size() > 0)
  {
    for (auto element: *req.active_components_only)
    {
      COWPtr cow = manager.m_link2cow[element->getName()];
      manager.contactCastTest(cow, req.acm, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      manager.contactCastTest(element.second, req.acm, collisions);
    }
  }

  convertBulletCollisions(res, collisions, state1, req.active_components_only);
}

void collision_detection::CollisionRobotBullet::distanceSelfHelperOriginal(const DistanceRequest& req, DistanceResult& res,
                                                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2) const
{
  BulletManager manager;
  std::vector<collision_detection::DistanceResultsData> collisions;

  constructBulletObject(manager.m_link2cow, req.distance_threshold, state1, req.active_components_only, true);
  manager.processCollisionObjects();

  if (req.active_components_only->size() > 0)
  {
    for (auto element: *req.active_components_only)
    {
      btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(element));
      btTransform tf2 = convertEigenToBt(state2.getGlobalLinkTransform(element));

      manager.contactCastTestOriginal(element->getName(), tf1, tf2, req.acm, collisions);
    }
  }
  else
  {
    for (auto element: manager.m_link2cow)
    {
      btTransform tf1 = convertEigenToBt(state1.getGlobalLinkTransform(element.first));
      btTransform tf2 = convertEigenToBt(state2.getGlobalLinkTransform(element.first));
      manager.contactCastTestOriginal(element.first, tf1, tf2, req.acm, collisions);
    }
  }

  convertBulletCollisions(res, collisions, state1, req.active_components_only);
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
