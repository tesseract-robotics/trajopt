/**
 * @file bullet_env.cpp
 * @brief Tesseract ROS Bullet environment implementation.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "tesseract_collision/bullet/bullet_contact_checker.h"
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <limits>
#include <octomap/octomap.h>

namespace tesseract
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

void BulletContactChecker::calcDistancesDiscrete(ContactResultMap &contacts)
{
  BulletDistanceData collisions(&request_, &contacts);

  for (auto& obj : active_objects_)
  {
    COWPtr cow = manager_.m_link2cow[obj];
    assert(cow);

    manager_.contactDiscreteTest(cow, collisions);

    if (collisions.done) break;
  }
}

void BulletContactChecker::calcDistancesDiscrete(const ContactRequest &req, const TransformMap& transforms, ContactResultMap &contacts) const
{
  BulletManager manager;
  BulletDistanceData collisions(&req, &contacts);

  std::vector<std::string> active_objects;

  constructBulletObject(manager.m_link2cow, active_objects, req.contact_distance, transforms, req.link_names);

  manager.processCollisionObjects();

  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    manager.contactDiscreteTest(cow, collisions);

    if (collisions.done) break;
  }
}

void BulletContactChecker::calcDistancesContinuous(const ContactRequest &req, const TransformMap& transforms1, const TransformMap& transforms2, ContactResultMap &contacts) const
{
  BulletManager manager;
  BulletDistanceData collisions(&req, &contacts);

  std::vector<std::string> active_objects;

  constructBulletObject(manager.m_link2cow, active_objects, req.contact_distance, transforms1, transforms2, req.link_names);
  manager.processCollisionObjects();

  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    manager.contactCastTest(cow, collisions);

    if (collisions.done) break;
  }
}

void BulletContactChecker::calcCollisionsDiscrete(ContactResultMap &contacts)
{
  calcDistancesDiscrete(contacts);
}


void BulletContactChecker::calcCollisionsDiscrete(const ContactRequest &req, const TransformMap& transforms, ContactResultMap &contacts) const
{
  calcDistancesDiscrete(req, transforms, contacts);
}

void BulletContactChecker::calcCollisionsContinuous(const ContactRequest &req, const TransformMap& transforms1, const TransformMap& transforms2, ContactResultMap &contacts) const
{
  calcDistancesContinuous(req, transforms1, transforms2, contacts);
}

bool BulletContactChecker::addObject(const std::string &name, const int &mask_id, const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &shape_poses, const CollisionObjectTypeVector &collision_object_types, bool enabled)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty())
  {
    ROS_DEBUG("ignoring link %s", name.c_str());
    return false;
  }

  COWPtr new_cow(new COW(name, mask_id, shapes, shape_poses, collision_object_types));

  if (new_cow)
  {
    new_cow->m_enabled = enabled;
    setContactDistance(new_cow, BULLET_DEFAULT_CONTACT_DISTANCE);
    manager_.m_link2cow[new_cow->getName()] = new_cow;
    ROS_DEBUG("Added collision object for link %s", new_cow->getName().c_str());
    return true;
  }
  else
  {
    ROS_DEBUG("ignoring link %s", name.c_str());
    return false;
  }
}

bool BulletContactChecker::removeObject(const std::string& name)
{
  return manager_.m_link2cow.erase(name);
}

void BulletContactChecker::enableObject(const std::string& name)
{
  if (manager_.m_link2cow.find(name) != manager_.m_link2cow.end())
    manager_.m_link2cow[name]->m_enabled = true;
}

void BulletContactChecker::disableObject(const std::string& name)
{
  if (manager_.m_link2cow.find(name) != manager_.m_link2cow.end())
    manager_.m_link2cow[name]->m_enabled = false;
}

void BulletContactChecker::setObjectsTransform(const std::string& name, const Eigen::Affine3d& pose)
{
  if (manager_.m_link2cow.find(name) != manager_.m_link2cow.end())
    manager_.m_link2cow[name]->setWorldTransform(convertEigenToBt(pose));
}

void BulletContactChecker::setObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0; i < names.size(); ++i)
    setObjectsTransform(names[i], poses[i]);

}

void BulletContactChecker::setObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setObjectsTransform(transform.first, transform.second);
}

void BulletContactChecker::setContactRequest(const ContactRequest &req)
{
  request_ = req;
  active_objects_.clear();

  for (auto& element : manager_.m_link2cow)
  {
    // For descrete checks we can check static to kinematic and kinematic to kinematic
    element.second->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!request_.link_names.empty())
    {
      bool check = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) { return link == element.first; }) == req.link_names.end());
      if (check)
      {
        element.second->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (element.second->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      element.second->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects_.push_back(element.first);
      element.second->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
    }

    setContactDistance(element.second, request_.contact_distance);
  }
}

const ContactRequest& BulletContactChecker::getContactRequest() const
{
  return request_;
}

void BulletContactChecker::constructBulletObject(Link2Cow &collision_objects,
                                                 std::vector<std::string> &active_objects,
                                                 double contact_distance,
                                                 const TransformMap& transforms,
                                                 const std::vector<std::string> &active_links,
                                                 bool continuous) const
{

  for (const auto& transform : transforms)
  {
    const auto element = manager_.m_link2cow.find(transform.first);
    if (element == manager_.m_link2cow.end())
      continue;

    if (!element->second->m_enabled)
      continue;

    COWPtr new_cow(new COW(*(element->second.get())));
    assert(new_cow->getCollisionShape());


    new_cow->setWorldTransform(convertEigenToBt(transform.second));

    // For descrete checks we can check static to kinematic and kinematic to kinematic
    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!active_links.empty())
    {
      bool check = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) { return link == transform.first; }) == active_links.end());
      if (check)
      {
        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(transform.first);
      (continuous) ? (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter) : (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[transform.first] = new_cow;
  }
}

void BulletContactChecker::constructBulletObject(Link2Cow& collision_objects,
                                                 std::vector<std::string> &active_objects,
                                                 double contact_distance,
                                                 const TransformMap& transforms1,
                                                 const TransformMap& transforms2,
                                                 const std::vector<std::string> &active_links) const
{
  assert(transforms1.size() == transforms2.size());

  auto it1 = transforms1.begin();
  auto it2 = transforms2.begin();
  while (it1 != transforms1.end())
  {   
    const auto element = manager_.m_link2cow.find(it1->first);
    if (element == manager_.m_link2cow.end())
    {
      std::advance(it1, 1);
      std::advance(it2, 1);
      continue;
    }

    if (!element->second->m_enabled)
    {
      std::advance(it1, 1);
      std::advance(it2, 1);
      continue;
    }

    COWPtr new_cow(new COW(*(element->second.get())));
    assert(new_cow->getCollisionShape());
    assert(transforms2.find(it1->first) != transforms2.end());

    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!active_links.empty())
    {
      bool check = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) { return link == it1->first; }) == active_links.end());
      if (check)
      {
        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->setWorldTransform(convertEigenToBt(it1->second));
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(it1->first);

      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
        assert(convex != NULL);

        btTransform tf1 = convertEigenToBt(it1->second);
        btTransform tf2 = convertEigenToBt(it2->second);

        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
        assert(shape != NULL);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape);
        new_cow->setWorldTransform(tf1);
      }
      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
        const Eigen::Affine3d &tf1 = it1->second;
        const Eigen::Affine3d &tf2 = it2->second;

        btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
          assert(convex != NULL);

          btTransform geomTrans = compound->getChildTransform(i);
          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

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
        ROS_ERROR("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
      }

      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[it1->first] = new_cow;
    std::advance(it1, 1);
    std::advance(it2, 1);
  }
}

}
