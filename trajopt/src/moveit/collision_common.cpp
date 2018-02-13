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

/* Author: Ioan Sucan, Jia Pan */

#include <trajopt/moveit/collision_common.h>
#include <geometric_shapes/shapes.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <boost/thread/mutex.hpp>
#include <memory>

namespace collision_detection
{

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom, bool useTrimesh, CollisionObjectWrapper* cow)
{

  btCollisionShape* subshape=0;

  switch (geom->type)
  {
  case shapes::BOX:
  {
    const shapes::Box* s = static_cast<const shapes::Box*>(geom.get());
    const double* size = s->size;
    subshape = new btBoxShape(btVector3(size[0]/2, size[1]/2, size[2]/2));
    break;
  }
  case shapes::SPHERE:
  {
    const shapes::Sphere* s = static_cast<const shapes::Sphere*>(geom.get());
    subshape = new btSphereShape(s->radius);
    break;
  }
  case shapes::CYLINDER:
  {
    const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(geom.get());
    subshape = new btCylinderShapeZ(btVector3(s->radius, s->radius, s->length / 2));
    break;
  }
  case shapes::CONE:
  {
    const shapes::Cone* s = static_cast<const shapes::Cone*>(geom.get());
    subshape = new btConeShapeZ(s->radius, s->length);
    break;
  }
  case shapes::MESH:
  {
    const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(geom.get());
    boost::shared_ptr<btTriangleMesh> ptrimesh(new btTriangleMesh());
    if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
    {
      for (unsigned int i = 0; i < mesh->triangle_count; ++i)
      {
        unsigned int index1 = mesh->triangles[3 * i];
        unsigned int index2 = mesh->triangles[3 * i + 1];
        unsigned int index3 = mesh->triangles[3 * i + 2];

        btVector3 v1(mesh->vertices[3 * index1], mesh->vertices[3 * index1 + 1], mesh->vertices[3 * index1 + 2]);
        btVector3 v2(mesh->vertices[3 * index2], mesh->vertices[3 * index2 + 1], mesh->vertices[3 * index2 + 2]);
        btVector3 v3(mesh->vertices[3 * index3], mesh->vertices[3 * index3 + 1], mesh->vertices[3 * index3 + 2]);

        ptrimesh->addTriangle(v1, v2, v3);

      }

      if (useTrimesh)
      {
        subshape = new btBvhTriangleMeshShape(ptrimesh.get(), true);
        cow->manage(ptrimesh);
      }
      else
      {
        // CONVEX HULL
        btConvexTriangleMeshShape convexTrimesh(ptrimesh.get());
        convexTrimesh.setMargin(BULLET_MARGIN); // margin: hull padding
        //Create a hull shape to approximate Trimesh

        bool useShapeHull;

        btShapeHull shapeHull(&convexTrimesh);
        if (mesh->vertex_count >= 50)
        {
          bool success = shapeHull.buildHull(-666); // note: margin argument not used
          if (!success)
          {
            logWarn("shapehull convex hull failed! falling back to original vertices");
          }
          useShapeHull = success;
        }
        else
        {
          useShapeHull = false;
        }

        btConvexHullShape *convexShape = new btConvexHullShape();
        subshape = convexShape;
        if (useShapeHull)
        {
          for (int i = 0; i < shapeHull.numVertices(); ++i)
            convexShape->addPoint(shapeHull.getVertexPointer()[i]);
          break;
        }
        else
        {
          for (int i = 0; i < mesh->vertex_count; ++i)
          {
            convexShape->addPoint(btVector3(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]));
          }
          break;
        }
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

CollisionObjectWrapper::CollisionObjectWrapper(const robot_model::LinkModel* link) : m_type(BodyTypes::ROBOT_LINK), m_index(-1)
{
  ptr.m_link = link;

  if (link->getShapes().empty()) return;

  m_index = link->getFirstCollisionBodyTransformIndex();
  initialize(link->getShapes(), link->getCollisionOriginTransforms());
}

CollisionObjectWrapper::CollisionObjectWrapper(const robot_state::AttachedBody* ab) : m_type(BodyTypes::ROBOT_ATTACHED), m_index(-1)
{
  ptr.m_ab = ab;

  if (ab->getShapes().empty()) return;

  m_index = ab->getAttachedLink()->getFirstCollisionBodyTransformIndex();
  initialize(ab->getShapes(), ab->getFixedTransforms());
}

CollisionObjectWrapper::CollisionObjectWrapper(const World::Object* obj) : m_type(BodyTypes::WORLD_OBJECT), m_index(-1)
{
  ptr.m_obj = obj;

  if (obj->shapes_.empty()) return;

  m_index = -1000;
  initialize(obj->shapes_, obj->shape_poses_);
}

void CollisionObjectWrapper::initialize(const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &transforms)
{
  bool useTrimesh = false;
  if (shapes.size() == 1 && transforms[0].matrix().isIdentity())
  {
    btCollisionShape* shape = createShapePrimitive(shapes[0], useTrimesh, this);
    shape->setMargin(BULLET_MARGIN);
    manage(shape);
    setCollisionShape(shape);
  }
  else
  {
    btCompoundShape* compound = new btCompoundShape(/*dynamicAABBtree=*/false);
    manage(compound);
    compound->setMargin(BULLET_MARGIN); //margin: compound. seems to have no effect when positive but has an effect when negative
    setCollisionShape(compound);

    for (std::size_t j = 0; j < shapes.size(); ++j)
    {
      btCollisionShape* subshape = createShapePrimitive(shapes[j], useTrimesh, this);
      if (subshape != NULL)
      {
        manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        btTransform geomTrans = convertEigenToBt(transforms[j]);
        compound->addChildShape(geomTrans, subshape);
      }
    }
  }

  btTransform trans;
  trans.setIdentity();
  setWorldTransform(trans);
}

}
