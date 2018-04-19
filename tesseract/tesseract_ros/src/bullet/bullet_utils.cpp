/**
 * @file bullet_utils.cpp
 * @brief Tesseract ROS Bullet environment utility function.
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
#include "tesseract_ros/bullet/bullet_utils.h"
#include <geometric_shapes/shapes.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <octomap/octomap.h>
#include <ros/console.h>

namespace tesseract
{
namespace tesseract_ros
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
    btCollisionShape* subshape = new btBoxShape(btVector3(size[0]/2, size[1]/2, size[2]/2));
    return subshape;
  }
  case shapes::SPHERE:
  {
    const shapes::Sphere* s = static_cast<const shapes::Sphere*>(geom.get());
    btCollisionShape* subshape = new btSphereShape(s->radius);
    return subshape;
  }
  case shapes::CYLINDER:
  {
    const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(geom.get());
    btCollisionShape* subshape = new btCylinderShapeZ(btVector3(s->radius, s->radius, s->length / 2));
    return subshape;
  }
  case shapes::CONE:
  {
    const shapes::Cone* s = static_cast<const shapes::Cone*>(geom.get());
    btCollisionShape* subshape = new btConeShapeZ(s->radius, s->length);
    return subshape;
  }
  case shapes::MESH:
  {
    const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(geom.get());
    std::shared_ptr<btTriangleMesh> ptrimesh(new btTriangleMesh());
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
        return subshape;
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
            ROS_WARN("shapehull convex hull failed! falling back to original vertices");
          }
          useShapeHull = success;
        }
        else
        {
          useShapeHull = false;
        }

        btConvexHullShape *subshape = new btConvexHullShape();
        if (useShapeHull)
        {
          for (int i = 0; i < shapeHull.numVertices(); ++i)
          {
            subshape->addPoint(shapeHull.getVertexPointer()[i]);
          }
        }
        else
        {
          for (int i = 0; i < mesh->vertex_count; ++i)
          {
            subshape->addPoint(btVector3(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]));
          }
        }
        return subshape;
      }
    }
    break;
  }
  case shapes::OCTREE:
  {
    const shapes::OcTree* g = static_cast<const shapes::OcTree*>(geom.get());
    btCompoundShape* subshape = new btCompoundShape(/*dynamicAABBtree=*/false);
    double occupancy_threshold = g->octree->getOccupancyThres();

    for(auto it = g->octree->begin(g->octree->getTreeDepth()), end = g->octree->end(); it != end; ++it)
    {
      if(it->getOccupancy() >= occupancy_threshold)
      {
        double size = it.getSize();
        btTransform geomTrans;
        geomTrans.setIdentity();
        geomTrans.setOrigin(btVector3(it.getX(), it.getY(), it.getZ()));
        btBoxShape* childshape = new btBoxShape(btVector3(size/2, size/2, size/2));
        childshape->setMargin(BULLET_MARGIN);
        cow->manage(childshape);

        subshape->addChildShape(geomTrans, childshape);
      }
    }
    return subshape;
  }
  default:
    ROS_ERROR("This shape type (%d) is not supported using BULLET yet", (int)geom->type);
    return nullptr;
  }

}

CollisionObjectWrapper::CollisionObjectWrapper(const urdf::Link* link) : m_type(BodyType::ROBOT_LINK), m_index(-1)
{
  ptr.m_link = link;
  const std::vector<urdf::CollisionSharedPtr>& col_array =
        link->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, link->collision) : link->collision_array;

  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Affine3d poses;

  for (std::size_t i = 0; i < col_array.size(); ++i)
  {
    if (col_array[i] && col_array[i]->geometry)
    {
      shapes::ShapeConstPtr s = constructShape(col_array[i]->geometry.get());
      if (s)
      {
        shapes.push_back(s);
        poses.push_back(urdfPose2Affine3d(col_array[i]->origin));
      }
    }
  }

  initialize(shapes, poses);
}

CollisionObjectWrapper::CollisionObjectWrapper(const AttachedBody* ab) : m_type(BodyType::ROBOT_ATTACHED), m_index(-1)
{
  ptr.m_ab = ab;

  if (ptr.m_ab->obj->collision.shapes.empty()) return;

  initialize(ptr.m_ab->obj->collision.shapes, ptr.m_ab->obj->collision.shape_poses);
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
}
