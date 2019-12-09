/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.


 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 disclaimer in the documentation and/or other materials provided with the distribution.

 3. The names of the contributors may not be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "vhacd/inc/vhacdMesh.h"
#include "vhacd/inc/FloatMath.h"

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include "vhacd/inc/btConvexHullComputer.h"
#include <fstream>
#include <iosfwd>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

namespace VHACD
{
Mesh::Mesh() { m_diag = 1.0; }
Mesh::~Mesh() {}
Vec3<double>& Mesh::ComputeCenter(void)
{
  const size_t nV = GetNPoints();
  if (nV)
  {
    double center[3];
    uint32_t pcount = uint32_t(GetNPoints());
    const double* points = GetPoints();
    uint32_t tcount = uint32_t(GetNTriangles());
    const uint32_t* indices = reinterpret_cast<const uint32_t*>(GetTriangles());
    FLOAT_MATH::fm_computeCentroid(pcount, points, tcount, indices, center);
    m_center.X() = center[0];
    m_center.Y() = center[1];
    m_center.Z() = center[2];
    m_minBB = GetPoint(0);
    m_maxBB = GetPoint(0);
    for (size_t v = 1; v < nV; v++)
    {
      Vec3<double> p = GetPoint(v);
      if (p.X() < m_minBB.X())
      {
        m_minBB.X() = p.X();
      }
      if (p.Y() < m_minBB.Y())
      {
        m_minBB.Y() = p.Y();
      }
      if (p.Z() < m_minBB.Z())
      {
        m_minBB.Z() = p.Z();
      }
      if (p.X() > m_maxBB.X())
      {
        m_maxBB.X() = p.X();
      }
      if (p.Y() > m_maxBB.Y())
      {
        m_maxBB.Y() = p.Y();
      }
      if (p.Z() > m_maxBB.Z())
      {
        m_maxBB.Z() = p.Z();
      }
    }
  }
  return m_center;
}

double Mesh::ComputeVolume() const
{
  const size_t nV = GetNPoints();
  const size_t nT = GetNTriangles();
  if (nV == 0 || nT == 0)
  {
    return 0.0;
  }

  Vec3<double> bary(0.0, 0.0, 0.0);
  for (size_t v = 0; v < nV; v++)
  {
    bary += GetPoint(v);
  }
  bary /= static_cast<double>(nV);

  Vec3<double> ver0, ver1, ver2;
  double totalVolume = 0.0;
  for (size_t t = 0; t < nT; t++)
  {
    const Vec3<int32_t>& tri = GetTriangle(t);
    ver0 = GetPoint(static_cast<size_t>(tri[0]));
    ver1 = GetPoint(static_cast<size_t>(tri[1]));
    ver2 = GetPoint(static_cast<size_t>(tri[2]));
    totalVolume += ComputeVolume4(ver0, ver1, ver2, bary);
  }
  return totalVolume / 6.0;
}

void Mesh::ComputeConvexHull(const double* const pts, const size_t nPts)
{
  ResizePoints(0);
  ResizeTriangles(0);
  btConvexHullComputer ch;
  ch.compute(pts, 3 * sizeof(double), static_cast<int32_t>(nPts), -1.0, -1.0);
  for (int32_t v = 0; v < ch.vertices.size(); v++)
  {
    AddPoint(Vec3<double>(static_cast<double>(ch.vertices[v].getX()),
                          static_cast<double>(ch.vertices[v].getY()),
                          static_cast<double>(ch.vertices[v].getZ())));
  }
  const int32_t nt = ch.faces.size();
  for (int32_t t = 0; t < nt; ++t)
  {
    const btConvexHullComputer::Edge* sourceEdge = &(ch.edges[ch.faces[t]]);
    int32_t a = sourceEdge->getSourceVertex();
    int32_t b = sourceEdge->getTargetVertex();
    const btConvexHullComputer::Edge* edge = sourceEdge->getNextEdgeOfFace();
    int32_t c = edge->getTargetVertex();
    while (c != a)
    {
      AddTriangle(Vec3<int32_t>(a, b, c));
      edge = edge->getNextEdgeOfFace();
      b = c;
      c = edge->getTargetVertex();
    }
  }
}
void Mesh::Clip(const Plane& plane, SArray<Vec3<double> >& positivePart, SArray<Vec3<double> >& negativePart) const
{
  const size_t nV = GetNPoints();
  if (nV == 0)
  {
    return;
  }
  double d;
  for (size_t v = 0; v < nV; v++)
  {
    const Vec3<double>& pt = GetPoint(v);
    d = plane.m_a * pt[0] + plane.m_b * pt[1] + plane.m_c * pt[2] + plane.m_d;
    if (d > 0.0)
    {
      positivePart.PushBack(pt);
    }
    else if (d < 0.0)
    {
      negativePart.PushBack(pt);
    }
    else
    {
      positivePart.PushBack(pt);
      negativePart.PushBack(pt);
    }
  }
}
bool Mesh::IsInside(const Vec3<double>& pt) const
{
  const size_t nV = GetNPoints();
  const size_t nT = GetNTriangles();
  if (nV == 0 || nT == 0)
  {
    return false;
  }
  Vec3<double> ver0, ver1, ver2;
  double volume;
  for (size_t t = 0; t < nT; t++)
  {
    const Vec3<int32_t>& tri = GetTriangle(t);
    ver0 = GetPoint(static_cast<size_t>(tri[0]));
    ver1 = GetPoint(static_cast<size_t>(tri[1]));
    ver2 = GetPoint(static_cast<size_t>(tri[2]));
    volume = ComputeVolume4(ver0, ver1, ver2, pt);
    if (volume < 0.0)
    {
      return false;
    }
  }
  return true;
}
double Mesh::ComputeDiagBB()
{
  const size_t nPoints = GetNPoints();
  if (nPoints == 0)
    return 0.0;
  Vec3<double> minBB = m_points[0];
  Vec3<double> maxBB = m_points[0];
  double x, y, z;
  for (size_t v = 1; v < nPoints; v++)
  {
    x = m_points[v][0];
    y = m_points[v][1];
    z = m_points[v][2];
    if (x < minBB[0])
      minBB[0] = x;
    else if (x > maxBB[0])
      maxBB[0] = x;
    if (y < minBB[1])
      minBB[1] = y;
    else if (y > maxBB[1])
      maxBB[1] = y;
    if (z < minBB[2])
      minBB[2] = z;
    else if (z > maxBB[2])
      maxBB[2] = z;
  }
  return (m_diag = (maxBB - minBB).GetNorm());
}

#ifdef VHACD_DEBUG_MESH
bool Mesh::SaveVRML2(const std::string& fileName) const
{
  std::ofstream fout(fileName.c_str());
  if (fout.is_open())
  {
    const Material material;

    if (SaveVRML2(fout, material))
    {
      fout.close();
      return true;
    }
    return false;
  }
  return false;
}
bool Mesh::SaveVRML2(std::ofstream& fout, const Material& material) const
{
  if (fout.is_open())
  {
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.setf(std::ios::showpoint);
    fout.precision(6);
    size_t nV = m_points.Size();
    size_t nT = m_triangles.Size();
    fout << "#VRML V2.0 utf8" << std::endl;
    fout << "" << std::endl;
    fout << "# Vertices: " << nV << std::endl;
    fout << "# Triangles: " << nT << std::endl;
    fout << "" << std::endl;
    fout << "Group {" << std::endl;
    fout << "    children [" << std::endl;
    fout << "        Shape {" << std::endl;
    fout << "            appearance Appearance {" << std::endl;
    fout << "                material Material {" << std::endl;
    fout << "                    diffuseColor " << material.m_diffuseColor[0] << " " << material.m_diffuseColor[1]
         << " " << material.m_diffuseColor[2] << std::endl;
    fout << "                    ambientIntensity " << material.m_ambientIntensity << std::endl;
    fout << "                    specularColor " << material.m_specularColor[0] << " " << material.m_specularColor[1]
         << " " << material.m_specularColor[2] << std::endl;
    fout << "                    emissiveColor " << material.m_emissiveColor[0] << " " << material.m_emissiveColor[1]
         << " " << material.m_emissiveColor[2] << std::endl;
    fout << "                    shininess " << material.m_shininess << std::endl;
    fout << "                    transparency " << material.m_transparency << std::endl;
    fout << "                }" << std::endl;
    fout << "            }" << std::endl;
    fout << "            geometry IndexedFaceSet {" << std::endl;
    fout << "                ccw TRUE" << std::endl;
    fout << "                solid TRUE" << std::endl;
    fout << "                convex TRUE" << std::endl;
    if (nV > 0)
    {
      fout << "                coord DEF co Coordinate {" << std::endl;
      fout << "                    point [" << std::endl;
      for (size_t v = 0; v < nV; v++)
      {
        fout << "                        " << m_points[v][0] << " " << m_points[v][1] << " " << m_points[v][2] << ","
             << std::endl;
      }
      fout << "                    ]" << std::endl;
      fout << "                }" << std::endl;
    }
    if (nT > 0)
    {
      fout << "                coordIndex [ " << std::endl;
      for (size_t f = 0; f < nT; f++)
      {
        fout << "                        " << m_triangles[f][0] << ", " << m_triangles[f][1] << ", "
             << m_triangles[f][2] << ", -1," << std::endl;
      }
      fout << "                ]" << std::endl;
    }
    fout << "            }" << std::endl;
    fout << "        }" << std::endl;
    fout << "    ]" << std::endl;
    fout << "}" << std::endl;
    return true;
  }
  return false;
}
bool Mesh::SaveOFF(const std::string& fileName) const
{
  std::ofstream fout(fileName.c_str());
  if (fout.is_open())
  {
    size_t nV = m_points.Size();
    size_t nT = m_triangles.Size();
    fout << "OFF" << std::endl;
    fout << nV << " " << nT << " " << 0 << std::endl;
    for (size_t v = 0; v < nV; v++)
    {
      fout << m_points[v][0] << " " << m_points[v][1] << " " << m_points[v][2] << std::endl;
    }
    for (size_t f = 0; f < nT; f++)
    {
      fout << "3 " << m_triangles[f][0] << " " << m_triangles[f][1] << " " << m_triangles[f][2] << std::endl;
    }
    fout.close();
    return true;
  }
  return false;
}

bool Mesh::LoadOFF(const std::string& fileName, bool invert)
{
  FILE* fid = fopen(fileName.c_str(), "r");
  if (fid)
  {
    const std::string strOFF("OFF");
    char temp[1024];
    if (fscanf(fid, "%s", temp) != 1)
      return false;

    if (std::string(temp) != strOFF)
    {
      fclose(fid);
      return false;
    }

    int32_t nv = 0;
    int32_t nf = 0;
    int32_t ne = 0;
    if (fscanf(fid, "%i", &nv) != 1)
      return false;

    if (fscanf(fid, "%i", &nf) != 1)
      return false;

    if (fscanf(fid, "%i", &ne) != 1)
      return false;

    m_points.Resize(static_cast<size_t>(nv));
    m_triangles.Resize(static_cast<size_t>(nf));
    Vec3<double> coord;
    float x, y, z;
    for (size_t p = 0; p < static_cast<size_t>(nv); p++)
    {
      if (fscanf(fid, "%f", &x) != 1)
        return false;

      if (fscanf(fid, "%f", &y) != 1)
        return false;

      if (fscanf(fid, "%f", &z) != 1)
        return false;

      m_points[p][0] = static_cast<double>(x);
      m_points[p][1] = static_cast<double>(y);
      m_points[p][2] = static_cast<double>(z);
    }
    int32_t i, j, k, s;
    for (size_t t = 0; t < static_cast<size_t>(nf); ++t)
    {
      if (fscanf(fid, "%i", &s) != 1)
        return false;

      if (s == 3)
      {
        if (fscanf(fid, "%i", &i) != 1)
          return false;

        if (fscanf(fid, "%i", &j) != 1)
          return false;

        if (fscanf(fid, "%i", &k) != 1)
          return false;

        m_triangles[t][0] = i;
        if (invert)
        {
          m_triangles[t][1] = k;
          m_triangles[t][2] = j;
        }
        else
        {
          m_triangles[t][1] = j;
          m_triangles[t][2] = k;
        }
      }
      else  // Fix me: support only triangular meshes
      {
        for (int32_t h = 0; h < s; ++h)
          if (fscanf(fid, "%i", &s) != 1)
            return false;
      }
    }
    fclose(fid);
  }
  else
  {
    return false;
  }
  return true;
}
#endif  // VHACD_DEBUG_MESH
}  // namespace VHACD
