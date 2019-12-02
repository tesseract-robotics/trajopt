#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/program_options.hpp>
#include <console_bridge/console.h>
#include <ConvexDecomposition/cd_wavefront.h>
#include <cstdio>

#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>

#include <HACD/hacdCircularList.h>
#include <HACD/hacdGraph.h>
#include <HACD/hacdHACD.h>
#include <HACD/hacdICHull.h>
#include <HACD/hacdVector.h>
TRAJOPT_IGNORE_WARNINGS_POP

class MyConvexDecomposition
{
public:
  btAlignedObjectArray<btTriangleMesh*> trimeshes_;
  int base_count_;
  int hull_count_;
  FILE* output_file_;

  MyConvexDecomposition(FILE* output_file) : base_count_(0), hull_count_(0), output_file_(output_file) {}
  void ConvexDecompResult(size_t nVertices, float* vertices, size_t nTriangles, const unsigned int* triangles)
  {
    auto* trimesh = new btTriangleMesh();
    trimeshes_.push_back(trimesh);

    btVector3 localScaling(6.f, 6.f, 6.f);

    // export data to .obj
    CONSOLE_BRIDGE_logInform("ConvexResult #%u.", trimeshes_.size());
    if (output_file_)
    {
      std::fprintf(output_file_,
                   "## Hull Piece %d with %lu vertices and %lu triangles.\r\n",
                   hull_count_,
                   nVertices,
                   nTriangles);

      std::fprintf(output_file_, "usemtl Material%i\r\n", base_count_);
      std::fprintf(output_file_, "o Object%i\r\n", base_count_);

      for (unsigned int i = 0; i < nVertices; i++)
      {
        const float* p = &vertices[i * 3];
        std::fprintf(output_file_,
                     "v %0.9f %0.9f %0.9f\r\n",
                     static_cast<double>(p[0]),
                     static_cast<double>(p[1]),
                     static_cast<double>(p[2]));
      }

      // calc centroid, to shift vertices around center of mass
      btVector3 centroid(0, 0, 0);

      btAlignedObjectArray<btVector3> centered_vertices;
      // const unsigned int *src = result.mHullIndices;
      for (unsigned int i = 0; i < nVertices; i++)
      {
        btVector3 vertex(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
        vertex *= localScaling;
        centroid += vertex;
      }

      centroid *= 1.f / (float(nVertices));

      // const unsigned int *src = result.mHullIndices;
      for (unsigned int i = 0; i < nVertices; i++)
      {
        btVector3 vertex(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
        vertex *= localScaling;
        vertex -= centroid;
        centered_vertices.push_back(vertex);
      }

      const unsigned int* src = triangles;
      for (unsigned int i = 0; i < nTriangles; i++)
      {
        unsigned int index0 = *src++;
        unsigned int index1 = *src++;
        unsigned int index2 = *src++;

        btVector3 vertex0(vertices[index0 * 3], vertices[index0 * 3 + 1], vertices[index0 * 3 + 2]);
        btVector3 vertex1(vertices[index1 * 3], vertices[index1 * 3 + 1], vertices[index1 * 3 + 2]);
        btVector3 vertex2(vertices[index2 * 3], vertices[index2 * 3 + 1], vertices[index2 * 3 + 2]);
        vertex0 *= localScaling;
        vertex1 *= localScaling;
        vertex2 *= localScaling;

        vertex0 -= centroid;
        vertex1 -= centroid;
        vertex2 -= centroid;

        trimesh->addTriangle(vertex0, vertex1, vertex2);

        index0 += static_cast<unsigned int>(base_count_);
        index1 += static_cast<unsigned int>(base_count_);
        index2 += static_cast<unsigned int>(base_count_);

        std::fprintf(output_file_, "f %d %d %d\r\n", index0 + 1, index1 + 1, index2 + 1);
      }
    }
  }
};

namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
// const size_t ERROR_UNHANDLED_EXCEPTION = 2;
}  // namespace

int main(int argc, char** argv)
{
  std::string input;
  std::string output;

  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()("help,h", "Print help messages")("input,i",
                                                      po::value<std::string>(&input)->required(),
                                                      "File path to mesh used to create a convex decomposition of.")(
      "output,o", po::value<std::string>(&output)->required(), "File path to save the generated convex decomposition.");

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);  // can throw

    /** --help option */
    if (vm.count("help"))
    {
      CONSOLE_BRIDGE_logInform("Basic Command Line Parameter App:");
      desc.print(std::cout);
      return SUCCESS;
    }

    po::notify(vm);  // throws on error, so do after help in case
                     // there are any problems
  }
  catch (po::error& e)
  {
    CONSOLE_BRIDGE_logError(e.what());
    desc.print(std::cout);
    return ERROR_IN_COMMAND_LINE;
  }

  ConvexDecomposition::WavefrontObj wo;

  unsigned int tcount = wo.loadObj(input.c_str());
  CONSOLE_BRIDGE_logInform("WavefrontObj num triangles read %i\n", tcount);

  //-----------------------------------------------
  // HACD
  //-----------------------------------------------

  FILE* outputFile = std::fopen(output.c_str(), "wb");
  MyConvexDecomposition convexDecomposition(outputFile);

  std::vector<HACD::Vec3<HACD::Real>> points;
  std::vector<HACD::Vec3<long>> triangles;

  for (int i = 0; i < wo.mVertexCount; i++)
  {
    int index = i * 3;
    HACD::Vec3<HACD::Real> vertex(static_cast<double>(wo.mVertices[index]),
                                  static_cast<double>(wo.mVertices[index + 1]),
                                  static_cast<double>(wo.mVertices[index + 2]));
    points.push_back(vertex);
  }

  for (int i = 0; i < wo.mTriCount; i++)
  {
    int index = i * 3;
    HACD::Vec3<long> triangle(wo.mIndices[index], wo.mIndices[index + 1], wo.mIndices[index + 2]);
    triangles.push_back(triangle);
  }

  HACD::HACD myHACD;
  myHACD.SetPoints(&points[0]);
  myHACD.SetNPoints(points.size());
  myHACD.SetTriangles(&triangles[0]);
  myHACD.SetNTriangles(triangles.size());
  myHACD.SetCompacityWeight(0.1);
  myHACD.SetVolumeWeight(0.0);

  // HACD parameters
  // Recommended parameters: 2 100 0 0 0 0
  size_t nClusters = 2;
  double concavity = 0.005;
  bool addExtraDistPoints = false;
  bool addNeighboursDistPoints = false;
  bool addFacesPoints = false;

  myHACD.SetNClusters(nClusters);  // minimum number of clusters
  myHACD.SetNVerticesPerCH(100);   // max of 100 vertices per convex-hull
  myHACD.SetConcavity(concavity);  // maximum concavity
  myHACD.SetAddExtraDistPoints(addExtraDistPoints);
  myHACD.SetAddNeighboursDistPoints(addNeighboursDistPoints);
  myHACD.SetAddFacesPoints(addFacesPoints);

  myHACD.Compute();
  nClusters = myHACD.GetNClusters();

  myHACD.Save("output.wrl", false);

  btTransform trans;
  trans.setIdentity();

  for (unsigned c = 0; c < nClusters; c++)
  {
    // generate convex result
    size_t nPoints = myHACD.GetNPointsCH(c);
    size_t nTriangles = myHACD.GetNTrianglesCH(c);

    auto* vertices = new float[nPoints * 3];
    auto* triangles = new unsigned int[nTriangles * 3];

    auto* pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
    auto* trianglesCH = new HACD::Vec3<long>[nTriangles];
    myHACD.GetCH(c, pointsCH, trianglesCH);

    // points
    for (size_t v = 0; v < nPoints; v++)
    {
      vertices[3 * v] = static_cast<float>(pointsCH[v].X());
      vertices[3 * v + 1] = static_cast<float>(pointsCH[v].Y());
      vertices[3 * v + 2] = static_cast<float>(pointsCH[v].Z());
    }
    // triangles
    for (size_t f = 0; f < nTriangles; f++)
    {
      triangles[3 * f] = static_cast<unsigned int>(trianglesCH[f].X());
      triangles[3 * f + 1] = static_cast<unsigned int>(trianglesCH[f].Y());
      triangles[3 * f + 2] = static_cast<unsigned int>(trianglesCH[f].Z());
    }

    delete[] pointsCH;
    delete[] trianglesCH;

    convexDecomposition.ConvexDecompResult(nPoints, vertices, nTriangles, triangles);
  }
}
