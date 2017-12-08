/// NOT YET IMPLEMENTED, BUT PLANNED FOR THE FUTURE...

#include "trajopt/collision_checker.hpp"
#include "boost/multi_array.hpp"
#include <cassert>

typedef boost::multi_array<float, 3> array_type;
typedef array_type::index index;

class  VoxelCollisionChecker : public CollisionChecker {
public:

  /** check everything vs everything else */
  virtual void AllVsAll(vector<Collision>& collisions)=0;
  /** check link vs everything else */
  virtual void LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions)=0;
  virtual void LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions)=0;

  /** contacts of distance < (arg) will be returned */
  virtual void SetContactDistance(float distance)  = 0;
  virtual double GetContactDistance() = 0;  

  ~VoxelCollisionChecker() {}
  static boost::shared_ptr<CollisionChecker> GetOrCreate(OR::EnvironmentBase& env);
};

}

/**
Approximate each link as a union of spheres
http://www.cs.mcgill.ca/~kry/pubs/cim10/cim10.pdf
First get distance field for mesh
Find medial axis points

Trilinear interpolation


*/

