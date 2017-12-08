#pragma once
#include <trajopt_ros/trajopt/typedefs.hpp>
#include <trajopt_ros/trajopt/configuration_space.hpp>
#include <trajopt_ros/utils/macros.h>
#include <set>
#include <utility>
#include <iostream>
#include <openrave/openrave.h>

namespace trajopt {

enum CastCollisionType {
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

struct Collision {
  const OR::KinBody::Link* linkA;
  const OR::KinBody::Link* linkB;
  OR::Vector ptA, ptB, normalB2A; /* normal points from 2 to 1 */
  OR::Vector ptB1;
  double distance; /* pt1 = pt2 + normal*dist */
  float weight, time;
  CastCollisionType cctype;
  Collision(const KinBody::Link* linkA, const KinBody::Link* linkB, const OR::Vector& ptA, const OR::Vector& ptB, const OR::Vector& normalB2A, double distance, float weight=1, float time=0) :
    linkA(linkA), linkB(linkB), ptA(ptA), ptB(ptB), normalB2A(normalB2A), distance(distance), weight(weight), time(0), cctype(CCType_None) {}
};
TRAJOPT_API std::ostream& operator<<(std::ostream&, const Collision&);

enum CollisionFilterGroups {
  RobotFilter = 1,
  KinBodyFilter = 2,
};

/** 
Each CollisionChecker object has a copy of the world, so for performance, don't make too many copies  
*/ 
class TRAJOPT_API CollisionChecker : public OR::UserData {
public:

  /** check everything vs everything else */
  virtual void AllVsAll(vector<Collision>& collisions)=0;
  /** check link vs everything else */
  virtual void LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions, short filterMask)=0;
  virtual void LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions, short filterMask)=0;

  /** check robot vs everything else. includes attached bodies */
  void BodyVsAll(const KinBody& body, vector<Collision>& collisions, short filterMask=-1) {
    LinksVsAll(body.GetLinks(), collisions, filterMask);
  }
  /** contacts of distance < (arg) will be returned */
  virtual void SetContactDistance(float distance)  = 0;
  virtual double GetContactDistance() = 0;
  
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>&) {throw std::runtime_error("not implemented");}

  virtual void ContinuousCheckTrajectory(const TrajArray& traj, Configuration& rad, vector<Collision>& collisions) {throw std::runtime_error("not implemented");}
  
  /** Find contacts between swept-out shapes of robot links and everything in the environment, as robot goes from startjoints to endjoints */ 
  virtual void CastVsAll(Configuration& rad, const vector<KinBody::LinkPtr>& links, const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) {throw std::runtime_error("not implemented");}

  /** Finds all self collisions when all joints are set to zero, and ignore collisions between the colliding links */
  void IgnoreZeroStateSelfCollisions();
  void IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body);

  /** Prevent this pair of links from colliding */
  virtual void ExcludeCollisionPair(const KinBody::Link& link0, const KinBody::Link& link1) = 0;
  virtual void IncludeCollisionPair(const KinBody::Link& link0, const KinBody::Link& link1) = 0;

  /** Check whether a raycast hits the environment */
  virtual bool RayCastCollision(const OpenRAVE::Vector& point1, const OpenRAVE::Vector& point2) = 0;


  OpenRAVE::EnvironmentBaseConstPtr GetEnv() {return m_env;}

  virtual ~CollisionChecker() {}
  /** Get or create collision checker for this environment */
  static boost::shared_ptr<CollisionChecker> GetOrCreate(OR::EnvironmentBase& env);
protected:
  CollisionChecker(OpenRAVE::EnvironmentBaseConstPtr env) : m_env(env) {}
  OpenRAVE::EnvironmentBaseConstPtr m_env;
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr TRAJOPT_API CreateCollisionChecker(OR::EnvironmentBaseConstPtr env);

TRAJOPT_API void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist);

}

