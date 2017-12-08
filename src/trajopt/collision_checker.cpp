#include <trajopt_ros/trajopt/collision_checker.hpp>
#include <trajopt_ros/trajopt/rave_utils.hpp>
#include <trajopt_ros/utils/eigen_conversions.hpp>
#include <trajopt_ros/utils/logging.hpp>
#include <boost/foreach.hpp>

using namespace OpenRAVE;

namespace trajopt {

#if 0
void CollisionPairIgnorer::AddExcludes(const CollisionPairIgnorer& other) {
  m_pairs.insert(other.m_pairs.begin(), other.m_pairs.end());
}
#endif

boost::shared_ptr<CollisionChecker> CollisionChecker::GetOrCreate(OR::EnvironmentBase& env) {
  UserDataPtr ud = GetUserData(env, "trajopt_cc");
  if (!ud) {
    LOG_INFO("creating bullet collision checker for environment");
    ud =  CreateCollisionChecker(env.shared_from_this());
    SetUserData(env, "trajopt_cc", ud);
  }
  else {
    LOG_DEBUG("already have a collision checker for this environment");
  }
  return boost::dynamic_pointer_cast<CollisionChecker>(ud);
}


#if 0
void CollisionPairIgnorer::ExcludePair(const KinBody::Link& link1, const KinBody::Link& link2) {

  m_pairs.insert(LinkPair(&link1, &link2));
  m_pairs.insert(LinkPair(&link2, &link1));
}
bool CollisionPairIgnorer::CanCollide(const KinBody::Link& link1, const KinBody::Link& link2) const {
  return m_pairs.find(LinkPair(&link1, &link2)) == m_pairs.end();
}
#endif

void CollisionChecker::IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body) {
  LOG_DEBUG("IgnoreZeroStateSelfCollisions for %s", body->GetName().c_str());
  KinBody::KinBodyStateSaver saver(body);
  body->SetDOFValues(DblVec(body->GetDOF(), 0));
  body->SetTransform(Transform(Vector(1,0,0,0), (Vector(0,0,10))));


  vector<Collision> collisions;
  BodyVsAll(*body,  collisions);
  LOG_DEBUG("%li extra self collisions in zero state", collisions.size());
  for(int i=0; i < collisions.size(); ++i) {
    LOG_DEBUG("ignoring self-collision: %s %s", collisions[i].linkA->GetName().c_str(), collisions[i].linkB->GetName().c_str());
    ExcludeCollisionPair(*collisions[i].linkA, *collisions[i].linkB);
  }
  LOG_DEBUG("------");
}

void CollisionChecker::IgnoreZeroStateSelfCollisions() {

  vector<KinBodyPtr> bodies;
  GetEnv()->GetBodies(bodies);

  BOOST_FOREACH(const KinBodyPtr& body, bodies) {
    IgnoreZeroStateSelfCollisions(body);
  }
}

std::ostream& operator<<(std::ostream& o, const Collision& c) {
  o << (c.linkA ? c.linkA->GetName() : "NULL") << "--" <<  (c.linkB ? c.linkB->GetName() : "NULL") <<
      " distance: " << c.distance <<
      " normal: " << c.normalB2A <<
      " ptA: " << c.ptA <<
      " ptB: " << c.ptB <<
      " time: " << c.time <<
      " weight: " << c.weight;
  return o;
}


}
