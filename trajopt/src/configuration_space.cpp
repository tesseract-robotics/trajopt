#include <trajopt/configuration_space.hpp>
#include <trajopt/rave_utils.hpp>
#include <trajopt_utils/math.hpp>
#include <boost/foreach.hpp>
using namespace OpenRAVE;
using namespace util;

// TODO: configuration should know something about what dofs are part of SO(1), SO(2) etc for functions like RandomDOFValues

namespace trajopt {

void RobotAndDOF::SetDOFValues(const DblVec& dofs) {
  if (affinedofs != 0) {
    OR::Transform T;
    OR::RaveGetTransformFromAffineDOFValues(T, dofs.begin()+joint_inds.size(), affinedofs, rotationaxis, true);
    robot->SetTransform(T);
    if (joint_inds.size() > 0)
      robot->SetDOFValues(dofs, false, joint_inds);
  }
  else {
    robot->SetDOFValues(dofs, false, joint_inds);
  }
}

DblVec RobotAndDOF::GetDOFValues() {
  DblVec out;
  if (joint_inds.size() > 0)
    robot->GetDOFValues(out, joint_inds);
  if (affinedofs != 0) {
    out.resize(GetDOF());
    OR::Transform T = robot->GetTransform();
    OR::RaveGetAffineDOFValuesFromTransform(out.begin() + joint_inds.size(), T, affinedofs, rotationaxis);
  }
  return out;
}

void RobotAndDOF::SetRobotActiveDOFs()  {
  RobotBasePtr robot1 = boost::dynamic_pointer_cast<RobotBase>(robot); // since robot is just a kinbody
  vector<int> current_active = robot1->GetActiveDOFIndices();
  if (robot1->GetActiveDOF() != GetDOF() || !std::equal(current_active.begin(), current_active.end(), joint_inds.begin()))
    robot1->SetActiveDOFs(joint_inds, affinedofs);
}

void RobotAndDOF::GetDOFLimits(DblVec& lower, DblVec& upper) const {
  if (joint_inds.size() > 0)
    robot->GetDOFLimits(lower, upper, joint_inds);
  const int translation_dofs[3] = {DOF_X, DOF_Y, DOF_Z};
  for (int i=0; i < 3; ++i) {
    if (affinedofs & translation_dofs[i]) {
      lower.push_back(-INFINITY);
      upper.push_back(INFINITY);
    }
  }
  if (affinedofs & DOF_RotationMask) {
    if (affinedofs & DOF_RotationAxis) {
      lower.push_back(-INFINITY);
      upper.push_back(INFINITY);
    }
    else if (affinedofs & DOF_Rotation3D) {
      for (int i=0; i < 3; ++i) {
        lower.push_back(-INFINITY);
        upper.push_back(INFINITY);
      }
    }
    else if (affinedofs & DOF_Rotation3D) {
      for (int i=0; i < 3; ++i) {
        lower.push_back(-INFINITY);
        upper.push_back(INFINITY);
      }
    }
    else if (affinedofs & DOF_RotationQuat) {
      for (int i=0; i < 4; ++i) {
        lower.push_back(-1);
        upper.push_back(1);
      }
    }
    else throw OR::openrave_exception("invalid rotation dofs", ORE_InvalidArguments);
  }
}
int RobotAndDOF::GetDOF() const {
  return joint_inds.size() + RaveGetAffineDOF(affinedofs);
}
DblMatrix RobotAndDOF::PositionJacobian(int link_ind, const OR::Vector& pt) const {
  Configuration::SaverPtr saver = const_cast<RobotAndDOF*>(this)->Save();
  const_cast<RobotAndDOF*>(this)->SetRobotActiveDOFs();
  vector<double> jacdata;
  boost::dynamic_pointer_cast<RobotBase>(robot)->CalculateActiveJacobian(link_ind, pt, jacdata);
  return Eigen::Map<DblMatrix>(jacdata.data(), 3, GetDOF());
}
DblMatrix RobotAndDOF::RotationJacobian(int link_ind) const {
  Configuration::SaverPtr saver = const_cast<RobotAndDOF*>(this)->Save();
  const_cast<RobotAndDOF*>(this)->SetRobotActiveDOFs();
  vector<double> jacdata;
  boost::dynamic_pointer_cast<RobotBase>(robot)->ComputeJacobianAxisAngle(link_ind, jacdata);
  return Eigen::Map<DblMatrix>(jacdata.data(), 3, GetDOF());  
}
bool RobotAndDOF::DoesAffect(const KinBody::Link& link) {
  if (affinedofs > 0) return true;
  else if (link.GetParent() == GetRobot()) return trajopt::DoesAffect(*GetRobot(), joint_inds, GetRobotLinkIndex(*GetRobot(), link));
  else return false;
}

std::vector<KinBody::LinkPtr> RobotAndDOF::GetAffectedLinks() {
  vector<int> inds;
  std::vector<KinBody::LinkPtr> out;
  GetAffectedLinks(out,false,inds);
  return out;
}

void RobotAndDOF::GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
  links.clear();
  link_inds.clear();
  BOOST_FOREACH(const KinBody::LinkPtr& link, GetRobot()->GetLinks()) {
    if (this->DoesAffect(*link) && !(only_with_geom && link->GetGeometries().empty())) {
      links.push_back(link);
      link_inds.push_back(link->GetIndex());
    }
  }

  vector<KinBodyPtr> grabbed;
  boost::dynamic_pointer_cast<RobotBase>(robot)->GetGrabbed(grabbed);
  BOOST_FOREACH(const KinBodyPtr& body, grabbed) {
    KinBody::LinkPtr grabberLink = boost::dynamic_pointer_cast<RobotBase>(robot)->IsGrabbing(body);
    assert(grabberLink);
    BOOST_FOREACH(const KinBody::LinkPtr& link, body->GetLinks()) {
      if (link->GetGeometries().size() > 0) {
        links.push_back(link);
        link_inds.push_back(grabberLink->GetIndex());
      }
    }
  }

}

vector<KinBodyPtr> RobotAndDOF::GetBodies() {
  std::set<KinBodyPtr> bodies;
  robot->GetAttached(bodies);
  return vector<KinBodyPtr> (bodies.begin(), bodies.end());
} 


DblVec RobotAndDOF::RandomDOFValues() {
  int ndof = GetDOF();
  DblVec lower, upper;
  GetDOFLimits(lower, upper);
  DblVec out(ndof);
  for (int i=0; i < ndof; ++i) {
    // TODO this is only right for a circular joint!!
    lower[i] = fmax(lower[i], -2*M_PI);
    upper[i] = fmin(upper[i], 2*M_PI);
    out[i] = lower[i] + randf() * (upper[i] - lower[i]);
  }
  return out;
}



}
