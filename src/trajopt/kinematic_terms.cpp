#include <trajopt_ros/sco/expr_ops.hpp>
#include <trajopt_ros/sco/modeling_utils.hpp>
#include <trajopt_ros/trajopt/kinematic_terms.hpp>
#include <trajopt_ros/trajopt/rave_utils.hpp>
#include <trajopt_ros/trajopt/utils.hpp>
#include <trajopt_ros/utils/eigen_conversions.hpp>
#include <trajopt_ros/utils/eigen_slicing.hpp>
#include <trajopt_ros/utils/logging.hpp>
#include <trajopt_ros/utils/stl_to_string.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace sco;
using namespace Eigen;
using namespace util;


namespace {
  
#if 0
Vector3d rotVec(const Matrix3d& m) {
  Quaterniond q; q = m;
  return Vector3d(q.x(), q.y(), q.z());
}
#endif
inline Vector3d rotVec(const OpenRAVE::Vector& q) {
  return Vector3d(q[1], q[2], q[3]);
}

#if 0
VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}
#endif

}

namespace trajopt {


// CostPtr ConstructCost(VectorOfVectorPtr err_calc, const VarVector& vars, const VectorXd& coeffs, PenaltyType type, const string& name) {
//   return CostPtr(new CostFromErrFunc(err_calc), vars, coeffs, type, name);
// }
  
  
VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const {
  manip_->SetDOFValues(toDblVec(dof_vals));
  OR::Transform newpose = link_->GetTransform();

  OR::Transform pose_err = pose_inv_ * newpose;
  VectorXd err = concat(rotVec(pose_err.rot), toVector3d(pose_err.trans));
  return err;  
}

#if 0
CartPoseCost::CartPoseCost(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs) :
    CostFromErrFunc(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link)), vars, coeffs, ABS, "CartPose")
{}
CartPoseConstraint::CartPoseConstraint(const VarVector& vars, const OR::Transform& pose,
    RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs) :
    ConstraintFromFunc(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link)), vars, coeffs, EQ, "CartPose")
{}
#endif

void CartPoseErrorPlotter::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  CartPoseErrCalculator* calc = static_cast<CartPoseErrCalculator*>(m_calc.get());
  DblVec dof_vals = getDblVec(x, m_vars);
  calc->manip_->SetDOFValues(dof_vals);
  OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
  PlotAxes(env, cur, .05,  handles);
  PlotAxes(env, target, .05,  handles);
  handles.push_back(env.drawarrow(cur.trans, target.trans, .005, OR::Vector(1,0,1,1)));
}


#if 0
struct CartPositionErrCalculator {
  Vector3d pt_world_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  CartPositionErrCalculator(const Vector3d& pt_world, RobotAndDOFPtr manip, OR::KinBody::LinkPtr link) :
  pt_world_(pt_world),
  manip_(manip),
  link_(link)
  {}
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return pt_world_ - toVector3d(newpose.trans);
  }
};
#endif

MatrixXd CartVelJacCalculator::operator()(const VectorXd& dof_vals) const {
  int n_dof = manip_->GetDOF();
  MatrixXd out(6, 2*n_dof);
  manip_->SetDOFValues(toDblVec(dof_vals.topRows(n_dof)));
  OR::Transform pose0 = link_->GetTransform();
  MatrixXd jac0 = manip_->PositionJacobian(link_->GetIndex(), pose0.trans);
  manip_->SetDOFValues(toDblVec(dof_vals.bottomRows(n_dof)));
  OR::Transform pose1 = link_->GetTransform();
  MatrixXd jac1 = manip_->PositionJacobian(link_->GetIndex(), pose1.trans);
  out.block(0,0,3,n_dof) = -jac0;
  out.block(0,n_dof,3,n_dof) = jac1;
  out.block(3,0,3,n_dof) = jac0;
  out.block(3,n_dof,3,n_dof) = -jac1;
  return out;
}

VectorXd CartVelCalculator::operator()(const VectorXd& dof_vals) const {
  int n_dof = manip_->GetDOF();
  manip_->SetDOFValues(toDblVec(dof_vals.topRows(n_dof)));
  OR::Transform pose0 = link_->GetTransform();
  manip_->SetDOFValues(toDblVec(dof_vals.bottomRows(n_dof)));
  OR::Transform pose1 = link_->GetTransform();
  VectorXd out(6);
  out.topRows(3) = toVector3d(pose1.trans - pose0.trans - OR::Vector(limit_,limit_,limit_));
  out.bottomRows(3) = toVector3d( - pose1.trans + pose0.trans - OR::Vector(limit_, limit_, limit_));
  return out;
}


#if 0
CartVelConstraint::CartVelConstraint(const VarVector& step0vars, const VarVector& step1vars, RobotAndDOFPtr manip, KinBody::LinkPtr link, double distlimit) :
        ConstraintFromFunc(VectorOfVectorPtr(new CartVelCalculator(manip, link, distlimit)),
             MatrixOfVectorPtr(new CartVelJacCalculator(manip, link, distlimit)), concat(step0vars, step1vars), VectorXd::Ones(0), INEQ, "CartVel") 
{} // TODO coeffs
#endif

#if 0
struct UpErrorCalculator {
  Vector3d dir_local_;
  Vector3d goal_dir_world_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  MatrixXd perp_basis_; // 2x3 matrix perpendicular to goal_dir_world
  UpErrorCalculator(const Vector3d& dir_local, const Vector3d& goal_dir_world, RobotAndDOFPtr manip, KinBody::LinkPtr link) :
    dir_local_(dir_local),
    goal_dir_world_(goal_dir_world),
    manip_(manip),
    link_(link)
  {
    Vector3d perp0 = goal_dir_world_.cross(Vector3d::Random()).normalized();
    Vector3d perp1 = goal_dir_world_.cross(perp0);
    perp_basis_.resize(2,3);
    perp_basis_.row(0) = perp0.transpose();
    perp_basis_.row(1) = perp1.transpose();
  }
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return perp_basis_*(toRot(newpose.rot) * dir_local_ - goal_dir_world_);
  }
};
#endif
}
