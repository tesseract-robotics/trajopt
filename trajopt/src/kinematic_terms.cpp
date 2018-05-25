#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <iostream>
#include <trajopt/kinematic_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace std;
using namespace sco;
using namespace Eigen;
using namespace util;

namespace
{
#if 0
Vector3d rotVec(const Matrix3d& m) {
  Quaterniond q; q = m;
  return Vector3d(q.x(), q.y(), q.z());
}
#endif

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

namespace trajopt
{
// CostPtr ConstructCost(VectorOfVectorPtr err_calc, const VarVector& vars,
// const VectorXd& coeffs, PenaltyType type, const string& name) {
//   return CostPtr(new CostFromErrFunc(err_calc), vars, coeffs, type, name);
// }

VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  Affine3d new_pose, change_base;
  change_base = env_->getLinkTransform(manip_->getBaseLinkName());
  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_);

  Affine3d pose_err = pose_inv_ * (new_pose * tcp_);
  Quaterniond q(pose_err.rotation());
  VectorXd err = concat(Vector3d(q.x(), q.y(), q.z()), pose_err.translation());
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

void CartPoseErrorPlotter::Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x)
{
  CartPoseErrCalculator* calc = static_cast<CartPoseErrCalculator*>(m_calc.get());
  VectorXd dof_vals = getVec(x, m_vars);
  Affine3d cur_pose, change_base;
  change_base = calc->env_->getLinkTransform(calc->manip_->getBaseLinkName());
  calc->manip_->calcFwdKin(cur_pose, change_base, dof_vals, calc->link_);

  cur_pose = cur_pose * calc->tcp_;

  Affine3d target = calc->pose_inv_.inverse();

  plotter->plotAxis(cur_pose, 0.05);
  plotter->plotAxis(target, 0.05);
  plotter->plotArrow(cur_pose.translation(), target.translation(), Eigen::Vector4d(1, 0, 1, 1), 0.005);
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

MatrixXd CartVelJacCalculator::operator()(const VectorXd& dof_vals) const
{
  int n_dof = manip_->numJoints();
  MatrixXd out(6, 2 * n_dof);

  Affine3d change_base = env_->getLinkTransform(manip_->getBaseLinkName());

  MatrixXd jac0, jac1;

  if (tcp_.translation().isZero())
  {
    manip_->calcJacobian(jac0, change_base, dof_vals.topRows(n_dof), link_);
    manip_->calcJacobian(jac1, change_base, dof_vals.bottomRows(n_dof), link_);
  }
  else
  {
    manip_->calcJacobian(jac0, change_base, dof_vals.topRows(n_dof), link_, tcp_.translation());
    manip_->calcJacobian(jac1, change_base, dof_vals.bottomRows(n_dof), link_, tcp_.translation());
  }

  out.block(0, 0, 3, n_dof) = -jac0.topRows(3);
  out.block(0, n_dof, 3, n_dof) = jac1.topRows(3);
  out.block(3, 0, 3, n_dof) = jac0.topRows(3);
  out.block(3, n_dof, 3, n_dof) = -jac1.topRows(3);
  return out;
}

VectorXd CartVelCalculator::operator()(const VectorXd& dof_vals) const
{
  int n_dof = manip_->numJoints();
  Affine3d pose0, pose1, change_base;
  change_base = env_->getLinkTransform(manip_->getBaseLinkName());

  manip_->calcFwdKin(pose0, change_base, dof_vals.topRows(n_dof), link_);
  manip_->calcFwdKin(pose1, change_base, dof_vals.bottomRows(n_dof), link_);

  pose0 = pose0 * tcp_;
  pose1 = pose1 * tcp_;

  VectorXd out(6);
  out.topRows(3) = (pose1.translation() - pose0.translation() - Vector3d(limit_, limit_, limit_));
  out.bottomRows(3) = (pose0.translation() - pose1.translation() - Vector3d(limit_, limit_, limit_));
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
