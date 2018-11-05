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
VectorXd DynamicCartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  Isometry3d new_pose, target_pose, change_base;
  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals)->transforms.at(manip_->getBaseLinkName())));
  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_, *state);
  manip_->calcFwdKin(target_pose, change_base, dof_vals, target_, *state);

  Isometry3d pose_err = target_pose.inverse() * (new_pose * tcp_);
  Quaterniond q(pose_err.rotation());
  VectorXd err = concat(Vector3d(q.x(), q.y(), q.z()), pose_err.translation());
  return err;
}

void DynamicCartPoseErrCalculator::Plot(const tesseract::BasicPlottingPtr &plotter, const VectorXd &dof_vals)
{
  Isometry3d cur_pose, target_pose, change_base;

  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());
  manip_->calcFwdKin(cur_pose, change_base, dof_vals, link_, *state);
  manip_->calcFwdKin(target_pose, change_base, dof_vals, target_, *state);

  cur_pose = cur_pose * tcp_;

  plotter->plotAxis(cur_pose, 0.05);
  plotter->plotAxis(target_pose, 0.05);
  plotter->plotArrow(cur_pose.translation(), target_pose.translation(), Eigen::Vector4d(1, 0, 1, 1), 0.005);
}

VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  Isometry3d new_pose, change_base;
  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals)->transforms.at(manip_->getBaseLinkName())));
  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_, *state);

  Isometry3d pose_err = pose_inv_ * (new_pose * tcp_);
  Quaterniond q(pose_err.rotation());
  VectorXd err = concat(Vector3d(q.x(), q.y(), q.z()), pose_err.translation());
  return err;
}

void CartPoseErrCalculator::Plot(const tesseract::BasicPlottingPtr& plotter, const VectorXd& dof_vals)
{
  Isometry3d cur_pose, change_base;

  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());
  manip_->calcFwdKin(cur_pose, change_base, dof_vals, link_, *state);

  cur_pose = cur_pose * tcp_;

  Isometry3d target = pose_inv_.inverse();

  plotter->plotAxis(cur_pose, 0.05);
  plotter->plotAxis(target, 0.05);
  plotter->plotArrow(cur_pose.translation(), target.translation(), Eigen::Vector4d(1, 0, 1, 1), 0.005);
}

MatrixXd CartVelJacCalculator::operator()(const VectorXd& dof_vals) const
{
  int n_dof = manip_->numJoints();
  MatrixXd out(6, 2 * n_dof);

  tesseract::EnvStateConstPtr state = env_->getState();
  Isometry3d change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.topRows(n_dof))->transforms.at(manip_->getBaseLinkName())));
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.bottomRows(n_dof))->transforms.at(manip_->getBaseLinkName())));

  MatrixXd jac0, jac1;
  jac0.resize(6,manip_->numJoints());
  jac1.resize(6,manip_->numJoints());

  if (tcp_.translation().isZero())
  {
    manip_->calcJacobian(jac0, change_base, dof_vals.topRows(n_dof), link_, *state);
    manip_->calcJacobian(jac1, change_base, dof_vals.bottomRows(n_dof), link_, *state);
  }
  else
  {
    manip_->calcJacobian(jac0, change_base, dof_vals.topRows(n_dof), link_, *state, tcp_.translation());
    manip_->calcJacobian(jac1, change_base, dof_vals.bottomRows(n_dof), link_, *state, tcp_.translation());
  }

  out.block(0, 0, 3, n_dof) = -jac0.topRows(3);
  out.block(0, n_dof, 3, n_dof) = jac1.topRows(3);
  out.block(3, 0, 3, n_dof) = jac0.topRows(3);
  out.block(3, n_dof, 3, n_dof) = -jac1.topRows(3);
  return out;
}

VectorXd CartVelErrCalculator::operator()(const VectorXd& dof_vals) const
{
  int n_dof = manip_->numJoints();
  Isometry3d pose0, pose1, change_base;

  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.topRows(n_dof))->transforms.at(manip_->getBaseLinkName())));
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.bottomRows(n_dof))->transforms.at(manip_->getBaseLinkName())));

  manip_->calcFwdKin(pose0, change_base, dof_vals.topRows(n_dof), link_, *state);
  manip_->calcFwdKin(pose1, change_base, dof_vals.bottomRows(n_dof), link_, *state);

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
