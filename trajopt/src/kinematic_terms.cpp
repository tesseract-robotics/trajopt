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
VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  Affine3d new_pose, target_pose, change_base;
  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals)->transforms.at(manip_->getBaseLinkName())));
  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_, *state);
  manip_->calcFwdKin(target_pose, change_base, dof_vals, target_, *state);

  Affine3d pose_err = target_pose.inverse() * (new_pose * tcp_);
  Quaterniond q(pose_err.rotation());
  VectorXd err = concat(Vector3d(q.x(), q.y(), q.z()), pose_err.translation());
  return err;
}

void CartPoseErrorPlotter::Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x)
{
  CartPoseErrCalculator* calc = static_cast<CartPoseErrCalculator*>(m_calc.get());
  VectorXd dof_vals = getVec(x, m_vars);
  Affine3d cur_pose, target_pose, change_base;

  tesseract::EnvStateConstPtr state = calc->env_->getState();
  change_base = state->transforms.at(calc->manip_->getBaseLinkName());
  calc->manip_->calcFwdKin(cur_pose, change_base, dof_vals, calc->link_, *state);
  calc->manip_->calcFwdKin(target_pose, change_base, dof_vals, calc->target_, *state);

  cur_pose = cur_pose * calc->tcp_;

  plotter->plotAxis(cur_pose, 0.05);
  plotter->plotAxis(target_pose, 0.05);
  plotter->plotArrow(cur_pose.translation(), target_pose.translation(), Eigen::Vector4d(1, 0, 1, 1), 0.005);
}

VectorXd StaticCartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  Affine3d new_pose, change_base;
  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals)->transforms.at(manip_->getBaseLinkName())));
  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_, *state);

  // calculate the err of the current pose
  Affine3d pose_err = pose_inv_ * (new_pose * tcp_);

  // get the quaternion representation of the rotation error
  Quaterniond q(pose_err.rotation());

  // construct a 6D vector containing orientation and positional errors
  VectorXd err = concat(Vector3d(q.x(), q.y(), q.z()), pose_err.translation());
  return err;
}

void StaticCartPoseErrorPlotter::Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x)
{
  StaticCartPoseErrCalculator* calc = static_cast<StaticCartPoseErrCalculator*>(m_calc.get());
  VectorXd dof_vals = getVec(x, m_vars);
  Affine3d cur_pose, change_base;

  tesseract::EnvStateConstPtr state = calc->env_->getState();
  change_base = state->transforms.at(calc->manip_->getBaseLinkName());
  calc->manip_->calcFwdKin(cur_pose, change_base, dof_vals, calc->link_, *state);

  cur_pose = cur_pose * calc->tcp_;

  Affine3d target = calc->pose_inv_.inverse();

  plotter->plotAxis(cur_pose, 0.05);
  plotter->plotAxis(target, 0.05);
  plotter->plotArrow(cur_pose.translation(), target.translation(), Eigen::Vector4d(1, 0, 1, 1), 0.005);
}

MatrixXd CartVelJacCalculator::operator()(const VectorXd& dof_vals) const
{
  int n_dof = manip_->numJoints();
  MatrixXd out(6, 2 * n_dof);

  tesseract::EnvStateConstPtr state = env_->getState();
  Affine3d change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.topRows(n_dof))->transforms.at(manip_->getBaseLinkName())));
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.bottomRows(n_dof))->transforms.at(manip_->getBaseLinkName())));

  MatrixXd jac0, jac1;
  jac0.resize(manip_->numJoints(), 6);
  jac1.resize(manip_->numJoints(), 6);

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

VectorXd CartVelCalculator::operator()(const VectorXd& dof_vals) const
{
  int n_dof = manip_->numJoints();
  Affine3d pose0, pose1, change_base;

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

VectorXd AlignedAxisErrCalculator::operator()(const VectorXd& dof_vals) const {
  // calculate the current pose given the DOF values for the robot
  Affine3d new_pose, change_base;
  change_base = env_->getLinkTransform(manip_->getBaseLinkName());
  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_, *env_->getState());

  // get the error of the current pose
  Matrix3d orientation_err = orientation_inv_ * (new_pose.rotation() * tcp_orientation_);
  AngleAxisd aa_err(orientation_err);

  // gets terms for error determination
  double angle = fabs(aa_err.angle());

  // the angle error is typical: actual value - tolerance
  double angle_err = angle - tol_;

  // the axis error is scaled with the angle. This is for two reasons:
  // 1. The error terms will stay on the same scale, since 1 - |dot_prod| ranges from 0 to 1
  // 2. As the angle approaches zero, the axis loses meaning and should not be counted as erroneous.
  Vector3d axis_err = (axis_ - (orientation_err*axis_)).array().square();

  Vector4d err(axis_err.x(), axis_err.y(), axis_err.z(), angle_err);
  return err;
}

VectorXd ConicalAxisErrCalculator::operator()(const VectorXd& dof_vals) const {
  // calculate the current pose given the DOF values for the robot
  Affine3d new_pose, change_base;
  change_base = env_->getLinkTransform(manip_->getBaseLinkName());
  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_, *env_->getState());

  // get the orientation matrix of the error
  Matrix3d orientation_err = orientation_inv_ * (new_pose.rotation() * tcp_orientation_);
  VectorXd err(1);

  // determine the error of the conical axis
  err(0) = acos((orientation_err*axis_).dot(axis_)) - tol_;

  return err;
}

}
