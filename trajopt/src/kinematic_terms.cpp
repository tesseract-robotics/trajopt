#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <iostream>
TRAJOPT_IGNORE_WARNINGS_POP

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
}  // namespace

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

void DynamicCartPoseErrCalculator::Plot(const tesseract::BasicPlottingPtr& plotter, const VectorXd& dof_vals)
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
  int n_dof = static_cast<int>(manip_->numJoints());
  MatrixXd out(6, 2 * n_dof);

  tesseract::EnvStateConstPtr state = env_->getState();
  Isometry3d change_base = state->transforms.at(manip_->getBaseLinkName());
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.topRows(n_dof))->transforms.at(manip_->getBaseLinkName())));
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), dof_vals.bottomRows(n_dof))->transforms.at(manip_->getBaseLinkName())));

  MatrixXd jac0, jac1;
  jac0.resize(6, manip_->numJoints());
  jac1.resize(6, manip_->numJoints());

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
  int n_dof = static_cast<int>(manip_->numJoints());
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

Eigen::VectorXd JointVelErrCalculator::operator()(const VectorXd& var_vals) const
{
  assert(var_vals.rows() % 2 == 0);
  // Top half of the vector are the joint values. The bottom half are the 1/dt values
  int half = static_cast<int>(var_vals.rows() / 2);
  int num_vels = half - 1;
  // (x1-x0)*(1/dt)
  VectorXd vel = (var_vals.segment(1, num_vels) - var_vals.segment(0, num_vels)).array() *
                 var_vals.segment(half + 1, num_vels).array();

  // Note that for equality terms tols are 0, so error is effectively doubled
  VectorXd result(vel.rows() * 2);
  result.topRows(vel.rows()) = -(upper_tol_ - (vel.array() - target_));
  result.bottomRows(vel.rows()) = lower_tol_ - (vel.array() - target_);
  return result;
}

MatrixXd JointVelJacCalculator::operator()(const VectorXd& var_vals) const
{
  // var_vals = (theta_t1, theta_t2, theta_t3 ... 1/dt_1, 1/dt_2, 1/dt_3 ...)
  int num_vals = static_cast<int>(var_vals.rows());
  int half = num_vals / 2;
  int num_vels = half - 1;
  MatrixXd jac = MatrixXd::Zero(num_vels * 2, num_vals);

  for (int i = 0; i < num_vels; i++)
  {
    // v = (j_i+1 - j_i)*(1/dt)
    // We calculate v with the dt from the second pt
    int time_index = i + half + 1;
    // dv_i/dj_i = -(1/dt)
    jac(i, i) = -1.0 * var_vals(time_index);
    // dv_i/dj_i+1 = (1/dt)
    jac(i, i + 1) = 1.0 * var_vals(time_index);
    // dv_i/dt_i = j_i+1 - j_i
    jac(i, time_index) = var_vals(i + 1) - var_vals(i);
    // All others are 0
  }

  // bottom half is negative velocities
  jac.bottomRows(num_vels) = -jac.topRows(num_vels);

  return jac;
}

// TODO: convert to (1/dt) and use central finite difference method
VectorXd JointAccErrCalculator::operator()(const VectorXd& var_vals) const
{
  assert(var_vals.rows() % 2 == 0);
  int half = static_cast<int>(var_vals.rows() / 2);
  int num_acc = half - 2;
  VectorXd vels = vel_calc(var_vals);

  // v1-v0
  VectorXd vel_diff = (vels.segment(1, num_acc) - vels.segment(0, num_acc));
  // I'm not sure about this. We should probably use same method we use in non time version
  // v1-v0/avg(dt1,dt0)
  VectorXd acc =
      2.0 * vel_diff.array() / (var_vals.segment(half + 1, num_acc) + var_vals.segment(half + 2, num_acc)).array();

  return acc.array() - limit_;
}

MatrixXd JointAccJacCalculator::operator()(const VectorXd& var_vals) const
{
  int num_vals = static_cast<int>(var_vals.rows());
  int half = num_vals / 2;
  MatrixXd jac = MatrixXd::Zero(half - 2, num_vals);

  VectorXd vels = vel_calc(var_vals);
  MatrixXd vel_jac = vel_jac_calc(var_vals);
  for (int i = 0; i < jac.rows(); i++)
  {
    int dt_1_index = i + half + 1;
    int dt_2_index = dt_1_index + 1;
    double dt_1 = var_vals(dt_1_index);
    double dt_2 = var_vals(dt_2_index);
    double total_dt = dt_1 + dt_2;

    jac(i, i) = 2.0 * (vel_jac(i + 1, i) - vel_jac(i, i)) / total_dt;
    jac(i, i + 1) = 2.0 * (vel_jac(i + 1, i + 1) - vel_jac(i, i + 1)) / total_dt;
    jac(i, i + 2) = 2.0 * (vel_jac(i + 1, i + 2) - vel_jac(i, i + 2)) / total_dt;

    jac(i, dt_1_index) = 2.0 * ((vel_jac(i + 1, dt_1_index) - vel_jac(i, dt_1_index)) / total_dt -
                                (vels(i + 1) - vels(i)) / sq(total_dt));
    jac(i, dt_2_index) = 2.0 * ((vel_jac(i + 1, dt_2_index) - vel_jac(i, dt_2_index)) / total_dt -
                                (vels(i + 1) - vels(i)) / sq(total_dt));
  }

  return jac;
}

// TODO: convert to (1/dt) and use central finite difference method
VectorXd JointJerkErrCalculator::operator()(const VectorXd& var_vals) const
{
  assert(var_vals.rows() % 2 == 0);
  int half = static_cast<int>(var_vals.rows() / 2);
  int num_jerk = half - 3;
  VectorXd acc = acc_calc(var_vals);

  VectorXd acc_diff = acc.segment(1, num_jerk) - acc.segment(0, num_jerk);

  VectorXd jerk = 3.0 * acc_diff.array() /
                  (var_vals.segment(half + 1, num_jerk) + var_vals.segment(half + 2, num_jerk) +
                   var_vals.segment(half + 3, num_jerk))
                      .array();

  return jerk.array() - limit_;
}

MatrixXd JointJerkJacCalculator::operator()(const VectorXd& var_vals) const
{
  int num_vals = static_cast<int>(var_vals.rows());
  int half = num_vals / 2;
  MatrixXd jac = MatrixXd::Zero(half - 3, num_vals);

  VectorXd acc = acc_calc(var_vals);
  MatrixXd acc_jac = acc_jac_calc(var_vals);

  for (int i = 0; i < jac.rows(); i++)
  {
    int dt_1_index = i + half + 1;
    int dt_2_index = dt_1_index + 1;
    int dt_3_index = dt_2_index + 1;
    double dt_1 = var_vals(dt_1_index);
    double dt_2 = var_vals(dt_2_index);
    double dt_3 = var_vals(dt_3_index);
    double total_dt = dt_1 + dt_2 + dt_3;

    jac(i, i) = 3.0 * (acc_jac(i + 1, i) - acc_jac(i, i)) / total_dt;
    jac(i, i + 1) = 3.0 * (acc_jac(i + 1, i + 1) - acc_jac(i, i + 1)) / total_dt;
    jac(i, i + 2) = 3.0 * (acc_jac(i + 1, i + 2) - acc_jac(i, i + 2)) / total_dt;
    jac(i, i + 3) = 3.0 * (acc_jac(i + 1, i + 3) - acc_jac(i, i + 3)) / total_dt;

    jac(i, dt_1_index) =
        3.0 * ((acc_jac(i + 1, dt_1_index) - acc_jac(i, dt_1_index)) / total_dt - (acc(i + 1) - acc(i)) / sq(total_dt));
    jac(i, dt_2_index) =
        3.0 * ((acc_jac(i + 1, dt_2_index) - acc_jac(i, dt_2_index)) / total_dt - (acc(i + 1) - acc(i)) / sq(total_dt));
    jac(i, dt_3_index) =
        3.0 * ((acc_jac(i + 1, dt_3_index) - acc_jac(i, dt_3_index)) / total_dt - (acc(i + 1) - acc(i)) / sq(total_dt));
  }

  return jac;
}

VectorXd TimeCostCalculator::operator()(const VectorXd& time_vals) const
{
  VectorXd total(1);
  total(0) = time_vals.cwiseInverse().sum() - limit_;
  return total;
}

MatrixXd TimeCostJacCalculator::operator()(const VectorXd& time_vals) const
{
  MatrixXd jac(1, time_vals.rows());
  jac.row(0) = -1 * time_vals.cwiseAbs2().cwiseInverse();
  return jac;
}

}  // namespace trajopt
