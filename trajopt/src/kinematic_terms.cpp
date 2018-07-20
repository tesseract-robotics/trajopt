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

  Affine3d pose_err = pose_inv_ * (new_pose * tcp_);
  Quaterniond q(pose_err.rotation());
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

MatrixXd CartVelJacCalculator::operator()(const VectorXd& var_vals) const
{
  int n_dof = manip_->numJoints();
  MatrixXd out = MatrixXd::Zero(6, var_vals.rows());

  tesseract::EnvStateConstPtr state = env_->getState();
  Affine3d change_base = state->transforms.at(manip_->getBaseLinkName());

  VectorXd joints0 = var_vals.topRows(n_dof);
  VectorXd joints1 = var_vals.segment(n_dof + 1, n_dof);
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), joints0)->transforms.at(manip_->getBaseLinkName())));
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), joints1)->transforms.at(manip_->getBaseLinkName())));

  MatrixXd jac0, jac1;
  jac0.resize(6, manip_->numJoints());
  jac1.resize(6, manip_->numJoints());

  if (tcp_.translation().isZero())
  {
    manip_->calcJacobian(jac0, change_base, joints0, link_, *state);
    manip_->calcJacobian(jac1, change_base, joints1, link_, *state);
  }
  else
  {
    manip_->calcJacobian(jac0, change_base, joints0, link_, *state, tcp_.translation());
    manip_->calcJacobian(jac1, change_base, joints1, link_, *state, tcp_.translation());
  }

  Affine3d pose0, pose1;
  manip_->calcFwdKin(pose0, change_base, joints0, link_, *state);
  manip_->calcFwdKin(pose1, change_base, joints1, link_, *state);

  pose0 = pose0 * tcp_;
  pose1 = pose1 * tcp_;

  double dt = var_vals(var_vals.rows() - 1);
  out.block(0, 0, 3, n_dof) = -jac0.topRows(3)/dt;
  out.block(0, n_dof + 1, 3, n_dof) = jac1.topRows(3)/dt;

  // dx/d(dt)
  out.block(0, var_vals.rows() - 1, 3, 1) =  -(pose1.translation() - pose0.translation())/sq(dt);

  // bottom half is negative of top half (Jacobian for negative velocity)
  out.block(3, 0, 3, out.cols()) = -out.block(0, 0, 3, out.cols());
  return out;
}

VectorXd CartVelCalculator::operator()(const VectorXd& var_vals) const
{
  int n_dof = manip_->numJoints();
  Affine3d pose0, pose1, change_base;

  tesseract::EnvStateConstPtr state = env_->getState();
  change_base = state->transforms.at(manip_->getBaseLinkName());

  VectorXd joints0 = var_vals.topRows(n_dof);
  VectorXd joints1 = var_vals.segment(n_dof + 1, n_dof);
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), joints0)->transforms.at(manip_->getBaseLinkName())));
  assert(change_base.isApprox(
      env_->getState(manip_->getJointNames(), joints1)->transforms.at(manip_->getBaseLinkName())));

  manip_->calcFwdKin(pose0, change_base, joints0, link_, *state);
  manip_->calcFwdKin(pose1, change_base, joints1, link_, *state);

  pose0 = pose0 * tcp_;
  pose1 = pose1 * tcp_;

  double dt = var_vals(var_vals.rows() - 1);
  VectorXd out(6);
  out.topRows(3) = (pose1.translation() - pose0.translation())/dt - Vector3d(limit_, limit_, limit_);
  out.bottomRows(3) = (pose0.translation() - pose1.translation())/dt - Vector3d(limit_, limit_, limit_);
  return out;
}

VectorXd JointVelCalculator::operator()(const VectorXd& var_vals) const
{
  int half = var_vals.rows() / 2;
  int num_vels = half - 1;
  VectorXd vel = (var_vals.segment(1, num_vels) - var_vals.segment(0, num_vels)).array()/var_vals.segment(half + 1, num_vels).array();

  VectorXd result(vel.rows() * 2);
  result.topRows(vel.rows()) = vel.array() - limit_;
  result.bottomRows(vel.rows()) = -1.0 * vel.array() - limit_;
  return result;
}

MatrixXd JointVelJacCalculator::operator()(const VectorXd& var_vals) const
{
  // var_vals = (theta_t1, theta_t2, theta_t3 ... dt_1, dt_2, dt_3 ...)
  int num_vals = var_vals.rows();
  int half = num_vals/2;
  int num_vels = half - 1;
  MatrixXd jac = MatrixXd::Zero(num_vels * 2, num_vals);

  for (int i = 0; i < num_vels; i++)
  {
    int time_index = i + half + 1;
    jac(i, i) = -1.0/var_vals(time_index);
    jac(i, i + 1) = -jac(i, i); // = 1.0/var_vals(time_index);
    jac(i, time_index) = -(var_vals(i + 1) - var_vals(i))/sq(var_vals(time_index));
  }

  // bottom half is negative velocities
  jac.bottomRows(num_vels) = -jac.topRows(num_vels);

  return jac;
}

VectorXd JointAccCalculator::operator()(const VectorXd& var_vals) const
{
  int half = var_vals.rows() / 2;
  int num_acc = half - 2;
  VectorXd vels = vel_calc(var_vals);

  VectorXd vel_diff = (vels.segment(1, num_acc) - vels.segment(0, num_acc));
  VectorXd acc = 2.0*vel_diff.array()/
      (var_vals.segment(half + 1, num_acc) +
       var_vals.segment(half + 2, num_acc)).array();

  return acc.array() - limit_;
}

MatrixXd JointAccJacCalculator::operator()(const VectorXd& var_vals) const
{
  int num_vals = var_vals.rows();
  int half = num_vals/2;
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

    jac(i, i) = 2.0*(vel_jac(i + 1, i) - vel_jac(i, i))/total_dt;
    jac(i, i + 1) = 2.0*(vel_jac(i + 1, i + 1) - vel_jac(i, i + 1))/total_dt;
    jac(i , i + 2) = 2.0*(vel_jac(i + 1, i + 2) - vel_jac(i, i + 2))/total_dt;

    jac(i, dt_1_index) = 2.0*((vel_jac(i + 1, dt_1_index) - vel_jac(i, dt_1_index))/total_dt - (vels(i + 1) - vels(i))/sq(total_dt));
    jac(i, dt_2_index) = 2.0*((vel_jac(i + 1, dt_2_index) - vel_jac(i, dt_2_index))/total_dt - (vels(i + 1) - vels(i))/sq(total_dt));
  }

  return jac;
}

VectorXd JointJerkCalculator::operator()(const VectorXd& var_vals) const
{
  int half = var_vals.rows() / 2;
  int num_jerk = half - 3;
  VectorXd acc = acc_calc(var_vals);

  VectorXd acc_diff = acc.segment(1, num_jerk) - acc.segment(0, num_jerk);

  VectorXd jerk = 3.0*acc_diff.array()/
      (var_vals.segment(half + 1, num_jerk) +
       var_vals.segment(half + 2, num_jerk) +
       var_vals.segment(half + 3, num_jerk)).array();

  return jerk.array() - limit_;

}

MatrixXd JointJerkJacCalculator::operator()(const VectorXd& var_vals) const
{
  int num_vals = var_vals.rows();
  int half = num_vals/2;
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

    jac(i, i) = 3.0*(acc_jac(i + 1, i) - acc_jac(i, i))/total_dt;
    jac(i, i + 1) = 3.0*(acc_jac(i + 1, i + 1) - acc_jac(i, i + 1))/total_dt;
    jac(i , i + 2) = 3.0*(acc_jac(i + 1, i + 2) - acc_jac(i, i + 2))/total_dt;
    jac(i , i + 3) = 3.0*(acc_jac(i + 1, i + 3) - acc_jac(i, i + 3))/total_dt;

    jac(i, dt_1_index) = 3.0*((acc_jac(i + 1, dt_1_index) - acc_jac(i, dt_1_index))/total_dt - (acc(i + 1) - acc(i))/sq(total_dt));
    jac(i, dt_2_index) = 3.0*((acc_jac(i + 1, dt_2_index) - acc_jac(i, dt_2_index))/total_dt - (acc(i + 1) - acc(i))/sq(total_dt));
    jac(i, dt_3_index) = 3.0*((acc_jac(i + 1, dt_3_index) - acc_jac(i, dt_3_index))/total_dt - (acc(i + 1) - acc(i))/sq(total_dt));
  }

  return jac;
}

VectorXd TimeCostCalculator::operator()(const VectorXd& time_vals) const
{
  VectorXd total(1);
  total(0) = time_vals.array().sum() - limit_;
  return total;
}

MatrixXd TimeCostJacCalculator::operator()(const VectorXd& time_vals) const
{
  return MatrixXd::Ones(1, time_vals.rows());
}

}
