#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <boost/format.hpp>
#include <iostream>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_visualization/markers/axis_marker.h>
#include <tesseract_visualization/markers/arrow_marker.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/kinematic_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt_utils/utils.hpp>

using namespace std;
using namespace sco;
using namespace util;
using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace trajopt
{
VectorXd DynamicCartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  tesseract_common::TransformMap state = manip_->calcFwdKin(dof_vals);
  Isometry3d source_tf = state[source_frame_] * source_frame_offset_;
  Isometry3d target_tf = state[target_frame_] * target_frame_offset_;

  VectorXd err = tesseract_common::calcTransformError(target_tf, source_tf);
  VectorXd reduced_err(indices_.size());
  for (int i = 0; i < indices_.size(); ++i)
    reduced_err[i] = err[indices_[i]];

  return reduced_err;  // This is available in 3.4 err(indices_, Eigen::all);
}

void DynamicCartPoseErrCalculator::Plot(const tesseract_visualization::Visualization::Ptr& plotter,
                                        const VectorXd& dof_vals)
{
  tesseract_common::TransformMap state = manip_->calcFwdKin(dof_vals);
  Eigen::Isometry3d source_tf = state[source_frame_] * source_frame_offset_;
  Eigen::Isometry3d target_tf = state[target_frame_] * target_frame_offset_;

  tesseract_visualization::AxisMarker m1(source_tf);
  m1.setScale(Eigen::Vector3d::Constant(0.05));
  plotter->plotMarker(m1);

  tesseract_visualization::AxisMarker m2(target_tf);
  m2.setScale(Eigen::Vector3d::Constant(0.05));
  plotter->plotMarker(m2);

  tesseract_visualization::ArrowMarker m3(source_tf.translation(), target_tf.translation());
  m3.material = std::make_shared<tesseract_scene_graph::Material>("cart_pose_error_material");
  m3.material->color << 1, 0, 1, 1;
  plotter->plotMarker(m3);
}

MatrixXd DynamicCartPoseJacCalculator::operator()(const VectorXd& dof_vals) const
{
  auto n_dof = static_cast<int>(manip_->numJoints());
  tesseract_common::TransformMap state = manip_->calcFwdKin(dof_vals);

  Eigen::Isometry3d source_tf = state[source_frame_] * source_frame_offset_;
  Eigen::Isometry3d target_tf = state[target_frame_] * target_frame_offset_;

  // Get the jacobian of link in the targets coordinate system
  MatrixXd jac_link =
      manip_->calcJacobian(dof_vals, manip_->getBaseLinkName(), source_frame_, source_frame_offset_.translation());
  tesseract_common::jacobianChangeBase(jac_link, target_tf.inverse());

  // Get the jacobian of the target in the targets coordinate system
  MatrixXd jac_target =
      manip_->calcJacobian(dof_vals, manip_->getBaseLinkName(), target_frame_, target_frame_offset_.translation());
  tesseract_common::jacobianChangeBase(jac_target, target_tf.inverse());
  tesseract_common::jacobianChangeRefPoint(jac_target, (target_tf.inverse() * source_tf).translation());

  MatrixXd jac0 = jac_link - jac_target;

  // Paper:
  // https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
  // The jacobian of the robot is the geometric jacobian (Je) which maps generalized velocities in
  // joint space to time derivatives of the end-effector configuration representation. It does not
  // represent the analytic jacobian (Ja) given by a partial differentiation of position and rotation
  // to generalized coordinates. Since the geometric jacobian is unique there exists a linear mapping
  // between velocities and the derivatives of the representation.
  //
  // The approach in the paper was tried but it was having issues with getting correct jacobian.
  // Must of had an error in the implementation so should revisit at another time but the approach
  // below should be sufficient and faster than numerical calculations using the err function.

  // The approach below leverages the geometric jacobian and a small step in time to approximate
  // the partial derivative of the error function. Note that the rotational portion is the only part
  // that is required to be modified per the paper.
  Isometry3d pose_err = target_tf.inverse() * source_tf;
  Eigen::Vector3d rot_err = tesseract_common::calcRotationalError(pose_err.rotation());
  for (int c = 0; c < jac0.cols(); ++c)
  {
    auto new_pose_err = util::addTwist(pose_err, jac0.col(c), 1e-5);
    Eigen::VectorXd new_rot_err = tesseract_common::calcRotationalError(new_pose_err.rotation());
    jac0.col(c).tail(3) = ((new_rot_err - rot_err) / 1e-5);
  }

  MatrixXd reduced_jac(indices_.size(), n_dof);
  for (int i = 0; i < indices_.size(); ++i)
    reduced_jac.row(i) = jac0.row(indices_[i]);

  return reduced_jac;  // This is available in 3.4 jac0(indices_, Eigen::all);
}

VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  tesseract_common::TransformMap state = manip_->calcFwdKin(dof_vals);
  Isometry3d source_tf = state[source_frame_] * source_frame_offset_;
  Isometry3d target_tf = state[target_frame_] * target_frame_offset_;

  VectorXd err;
  if (is_target_active_)
    err = tesseract_common::calcTransformError(source_tf, target_tf);
  else
    err = tesseract_common::calcTransformError(target_tf, source_tf);

  VectorXd reduced_err(indices_.size());
  for (int i = 0; i < indices_.size(); ++i)
    reduced_err[i] = err[indices_[i]];

  return reduced_err;  // This is available in 3.4 err(indices_, Eigen::all);
}

void CartPoseErrCalculator::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{
  tesseract_common::TransformMap state = manip_->calcFwdKin(dof_vals);
  Eigen::Isometry3d source_tf = state[source_frame_] * source_frame_offset_;
  Eigen::Isometry3d target_tf = state[target_frame_] * target_frame_offset_;

  tesseract_visualization::AxisMarker m1(source_tf);
  m1.setScale(Eigen::Vector3d::Constant(0.05));
  plotter->plotMarker(m1);

  tesseract_visualization::AxisMarker m2(target_tf);
  m2.setScale(Eigen::Vector3d::Constant(0.05));
  plotter->plotMarker(m2);

  tesseract_visualization::ArrowMarker m3(source_tf.translation(), target_tf.translation());
  m3.material = std::make_shared<tesseract_scene_graph::Material>("cart_pose_error_material");
  m3.material->color << 1, 0, 1, 1;
  plotter->plotMarker(m3);
}

MatrixXd CartPoseJacCalculator::operator()(const VectorXd& dof_vals) const
{
  tesseract_common::TransformMap state = manip_->calcFwdKin(dof_vals);
  Eigen::Isometry3d pose_inv{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d tf0{ Eigen::Isometry3d::Identity() };
  Eigen::MatrixXd jac0;

  if (is_target_active_)
  {
    pose_inv = (state[source_frame_] * source_frame_offset_).inverse();
    tf0 = state[target_frame_] * target_frame_offset_;
    jac0 = manip_->calcJacobian(dof_vals, manip_->getBaseLinkName(), target_frame_, target_frame_offset_.translation());
    tesseract_common::jacobianChangeBase(jac0, pose_inv);
  }
  else
  {
    pose_inv = (state[target_frame_] * target_frame_offset_).inverse();
    tf0 = state[source_frame_] * source_frame_offset_;
    jac0 = manip_->calcJacobian(dof_vals, manip_->getBaseLinkName(), source_frame_, source_frame_offset_.translation());
    tesseract_common::jacobianChangeBase(jac0, pose_inv);
  }

  // Paper:
  // https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
  // The jacobian of the robot is the geometric jacobian (Je) which maps generalized velocities in
  // joint space to time derivatives of the end-effector configuration representation. It does not
  // represent the analytic jacobian (Ja) given by a partial differentiation of position and rotation
  // to generalized coordinates. Since the geometric jacobian is unique there exists a linear mapping
  // between velocities and the derivatives of the representation.
  //
  // The approach in the paper was tried but it was having issues with getting correct jacobian.
  // Must of had an error in the implementation so should revisit at another time but the approach
  // below should be sufficient and faster than numerical calculations using the err function.

  // The approach below leverages the geometric jacobian and a small step in time to approximate
  // the partial derivative of the error function. Note that the rotational portion is the only part
  // that is required to be modified per the paper.
  Isometry3d pose_err = pose_inv * tf0;
  Eigen::Vector3d rot_err = tesseract_common::calcRotationalError(pose_err.rotation());
  for (int c = 0; c < jac0.cols(); ++c)
  {
    auto new_pose_err = util::addTwist(pose_err, jac0.col(c), 1e-5);
    Eigen::VectorXd new_rot_err = tesseract_common::calcRotationalError(new_pose_err.rotation());
    jac0.col(c).tail(3) = ((new_rot_err - rot_err) / 1e-5);
  }

  MatrixXd reduced_jac(indices_.size(), manip_->numJoints());
  for (int i = 0; i < indices_.size(); ++i)
    reduced_jac.row(i) = jac0.row(indices_[i]);

  return reduced_jac;  // This is available in 3.4 jac0(indices_, Eigen::all);
}

MatrixXd CartVelJacCalculator::operator()(const VectorXd& dof_vals) const
{
  auto n_dof = static_cast<int>(manip_->numJoints());
  MatrixXd out(6, 2 * n_dof);

  MatrixXd jac0, jac1;
  jac0.resize(6, manip_->numJoints());
  jac1.resize(6, manip_->numJoints());

  if (tcp_.translation().isZero())
  {
    jac0 = manip_->calcJacobian(dof_vals.topRows(n_dof), manip_->getBaseLinkName(), link_);
    jac1 = manip_->calcJacobian(dof_vals.bottomRows(n_dof), manip_->getBaseLinkName(), link_);
  }
  else
  {
    jac0 = manip_->calcJacobian(dof_vals.topRows(n_dof), manip_->getBaseLinkName(), link_, tcp_.translation());
    jac1 = manip_->calcJacobian(dof_vals.bottomRows(n_dof), manip_->getBaseLinkName(), link_, tcp_.translation());
  }

  out.block(0, 0, 3, n_dof) = -jac0.topRows(3);
  out.block(0, n_dof, 3, n_dof) = jac1.topRows(3);
  out.block(3, 0, 3, n_dof) = jac0.topRows(3);
  out.block(3, n_dof, 3, n_dof) = -jac1.topRows(3);
  return out;
}

VectorXd CartVelErrCalculator::operator()(const VectorXd& dof_vals) const
{
  auto n_dof = static_cast<int>(manip_->numJoints());
  tesseract_common::TransformMap state0 = manip_->calcFwdKin(dof_vals.topRows(n_dof));
  tesseract_common::TransformMap state1 = manip_->calcFwdKin(dof_vals.bottomRows(n_dof));

  Isometry3d pose0 = state0[link_] * tcp_;
  Isometry3d pose1 = state1[link_] * tcp_;

  VectorXd out(6);
  out.topRows(3) = (pose1.translation() - pose0.translation() - Vector3d(limit_, limit_, limit_));
  out.bottomRows(3) = (pose0.translation() - pose1.translation() - Vector3d(limit_, limit_, limit_));
  return out;
}

Eigen::VectorXd JointVelErrCalculator::operator()(const VectorXd& var_vals) const
{
  assert(var_vals.rows() % 2 == 0);
  // Top half of the vector are the joint values. The bottom half are the 1/dt values
  auto half = static_cast<int>(var_vals.rows() / 2);
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
  auto num_vals = static_cast<int>(var_vals.rows());
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
  auto half = static_cast<int>(var_vals.rows() / 2);
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
  auto num_vals = static_cast<int>(var_vals.rows());
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
  auto half = static_cast<int>(var_vals.rows() / 2);
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
  auto num_vals = static_cast<int>(var_vals.rows());
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

VectorXd TimeCostCalculator::operator()(const VectorXd& var_vals) const
{
  VectorXd total(1);
  total(0) = var_vals.cwiseInverse().sum() - limit_;
  return total;
}

MatrixXd TimeCostJacCalculator::operator()(const VectorXd& var_vals) const
{
  MatrixXd jac(1, var_vals.rows());
  jac.row(0) = -1 * var_vals.cwiseAbs2().cwiseInverse();
  return jac;
}

VectorXd AvoidSingularityErrCalculator::operator()(const VectorXd& var_vals) const
{
  // Calculate the SVD of the jacobian at this joint state
  MatrixXd jacobian = fwd_kin_->calcJacobian(var_vals, link_name_);
  Eigen::JacobiSVD<MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Get the U and V vectors for the smallest singular value
  double smallest_sv = svd.singularValues().tail(1)(0);
  double cost = 1.0 / (smallest_sv + lambda_);

  const static double smallest_allowable_sv = 0.1;
  const double threshold = 1.0 / (smallest_allowable_sv + lambda_);

  VectorXd err(1);
  err(0) = cost - threshold;

  return err;
}

MatrixXd AvoidSingularityJacCalculator::jacobianPartialDerivative(const VectorXd& state,
                                                                  const Eigen::MatrixXd& jacobian,
                                                                  Eigen::Index jntIdx) const
{
  // Calculate the jacobian for the given joint perturbed by some epsilon
  Eigen::VectorXd joints(state);
  double eps = eps_;
  joints(jntIdx) += eps;

  MatrixXd jacobian_increment = fwd_kin_->calcJacobian(joints, link_name_);
  return (jacobian_increment - jacobian) / eps;
}

MatrixXd AvoidSingularityJacCalculator::operator()(const VectorXd& var_vals) const
{
  MatrixXd cost_jacobian;
  cost_jacobian.resize(1, var_vals.size());

  // Calculate the SVD of the jacobian at this joint state
  MatrixXd jacobian = fwd_kin_->calcJacobian(var_vals, link_name_);
  Eigen::JacobiSVD<MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Get the U and V vectors for the smallest singular value
  double smallest_sv = svd.singularValues().tail(1)(0);
  VectorXd ui = svd.matrixU().rightCols(1);
  VectorXd vi = svd.matrixV().rightCols(1);

  // Calculate the jacobian partial derivative for each joint, perturbing it slightly
  for (Eigen::Index jntIdx = 0; jntIdx < var_vals.size(); ++jntIdx)
  {
    // J is of size (m, n); U[i] is size (m, 1); V[i] is of size (n, 1)
    // The result of { u_i^T * dJ/dx_i * v_i } will always be a single number that is dS_i/dx_i
    cost_jacobian(0, jntIdx) = (ui.transpose() * jacobianPartialDerivative(var_vals, jacobian, jntIdx) * vi)(0);
  }

  cost_jacobian *= -1.0 / std::pow(smallest_sv + lambda_, 2.0);
  return cost_jacobian;
}

VectorXd AvoidSingularitySubsetErrCalculator::operator()(const VectorXd& var_vals) const
{
  UNUSED(var_vals);
  // Get the subset of the input variable values
  VectorXd subset_var_vals(fwd_kin_->numJoints());
  assert(getSubset(superset_kin_->getJointNames(), var_vals, fwd_kin_->getJointNames(), subset_var_vals) == true);

  // Return the cost using the base class
  return AvoidSingularityErrCalculator::operator()(subset_var_vals);
}

MatrixXd AvoidSingularitySubsetJacCalculator::operator()(const VectorXd& var_vals) const
{
  UNUSED(var_vals);
  // Calculate the gradient using the subset kinematics
  VectorXd subset_var_vals(fwd_kin_->numJoints());
  assert(getSubset(superset_kin_->getJointNames(), var_vals, fwd_kin_->getJointNames(), subset_var_vals) == true);
  MatrixXd subset_jac = AvoidSingularityJacCalculator::operator()(subset_var_vals);

  // Create an all-zero gradient that is the size of the superset joints
  MatrixXd superset_jac(1, superset_kin_->numJoints());
  superset_jac.setZero();

  // Update the all-zero superset gradient with the values from the subset gradient
  VectorXd tmp(superset_kin_->numJoints());
  assert(updateFromSubset(
             superset_kin_->getJointNames(), superset_jac.row(0), fwd_kin_->getJointNames(), subset_jac.row(0), tmp) ==
         true);

  // Create the output gradient
  MatrixXd jac(1, superset_kin_->numJoints());
  jac.row(0) = tmp;

  return jac;
}

}  // namespace trajopt
