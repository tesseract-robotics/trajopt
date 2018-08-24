#include <functional>
#include <fstream>
#include <trajopt/file_write_callback.hpp>
#include <ros/ros.h>

using namespace Eigen;
using namespace util;
using namespace std;
namespace trajopt
{

  void WriteFile(shared_ptr<ofstream> file, Affine3d tcp, TrajOptProbPtr prob, DblVec& x,
                 bool angle_axis, bool degrees)
  {
    auto env = prob->GetEnv();
    auto manip = prob->GetKin();
    auto change_base = env->getLinkTransform(manip->getBaseLinkName());

    // get joint angles and write to file
    TrajArray traj = getTraj(x, prob->GetVars());
    for (auto i = 0; i < traj.rows(); i++) {
      VectorXd joint_angles(traj.cols());
      for (auto j = 0; j < traj.cols(); j++) {
        if (j != 0) {
          *file << ',';
        }
        *file << traj(i,j);

        joint_angles(j) = traj(i,j);
      }

      //get pose
      Affine3d pose;
      manip->calcFwdKin(pose, change_base, joint_angles);
      pose = tcp * pose;

      Vector4d rot_vec;
      if (angle_axis) {
        AngleAxisd aa(pose.rotation());
        Vector3d axis = aa.axis();
        rot_vec(0) = degrees ? aa.angle() * 180.0/M_PI : aa.angle();
        rot_vec(1) = axis(0);
        rot_vec(2) = axis(1);
        rot_vec(3) = axis(2);

      }
      else {
        Quaterniond q(pose.rotation());
        rot_vec(0) = q.w();
        rot_vec(1) = q.x();
        rot_vec(2) = q.y();
        rot_vec(3) = q.z();
      }

      // write to file
      VectorXd pose_vec = concat(pose.translation(), rot_vec);
      for (auto i = 0;i < pose_vec.size(); i++) {
        *file << ',' << pose_vec(i);
      }

      *file << endl;
    }
    *file << endl;
  }

  Optimizer::Callback WriteCallback(shared_ptr<ofstream> file, TrajOptProbPtr prob, const vector<string>& joint_names,
                                    Affine3d tcp /*=identity*/, bool angle_axis /*=false*/, bool degrees /*=false*/)
  {
    if (!file->good())
    {
      ROS_WARN("ofstream passed to create callback not in 'good' state");
    }

    for (size_t i = 0; i < joint_names.size(); i++) {
      if (i != 0) {
        *file << ',';
      }
      *file << joint_names.at(i);
    }

    vector<string> pose_str = angle_axis ? vector<string>{"x", "y", "z", "angle", "axis_x", "axis_y", "axis_z"} :
                                      vector<string>{"x", "y", "z", "q_w", "q_x", "q_y", "q_z"};

    for (size_t i = 0; i < pose_str.size(); i++) {
      *file << ',' << pose_str.at(i);
    }

    *file << endl;

    // return callback function
    return bind(&WriteFile,
                file,
                tcp,
                prob,
                placeholders::_2,
                angle_axis,
                degrees);
  }

}
