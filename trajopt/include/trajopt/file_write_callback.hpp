#pragma once
#include <trajopt/problem_description.hpp>

namespace trajopt
{

  Optimizer::Callback WriteCallback(string file_name,TrajOptProbPtr prob, vector<string>& joint_names,
                                    EigenSTL::vector_Affine3d pose_inverses, Eigen::Affine3d tcp = Eigen::Affine3d::Identity(),
                                    bool angle_axis = true, bool degrees = true);

}
