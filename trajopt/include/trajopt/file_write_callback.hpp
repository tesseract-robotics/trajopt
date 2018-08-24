#pragma once
#include <trajopt/problem_description.hpp>


namespace trajopt
{

  Optimizer::Callback WriteCallback(std::shared_ptr<std::ofstream> file, TrajOptProbPtr prob, const vector<string>& joint_names,
                                    Eigen::Affine3d tcp = Eigen::Affine3d::Identity(),
                                    bool angle_axis = false, bool degrees = false);

}
