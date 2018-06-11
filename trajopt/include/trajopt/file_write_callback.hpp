#pragma once
#include <trajopt/common.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <tesseract_core/basic_kin.h>
#include <tesseract_core/basic_env.h>
#include <trajopt/plot_callback.hpp>
#include <functional>
#include <set>
#include <fstream>
#include <string>
#include <stdio.h>
#include <Eigen/Core>

typedef std::shared_ptr<std::ofstream> ofstreamPtr;

using namespace Eigen;
using namespace util;
using namespace std;
namespace trajopt
{

  Optimizer::Callback WriteCallback(string file_name,TrajOptProbPtr prob, vector<string> joint_names,
                                    vector<Affine3d> pose_inverses, Affine3d tcp = Affine3d::Identity(), bool angle_axis = true, bool degrees = true);

}
