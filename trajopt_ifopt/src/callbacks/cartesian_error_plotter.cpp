#include <trajopt_ifopt/callbacks/cartesian_error_plotter.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt/typedefs.hpp>

using namespace trajopt_sqp;

CartesianErrorPlottingCallback::CartesianErrorPlottingCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

void CartesianErrorPlottingCallback::plot(const ifopt::Problem& /*nlp*/)
{
  for (const auto& cnt : cart_position_cnts_)
  {
    Eigen::Isometry3d current_pose = cnt->GetCurrentPose();
    Eigen::Isometry3d target_pose = cnt->GetTargetPose();

    if (plotter_)
    {
      plotter_->plotAxis(current_pose, 0.05);
      plotter_->plotAxis(target_pose, 0.05);
      plotter_->plotArrow(current_pose.translation(), target_pose.translation(), Eigen::Vector4d(1, 0, 1, 1), 0.005);
    }
  }
}

void CartesianErrorPlottingCallback::addConstraintSet(const trajopt::CartPosConstraint::ConstPtr& cart_position_cnt)
{
  cart_position_cnts_.push_back(cart_position_cnt);
};

void CartesianErrorPlottingCallback::addConstraintSet(
    const std::vector<trajopt::CartPosConstraint::ConstPtr>& cart_position_cnts)
{
  for (const auto& cnt : cart_position_cnts)
    cart_position_cnts_.push_back(cnt);
}

bool CartesianErrorPlottingCallback::execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults&)
{
  plot(nlp);
  return true;
};
