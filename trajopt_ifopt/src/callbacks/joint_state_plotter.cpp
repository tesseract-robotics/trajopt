#include <trajopt_ifopt/callbacks/joint_state_plotter.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>

using namespace trajopt_sqp;

JointStatePlottingCallback::JointStatePlottingCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

void JointStatePlottingCallback::plot(const ifopt::Problem& /*nlp*/)
{
  trajopt::TrajArray trajectory = trajopt::toTrajArray(joint_positions_);

  if (plotter_)
    plotter_->plotTrajectory(joint_positions_[0]->GetJointNames(), trajectory);
}

void JointStatePlottingCallback::addVariableSet(const trajopt::JointPosition::ConstPtr& joint_position)
{
  joint_positions_.push_back(joint_position);
};

void JointStatePlottingCallback::addVariableSet(const std::vector<trajopt::JointPosition::ConstPtr>& joint_positions)
{
  for (const auto& cnt : joint_positions)
    joint_positions_.push_back(cnt);
}
bool JointStatePlottingCallback::execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults& /*sqp_results*/)
{
  plot(nlp);
  return true;
};
