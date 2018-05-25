#ifndef TESSERACT_PLANNING_CHAIN_OMPL_INTERFACE_H
#define TESSERACT_PLANNING_CHAIN_OMPL_INTERFACE_H

#include <ompl/geometric/SimpleSetup.h>
#include <tesseract_ros/kdl/kdl_env.h>

namespace tesseract
{
namespace tesseract_planning
{
struct OmplPlanParameters
{
  double planning_time = 5.0;
  bool simplify = true;
};

class ChainOmplInterface
{
public:
  ChainOmplInterface(tesseract::BasicEnvConstPtr environment, const std::string& manipulator_name);

  boost::optional<ompl::geometric::PathGeometric> plan(ompl::base::PlannerPtr planner,
                                                       const std::vector<double>& from,
                                                       const std::vector<double>& to,
                                                       const OmplPlanParameters& params);

  ompl::base::SpaceInformationPtr spaceInformation();

  void setMotionValidator(ompl::base::MotionValidatorPtr mv)
  {
    ss_->getSpaceInformation()->setMotionValidator(std::move(mv));
  }

private:
  bool isStateValid(const ompl::base::State* state) const;

  bool isContactAllowed(const std::string& a, const std::string& b) const;

private:
  ompl::geometric::SimpleSetupPtr ss_;
  tesseract::BasicEnvConstPtr env_;
  tesseract::IsContactAllowedFn contact_fn_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
};
}
}

#endif
