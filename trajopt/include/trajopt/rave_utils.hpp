#pragma once
#include <openrave/openrave.h>
#include <trajopt_utils/macros.h>
#include <trajopt_utils/openrave_userdata_utils.hpp>
namespace trajopt {


namespace OR = OpenRAVE;

OpenRAVE::KinBody::LinkPtr TRAJOPT_API GetLinkMaybeAttached(OpenRAVE::RobotBasePtr robot, const std::string& name);
OpenRAVE::RobotBase::ManipulatorPtr TRAJOPT_API GetManipulatorByName(OpenRAVE::RobotBase& robot, const std::string& name);
OpenRAVE::RobotBasePtr TRAJOPT_API GetRobotByName(OpenRAVE::EnvironmentBase& env, const std::string& name);
OpenRAVE::KinBodyPtr TRAJOPT_API GetBodyByName(OpenRAVE::EnvironmentBase& env, const std::string& name);
OpenRAVE::RobotBasePtr TRAJOPT_API GetRobot(OpenRAVE::EnvironmentBase& env);
int GetRobotLinkIndex(const OR::RobotBase& robot, const OR::KinBody::Link& link);
bool DoesAffect(const OR::RobotBase& robot, const std::vector<int>& dof_inds, int link_ind);
void PlotAxes(OR::EnvironmentBase& env, const OR::Transform& T, float size, std::vector<OR::GraphHandlePtr>& handles);


}

