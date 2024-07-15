#ifndef TRAJOPT_FWD_HPP
#define TRAJOPT_FWD_HPP

#include <cstdint>

namespace trajopt
{
// collision_terms.hpp
enum class CollisionExpressionEvaluatorType : std::uint8_t;
struct LinkGradientResults;
struct GradientResults;
struct CollisionEvaluator;

// problem_description.hpp
enum class TermType : char;
class TrajOptProb;
struct ProblemConstructionInfo;
struct TrajOptResult;
struct BasicInfo;
struct InitInfo;
struct TermInfo;
struct UserDefinedTermInfo;
struct DynamicCartPoseTermInfo;
struct CartPoseTermInfo;
struct CartVelTermInfo;
struct JointPosTermInfo;
struct JointVelTermInfo;
struct JointAccTermInfo;
struct JointJerkTermInfo;
enum class CollisionEvaluatorType : std::uint8_t;
struct CollisionTermInfo;
struct TotalTimeTermInfo;
struct AvoidSingularityTermInfo;

// Kinematic_terms.hpp
struct DynamicCartPoseErrCalculator;
struct DynamicCartPoseJacCalculator;
struct CartPoseErrCalculator;
struct CartPoseJacCalculator;
struct CartVelJacCalculator;
struct CartVelErrCalculator;
struct JointVelErrCalculator;
struct JointVelJacCalculator;
struct JointAccErrCalculator;
struct JointAccJacCalculator;
struct JointJerkErrCalculator;
struct JointJerkJacCalculator;
struct TimeCostCalculator;
struct TimeCostJacCalculator;
struct AvoidSingularityErrCalculator;
struct AvoidSingularityJacCalculator;
struct AvoidSingularitySubsetErrCalculator;
struct AvoidSingularitySubsetJacCalculator;

// trajectory_costs.hpp
class JointPosEqCost;
class JointPosIneqCost;
class JointPosEqConstraint;
class JointPosIneqConstraint;
class JointVelEqCost;
class JointVelIneqCost;
class JointVelEqConstraint;
class JointVelIneqConstraint;
class JointAccEqCost;
class JointAccIneqCost;
class JointAccEqConstraint;
class JointAccIneqConstraint;
class JointJerkEqCost;
class JointJerkIneqCost;
class JointJerkEqConstraint;
class JointJerkIneqConstraint;

}  // namespace trajopt

#endif  // TRAJOPT_FWD_HPP
