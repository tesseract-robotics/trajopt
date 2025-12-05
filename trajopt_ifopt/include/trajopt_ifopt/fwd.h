/**
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TRAJOPT_IFOPT_FWD_H
#define TRAJOPT_IFOPT_FWD_H

namespace trajopt_ifopt
{
// variable_sets
class NodesVariables;
class Node;
class Var;

// cartesian_line_constraint.h
struct CartLineInfo;
class CartLineConstraint;

// cartesian_position_constraint.h
struct CartPosInfo;
class CartPosConstraint;

// inverse_kinematics_constraint.h
struct InverseKinematicsInfo;
class InverseKinematicsConstraint;

// joint_acceleration_constraint.h
class JointAccelConstraint;

// joint_jerk_constraint.h
class JointJerkConstraint;

// joint_position_constraint.h
class JointPosConstraint;

// joint_velocity_constraint.h
class JointVelConstraint;

// continuous_collision_evaluators.h
class ContinuousCollisionEvaluator;
class LVSContinuousCollisionEvaluator;
class LVSDiscreteCollisionEvaluator;

// discrete_collision_evaluators.h
class DiscreteCollisionEvaluator;
class SingleTimestepCollisionEvaluator;

// continuous_collision_constraint.h
class ContinuousCollisionConstraint;
class ContinuousCollisionNumericalConstraint;

// discrete_collision_constraint.h
class DiscreteCollisionConstraint;
class DiscreteCollisionNumericalConstraint;

}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_FWD_H
