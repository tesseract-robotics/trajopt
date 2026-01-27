#ifndef TRAJOPT_SQP_FWD_H
#define TRAJOPT_SQP_FWD_H

#include <cstdint>

namespace trajopt_sqp
{
// types.h
enum class CostPenaltyType : std::uint8_t;
enum class SQPStatus : std::uint8_t;
struct SQPParameters;
struct SQPResults;

// expressions.h
struct Exprs;
struct AffExprs;
struct QuadExprs;

// qp_problem.h
class QPProblem;

// qp_solver.h
enum class QPSolverStatus : std::uint8_t;
class QPSolver;

// osqp_eigen_solver.h
class OSQPEigenSolver;

// ifopt_qp_problem.h
class IfoptQPProblem;

// trajopt_qp_problem.h
class TrajOptQPProblem;

// trust_region_sqp_solver.h
class TrustRegionSQPSolver;

// callbacks
class SQPCallback;
class CartesianErrorPlottingCallback;
class ClearPlotterCallback;
class CollisionPlottingCallback;
class JointStatePlottingCallback;
class WaitForInputCallback;

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_FWD_H
