#ifndef TRAJOPT_SQP_FWD_H
#define TRAJOPT_SQP_FWD_H

namespace trajopt_sqp
{
// types.h
enum class ConstraintType;
enum class CostPenaltyType;
enum class SQPStatus;
struct SQPParameters;
struct SQPResults;

// expressions.h
struct Exprs;
struct AffExprs;
struct QuadExprs;

// qp_problem.h
class QPProblem;

// qp_solver.h
enum class QPSolverStatus;
class QPSolver;

// osqp_eigen_solver.h
class OSQPEigenSolver;

// ifopt_qp_problem.h
class IfoptQPProblem;

// trajopt_qp_problem.h
class TrajOptQPProblem;

// callbacks
class SQPCallback;
class CartesianErrorPlottingCallback;
class ClearPlotterCallback;
class CollisionPlottingCallback;
class JointStatePlottingCallback;
class WaitForInputCallback;

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_FWD_H
