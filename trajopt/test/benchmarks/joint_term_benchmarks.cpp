#include <benchmark/benchmark.h>
#include <iostream>

#include <Eigen/Eigen>
#include <trajopt/typedefs.hpp>
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/osqp_interface.hpp>

/**
 * @brief Creates a VarArray based on the model passed in
 * @param model - Model that will store the varReps for the vars in the VarArray
 * @param num_rows - Number of rows in the VarArray
 * @param num_cols - Number of columns in the VarArray
 * @return
 */
inline trajopt::VarArray createVarArray(std::shared_ptr<sco::OSQPModel>& model,
                                        const int& num_rows,
                                        const int& num_cols)
{
  trajopt::VarArray output;

  for (int ind = 0; ind < num_rows; ind++)
  {
    for (int ind2 = 0; ind2 < num_cols; ind2++)
    {
      std::string name = std::to_string(ind) + "_" + std::to_string(ind2);
      model->addVar(name);

      // Copy in VarVector
      output.m_data = model->getVars();
      // Set rows and columns
      output.m_nRow = num_rows;
      output.m_nCol = num_cols;
    }
  }
  return output;
}

/**
 * @brief Contains information used to construct the terms
 *
 * Note: It is probably possible to make these templated fixture benchmarks in the future.
 */
class BenchmarkInfo
{
public:
  BenchmarkInfo(int num_rows, int num_cols) : num_rows_(num_rows), num_cols_(num_cols)
  {
    coeffs = Eigen::VectorXd::Ones(num_cols_);
    targets = Eigen::VectorXd::Zero(num_cols_);
    upper = Eigen::VectorXd::Ones(num_cols_) * 0.5;
    lower = Eigen::VectorXd::Ones(num_cols_) * -0.5;
    first_step = 0;
    last_step = num_rows_ - 1;
    model = std::make_shared<sco::OSQPModel>();
    traj = createVarArray(model, num_rows_, num_cols_);

    trajopt::TrajArray traj_array = trajopt::TrajArray::Random(num_rows_, num_cols_);
    dbl_vec = trajopt::DblVec(traj_array.data(), traj_array.data() + traj_array.rows() * traj_array.cols());
  }
  Eigen::VectorXd coeffs;
  Eigen::VectorXd targets;
  Eigen::VectorXd upper;
  Eigen::VectorXd lower;
  int first_step;
  int last_step;
  std::shared_ptr<sco::OSQPModel> model;
  trajopt::VarArray traj;
  trajopt::DblVec dbl_vec;

private:
  int num_rows_;
  int num_cols_;
};

/** @brief Benchmark that tests the construction time of equality costs/cnts*/
template <class CostClass>
void BM_EQ_TERM_CONSTRUCTION(benchmark::State& state)
{
  BenchmarkInfo info(10, 6);

  for (auto _ : state)
  {
    CostClass object(info.traj, info.coeffs, info.targets, info.first_step, info.last_step);
  }
}

BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointPosEqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointVelEqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointAccEqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointJerkEqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointPosEqConstraint)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointVelEqConstraint)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointAccEqConstraint)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONSTRUCTION, trajopt::JointJerkEqConstraint)->Unit(benchmark::TimeUnit::kMicrosecond);

/** @brief Benchmark that tests the construction time of inequality costs/cnts*/
template <class CostClass>
void BM_INEQ_TERM_CONSTRUCTION(benchmark::State& state)
{
  BenchmarkInfo info(10, 6);

  for (auto _ : state)
  {
    CostClass object(info.traj, info.coeffs, info.targets, info.upper, info.lower, info.first_step, info.last_step);
  }
}

BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointPosIneqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointVelIneqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointAccIneqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointJerkIneqCost)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointPosIneqConstraint)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointVelIneqConstraint)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointAccIneqConstraint)->Unit(benchmark::TimeUnit::kMicrosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONSTRUCTION, trajopt::JointJerkIneqConstraint)
    ->Unit(benchmark::TimeUnit::kMicrosecond);

/** @brief Benchmark that tests the exact evaluation time of equality costs/cnts*/
template <class CostClass>
void BM_EQ_TERM_VALUE(benchmark::State& state)
{
  BenchmarkInfo info(10, 6);
  CostClass object(info.traj, info.coeffs, info.targets, info.first_step, info.last_step);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(object.value(info.dbl_vec));
  }
}

BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointPosEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointVelEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointAccEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointJerkEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointPosEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointVelEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointAccEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_VALUE, trajopt::JointJerkEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);

/** @brief Benchmark that tests the exact evaluation time of inequality costs/cnts*/
template <class CostClass>
void BM_INEQ_TERM_VALUE(benchmark::State& state)
{
  BenchmarkInfo info(10, 6);
  CostClass object(info.traj, info.coeffs, info.targets, info.upper, info.lower, info.first_step, info.last_step);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(object.value(info.dbl_vec));
  }
}

BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointPosIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointVelIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointAccIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointJerkIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointPosIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointVelIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointAccIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_VALUE, trajopt::JointJerkIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);

/** @brief Benchmark that tests the convex conversion time of equality costs/cnts*/
template <class CostClass>
void BM_EQ_TERM_CONVEX(benchmark::State& state)
{
  BenchmarkInfo info(10, 6);
  CostClass object(info.traj, info.coeffs, info.targets, info.first_step, info.last_step);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(object.convex(info.dbl_vec, info.model.get()));
  }
}

BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointPosEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointVelEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointAccEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointJerkEqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointPosEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointVelEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointAccEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_EQ_TERM_CONVEX, trajopt::JointJerkEqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);

/** @brief Benchmark that tests the convex conversion time of equality costs/cnts*/
template <class CostClass>
void BM_INEQ_TERM_CONVEX(benchmark::State& state)
{
  BenchmarkInfo info(10, 6);
  CostClass object(info.traj, info.coeffs, info.targets, info.upper, info.lower, info.first_step, info.last_step);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(object.convex(info.dbl_vec, info.model.get()));
  }
}

BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointPosIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointVelIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointAccIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointJerkIneqCost)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointPosIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointVelIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointAccIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);
BENCHMARK_TEMPLATE(BM_INEQ_TERM_CONVEX, trajopt::JointJerkIneqConstraint)->Unit(benchmark::TimeUnit::kNanosecond);

BENCHMARK_MAIN();
