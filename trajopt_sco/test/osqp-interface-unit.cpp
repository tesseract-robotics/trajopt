#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <memory>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_sco/solver_interface.hpp>

using namespace sco;

namespace
{
// Add the constraint x == 1
Cnt pinVar(Model& model, const Var& var) { return model.addEqCnt(exprSub(AffExpr(var), 1.0), "pin"); }

// Add the constraint sum(x) == total
Cnt sumVars(Model& model, const VarVector& vars, double total)
{
  AffExpr expr;
  for (const auto& var : vars)
    exprInc(expr, var);
  return model.addEqCnt(exprSub(expr, total), "sum");
}

// Add variables and a badly conditioned objective, which makes OSQP adapt rho while solving
VarVector addIllConditionedObjective(Model& model, std::size_t n_vars)
{
  VarVector vars;
  for (std::size_t i = 0; i < n_vars; ++i)
    vars.push_back(model.addVar("x" + std::to_string(i), -10.0, 10.0));
  model.update();

  QuadExpr objective;
  double weight = 1e-4;
  for (const auto& var : vars)
  {
    exprInc(objective, exprMult(exprSquare(exprSub(AffExpr(var), 1.0)), weight));
    weight *= 10.0;
  }
  model.setObjective(objective);
  return vars;
}
}  // namespace

// Moving a constraint to another variable keeps the dimensions and the number of nonzeros of the
// constraint matrix, so only the index arrays tell the two sparsity patterns apart, and those first
// differ well past their start.
TEST(OSQPInterface, UpdateWorkspaceDetectsChangedSparsity)  // NOLINT
{
  const std::size_t n_vars = 16;
  const std::size_t first_pin = 5;
  const std::size_t second_pin = 9;

  auto config = std::make_shared<OSQPModelConfig>();
  config->update_workspace = true;
  const Model::Ptr model = createModel(ModelType::OSQP, config);

  VarVector vars;
  for (std::size_t i = 0; i < n_vars; ++i)
    vars.push_back(model->addVar("x" + std::to_string(i), -10.0, 10.0));
  model->update();

  QuadExpr objective;
  for (const auto& var : vars)
    exprInc(objective, exprSquare(var));
  model->setObjective(objective);

  const Cnt first_cnt = pinVar(*model, vars[first_pin]);
  ASSERT_EQ(model->optimize(), CVX_SOLVED);
  DblVec solution = model->getVarValues(vars);
  EXPECT_NEAR(solution[first_pin], 1.0, 1e-3);
  EXPECT_NEAR(solution[second_pin], 0.0, 1e-3);

  model->removeCnt(first_cnt);
  pinVar(*model, vars[second_pin]);
  ASSERT_EQ(model->optimize(), CVX_SOLVED);
  solution = model->getVarValues(vars);
  EXPECT_NEAR(solution[first_pin], 0.0, 1e-3);
  EXPECT_NEAR(solution[second_pin], 1.0, 1e-3);
}

// Without warm starting, a solve in a reused workspace starts from the same state as a solve in a
// new one, so both take the same iterations and agree up to rounding. Polishing is off because it
// would hide a difference.
TEST(OSQPInterface, UpdateWorkspaceWithoutWarmStartMatchesNewWorkspace)  // NOLINT
{
  const std::size_t n_vars = 8;

  auto config = std::make_shared<OSQPModelConfig>();
  config->settings.warm_starting = 0;
  config->settings.polishing = 0;

  auto update_config = std::make_shared<OSQPModelConfig>(*config);
  update_config->update_workspace = true;

  const Model::Ptr reused = createModel(ModelType::OSQP, update_config);
  const VarVector reused_vars = addIllConditionedObjective(*reused, n_vars);
  const Cnt first_cnt = sumVars(*reused, reused_vars, 1.0);
  ASSERT_EQ(reused->optimize(), CVX_SOLVED);
  reused->removeCnt(first_cnt);
  sumVars(*reused, reused_vars, 2.0);
  ASSERT_EQ(reused->optimize(), CVX_SOLVED);
  const DblVec reused_solution = reused->getVarValues(reused_vars);

  const Model::Ptr fresh = createModel(ModelType::OSQP, config);
  const VarVector fresh_vars = addIllConditionedObjective(*fresh, n_vars);
  sumVars(*fresh, fresh_vars, 2.0);
  ASSERT_EQ(fresh->optimize(), CVX_SOLVED);
  const DblVec fresh_solution = fresh->getVarValues(fresh_vars);

  for (std::size_t i = 0; i < n_vars; ++i)
    EXPECT_NEAR(reused_solution[i], fresh_solution[i], 1e-6) << "x" << i;
}
