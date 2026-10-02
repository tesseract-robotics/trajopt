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
