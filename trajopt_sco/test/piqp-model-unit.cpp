#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_op_overloads.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/piqp_interface.hpp>

using namespace sco;

// min (x - 1)^2 + (y - 2)^2 + z^2  s.t.  x + y = 1,  y <= 0.25,  z in [0.5, 0.5]  =>  x = 0.75, y = 0.25, z = 0.5
TEST(PIQPModel, EqualityInequalityAndPinnedVariable)  // NOLINT
{
  const Model::Ptr model = createModel(ModelType::PIQP);
  const Var x = model->addVar("x");
  const Var y = model->addVar("y");
  const Var z = model->addVar("z");
  model->update();

  QuadExpr objective = exprSquare(x - 1.0);
  exprInc(objective, exprSquare(y - 2.0));
  exprInc(objective, exprSquare(z));
  model->setObjective(objective);
  model->addEqCnt(x + y - 1.0, "sum");
  model->addIneqCnt(y - 0.25, "y_max");
  model->setVarBounds(z, 0.5, 0.5);
  model->update();

  ASSERT_EQ(model->optimize(), CVX_SOLVED);
  const DblVec values = model->getVarValues({ x, y, z });
  EXPECT_NEAR(values[0], 0.75, 1e-6);
  EXPECT_NEAR(values[1], 0.25, 1e-6);
  EXPECT_NEAR(values[2], 0.5, 1e-9);
}

TEST(PIQPModel, Infeasible)  // NOLINT
{
  const Model::Ptr model = createModel(ModelType::PIQP);
  const Var x = model->addVar("x");
  model->update();

  model->setObjective(exprSquare(x));
  model->addIneqCnt(AffExpr(x), "x_max");
  model->addIneqCnt(AffExpr(1.0) - x, "x_min");
  model->update();

  EXPECT_EQ(model->optimize(), CVX_INFEASIBLE);
}

TEST(PIQPModel, ConfigSettingsAreUsed)  // NOLINT
{
  auto config = std::make_shared<PIQPModelConfig>();
  config->settings.kkt_solver = piqp::KKTSolver::dense_cholesky;
  const Model::Ptr model = createModel(ModelType::PIQP, config);
  const Var x = model->addVar("x");
  model->update();
  model->setObjective(exprSquare(x));

  EXPECT_EQ(model->optimize(), CVX_FAILED);
}
