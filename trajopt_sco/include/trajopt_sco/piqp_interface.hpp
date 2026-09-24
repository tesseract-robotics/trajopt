#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <mutex>
#include <piqp/settings.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_interface.hpp>

namespace sco
{
/** @brief The PIQP configuration settings */
struct PIQPModelConfig : public ModelConfig
{
  using Ptr = std::shared_ptr<PIQPModelConfig>;
  using ConstPtr = std::shared_ptr<const PIQPModelConfig>;

  PIQPModelConfig();

  /** @brief kkt_solver must be one of the sparse_ldlt variants, the sparse backend has no other */
  piqp::Settings<double> settings;

  /**
   * @brief Set the default PIQP Settings
   * @param settings The object to apply default settings to
   */
  static void setDefaultPIQPSettings(piqp::Settings<double>& settings);
};

/**
 * PIQPModel uses the BSD solver PIQP, a proximal interior point method, to solve a linearly constrained QP.
 * PIQP solves a problem in the form:
 * ```
 * min   1/2*x'Px + c'x
 * s.t.  Ax = b
 *       h_l <= Gx <= h_u
 *       x_l <= x <= x_u
 * ```
 * Equality constraints map to A, inequality constraints to G and variable bounds to x_l, x_u.
 * The solver is set up from scratch on every optimize().
 *
 * More information about the solver is available at:
 * https://predict-epfl.github.io/piqp/
 */
class PIQPModel : public Model
{
  VarVector vars_;                 /**< model variables */
  CntVector cnts_;                 /**< model's constraints sizes */
  DblVec lbs_, ubs_;               /**< variables bounds */
  AffExprVector cnt_exprs_;        /**< constraints expressions */
  ConstraintTypeVector cnt_types_; /**< constraints types */
  DblVec solution_;                /**< optimizer's solution for current model */

  QuadExpr objective_; /**< objective QuadExpr expression */

  PIQPModelConfig config_; /**< The configuration settings */

  std::mutex mutex_; /**< The mutex */

public:
  PIQPModel(const ModelConfig::ConstPtr& config = nullptr);
  ~PIQPModel() override;
  PIQPModel(const PIQPModel& model) = delete;
  PIQPModel& operator=(const PIQPModel& model) = delete;
  PIQPModel(PIQPModel&&) = delete;
  PIQPModel& operator=(PIQPModel&&) = delete;

  // Must be threadsafe
  Var addVar(const std::string& name) override;
  Cnt addEqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const QuadExpr&, const std::string& name) override;
  void removeVars(const VarVector& vars) override;
  void removeCnts(const CntVector& cnts) override;

  // These do not need to be threadsafe
  void update() override;
  CvxOptStatus optimize() override;
  void setObjective(const AffExpr&) override;
  void setObjective(const QuadExpr&) override;
  void setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper) override;
  DblVec getVarValues(const VarVector& vars) const override;
  void writeToFile(const std::string& fname) const override;
  VarVector getVars() const override;
};
}  // namespace sco
