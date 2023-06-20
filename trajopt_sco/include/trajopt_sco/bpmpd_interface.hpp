#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <mutex>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_sco/solver_interface.hpp>

namespace sco
{
class BPMPDModel : public Model
{
public:
  VarVector m_vars;
  CntVector m_cnts;
  AffExprVector m_cntExprs;
  ConstraintTypeVector m_cntTypes;
  DblVec m_soln;
  DblVec m_lbs, m_ubs;

  QuadExpr m_objective;

  int m_pipeIn{ 0 };
  int m_pipeOut{ 0 };
  int m_pid{ 0 };

  std::mutex m_mutex; /**< The mutex */

  BPMPDModel();
  ~BPMPDModel() override = default;
  BPMPDModel(const BPMPDModel&) = delete;
  BPMPDModel& operator=(const BPMPDModel&) = delete;
  BPMPDModel(BPMPDModel&&) = delete;
  BPMPDModel& operator=(BPMPDModel&&) = delete;

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
