#pragma once
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

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

  int m_pipeIn, m_pipeOut, m_pid;

  BPMPDModel();
  virtual ~BPMPDModel();

  Var addVar(const std::string& name);
  Cnt addEqCnt(const AffExpr&, const std::string& name);
  Cnt addIneqCnt(const AffExpr&, const std::string& name);
  Cnt addIneqCnt(const QuadExpr&, const std::string& name);
  void removeVars(const VarVector& vars);
  void removeCnts(const CntVector& cnts);

  void update();
  void setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper);
  DblVec getVarValues(const VarVector& vars) const;
  virtual CvxOptStatus optimize();
  virtual void setObjective(const AffExpr&);
  virtual void setObjective(const QuadExpr&);
  virtual void writeToFile(const std::string& fname);
  virtual VarVector getVars() const;
};
}
