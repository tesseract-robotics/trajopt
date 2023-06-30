#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <mutex>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_sco/solver_interface.hpp>

/**

@file gurobi_interface.hpp

Gurobi backend

*/

struct _GRBmodel;
typedef struct _GRBmodel GRBmodel;

namespace sco
{
class GurobiModel : public Model
{
public:
  using Ptr = std::shared_ptr<GurobiModel>;

  GRBmodel* m_model;
  VarVector m_vars;
  CntVector m_cnts;
  std::mutex m_mutex;

  GurobiModel();
  ~GurobiModel();

  // Must be threadsafe
  Var addVar(const std::string& name) override;
  Var addVar(const std::string& name, double lower, double upper) override;
  Cnt addEqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const AffExpr&, const std::string& name) override;
  Cnt addIneqCnt(const QuadExpr&, const std::string& name) override;
  void removeVars(const VarVector&) override;
  void removeCnts(const CntVector&) override;

  // These do not need to be threadsafe
  void update() override;
  CvxOptStatus optimize() override;
  void setObjective(const AffExpr&) override;
  void setObjective(const QuadExpr&) override;
  void setVarBounds(const VarVector&, const DblVec& lower, const DblVec& upper) override;
  DblVec getVarValues(const VarVector&) const override;
  void writeToFile(const std::string& fname) const override;
  VarVector getVars() const override;

  /** Don't use this function, because it adds constraints that aren't tracked*/
  CvxOptStatus optimizeFeasRelax();
};
}  // namespace sco
