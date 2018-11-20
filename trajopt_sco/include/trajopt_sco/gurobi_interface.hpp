#pragma once
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
  GRBmodel* m_model;
  VarVector m_vars;
  CntVector m_cnts;

  GurobiModel();

  Var addVar(const std::string& name);
  Var addVar(const std::string& name, double lower, double upper);

  Cnt addEqCnt(const AffExpr&, const std::string& name);
  Cnt addIneqCnt(const AffExpr&, const std::string& name);
  Cnt addIneqCnt(const QuadExpr&, const std::string& name);

  void removeVars(const VarVector&);
  void removeCnts(const CntVector&);

  void update();
  void setVarBounds(const VarVector&, const DblVec& lower, const DblVec& upper);
  DblVec getVarValues(const VarVector&) const;

  CvxOptStatus optimize();
  /** Don't use this function, because it adds constraints that aren't tracked
   */
  CvxOptStatus optimizeFeasRelax();

  void setObjective(const AffExpr&);
  void setObjective(const QuadExpr&);
  void writeToFile(const std::string& fname);

  VarVector getVars() const;

  ~GurobiModel();
};
}
