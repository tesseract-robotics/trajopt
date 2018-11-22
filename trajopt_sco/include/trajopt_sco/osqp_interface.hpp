#pragma once
#include <Eigen/Core>
#include <osqp.h>

#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

namespace sco
{
class OSQPModel : public Model
{
  OSQPSettings osqp_settings_;     // OSQP Settings
  OSQPData osqp_data_;             // OSQPData
  OSQPWorkspace* osqp_workspace_;  // OSQP Workspace

  void updateObjective();
  void updateConstraints();
  void createOrUpdateSolver();

public:
  VarVector vars_;
  CntVector cnts_;
  DblVec lbs_, ubs_;
  AffExprVector cnt_exprs_;
  ConstraintTypeVector cnt_types_;
  DblVec solution_;

  std::vector<c_int> P_row_indices_;
  std::vector<c_int> P_column_pointers_;
  DblVec P_csc_data_;
  Eigen::VectorXd q_;

  std::vector<c_int> A_row_indices_;
  std::vector<c_int> A_column_pointers_;
  DblVec A_csc_data_;
  DblVec l_, u_;

  QuadExpr objective_;

  OSQPModel();
  virtual ~OSQPModel();

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
