#pragma once

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cassert>
#include <iosfwd>
#include <json/json.h>
#include <limits>
#include <string>
#include <vector>
#include <memory>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/sco_common.hpp>

/**
@file solver_interface.hpp
@brief Interface to convex solvers

  This is based on Gurobi's nice c++ API (though the SCO Gurobi backend uses the
Gurobi c api).
  However, our intention is to allow for different solvers to be used as
backends.
 */

namespace sco
{
enum ConstraintType
{
  EQ,
  INEQ
};

using ConstraintTypeVector = std::vector<ConstraintType>;

enum CvxOptStatus
{
  CVX_SOLVED,
  CVX_INFEASIBLE,
  CVX_FAILED
};

/** @brief Convex optimization problem

Gotchas:
- after adding a variable, need to call update() before doing anything else with
that variable

 */
class Model
{
public:
  using Ptr = std::shared_ptr<Model>;
  using ConstPtr = std::shared_ptr<const Model>;

  Model() = default;
  virtual ~Model() = default;
  Model(const Model&) = default;
  Model& operator=(const Model&) = default;
  Model(Model&&) = default;
  Model& operator=(Model&&) = default;

  virtual Var addVar(const std::string& name) = 0;
  virtual Var addVar(const std::string& name, double lb, double ub);

  virtual Cnt addEqCnt(const AffExpr&, const std::string& name) = 0;     // expr == 0
  virtual Cnt addIneqCnt(const AffExpr&, const std::string& name) = 0;   // expr <= 0
  virtual Cnt addIneqCnt(const QuadExpr&, const std::string& name) = 0;  // expr <= 0

  virtual void removeVar(const Var& var);
  virtual void removeCnt(const Cnt& cnt);
  virtual void removeVars(const VarVector& vars) = 0;
  virtual void removeCnts(const CntVector& cnts) = 0;

  virtual void update() = 0;  // call after adding/deleting stuff
  virtual void setVarBounds(const Var& var, double lower, double upper);
  virtual void setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper) = 0;
  virtual double getVarValue(const Var& var) const;
  virtual DblVec getVarValues(const VarVector& vars) const = 0;
  virtual CvxOptStatus optimize() = 0;

  virtual void setObjective(const AffExpr&) = 0;
  virtual void setObjective(const QuadExpr&) = 0;
  virtual void writeToFile(const std::string& fname) const = 0;

  virtual VarVector getVars() const = 0;
};

struct VarRep
{
  using Ptr = std::shared_ptr<VarRep>;

  VarRep(std::size_t _index, std::string _name, void* _creator)
    : index(_index), name(std::move(_name)), removed(false), creator(_creator)
  {
  }
  std::size_t index;
  std::string name;
  bool removed;
  void* creator;
};

struct Var
{
  using Ptr = std::shared_ptr<Var>;

  VarRep::Ptr var_rep{ nullptr };
  Var() = default;
  ~Var() = default;
  Var(VarRep::Ptr var_rep) : var_rep(std::move(var_rep)) {}
  Var(const Var& other) = default;
  Var& operator=(const Var&) = default;
  Var(Var&&) = default;
  Var& operator=(Var&&) = default;

  double value(const double* x) const { return x[var_rep->index]; }
  double value(const DblVec& x) const
  {
    assert(var_rep->index < x.size());
    return x[static_cast<size_t>(var_rep->index)];
  }
};

struct CntRep
{
  using Ptr = std::shared_ptr<CntRep>;

  CntRep(std::size_t _index, void* _creator) : index(_index), removed(false), creator(_creator) {}
  CntRep(const CntRep&) = default;
  CntRep& operator=(const CntRep&) = default;
  CntRep(CntRep&&) = default;
  CntRep& operator=(CntRep&&) = default;
  ~CntRep() = default;

  std::size_t index;
  bool removed;
  void* creator;
  ConstraintType type;
  std::string expr;  // todo placeholder
};

struct Cnt
{
  using Ptr = std::shared_ptr<Cnt>;

  CntRep::Ptr cnt_rep{ nullptr };
  Cnt() = default;
  Cnt(const Cnt&) = default;
  Cnt& operator=(const Cnt&) = default;
  Cnt(Cnt&&) = default;
  Cnt& operator=(Cnt&&) = default;
  ~Cnt() = default;

  Cnt(CntRep::Ptr cnt_rep) : cnt_rep(std::move(cnt_rep)) {}
};

struct AffExpr
{  // affine expression

  using Ptr = std::shared_ptr<AffExpr>;

  double constant{ 0 };
  DblVec coeffs;
  VarVector vars;
  AffExpr() = default;
  ~AffExpr() = default;
  AffExpr(const AffExpr& other) = default;
  AffExpr& operator=(const AffExpr&) = default;
  AffExpr(AffExpr&&) = default;
  AffExpr& operator=(AffExpr&&) = default;

  explicit AffExpr(double a) : constant(a) {}
  explicit AffExpr(const Var& v) : constant(0), coeffs(1, 1), vars(1, v) {}

  size_t size() const { return coeffs.size(); }
  double value(const double* x) const;
  double value(const DblVec& x) const;
};

struct QuadExpr
{
  using Ptr = std::shared_ptr<QuadExpr>;

  AffExpr affexpr;
  DblVec coeffs;
  VarVector vars1;
  VarVector vars2;
  QuadExpr() = default;
  explicit QuadExpr(double a) : affexpr(a) {}
  explicit QuadExpr(const Var& v) : affexpr(v) {}
  explicit QuadExpr(AffExpr aff) : affexpr(std::move(aff)) {}
  size_t size() const { return coeffs.size(); }
  double value(const double* x) const;
  double value(const DblVec& x) const;
};

std::ostream& operator<<(std::ostream&, const Var&);
std::ostream& operator<<(std::ostream&, const Cnt&);
std::ostream& operator<<(std::ostream&, const AffExpr&);
std::ostream& operator<<(std::ostream&, const QuadExpr&);

class ModelType
{
public:
  enum Value
  {
    GUROBI,
    OSQP,
    QPOASES,
    BPMPD,
    AUTO_SOLVER
  };

  static const std::vector<std::string> MODEL_NAMES_;

  ModelType();
  ModelType(const ModelType::Value& v);
  ModelType(const int& v);
  ModelType(const std::string& s);
  operator int() const;
  bool operator==(const ModelType::Value& a) const;
  bool operator==(const ModelType& a) const;
  bool operator!=(const ModelType& a) const;
  void fromJson(const Json::Value& v);
  friend std::ostream& operator<<(std::ostream& os, const ModelType& cs);

private:
  Value value_;
};

std::vector<ModelType> availableSolvers();

std::ostream& operator<<(std::ostream& os, const ModelType& cs);

Model::Ptr createModel(ModelType model_type = ModelType::AUTO_SOLVER);

void vars2inds(const VarVector& vars, SizeTVec& inds);

void vars2inds(const VarVector& vars, IntVec& inds);

void cnts2inds(const CntVector& cnts, SizeTVec& inds);

void cnts2inds(const CntVector& cnts, IntVec& inds);

/**
 * @brief simplify2 gets as input a list of indices, corresponding to non-zero
 *        values in vals, checks that all indexed values are actually non-zero,
 *        and if they are not, removes them from vals and inds, so that
 *        inds_out.size() <= inds.size(). Also, it will compact vals so that
 *        vals_out.size() == inds_out.size()
 *
 * @param[in,out] inds indices of non-vero variables in vals
 * @param[in,out] val values
 */
void simplify2(IntVec& inds, DblVec& vals);
}  // namespace sco
