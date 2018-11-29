#include <boost/format.hpp>
#include <iostream>
#include <map>
#include <sstream>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

namespace sco
{
const std::vector<std::string> ConvexSolver::SOLVER_NAMES_ = {
  "GUROBI",
  "BPMPD",
  "OSQP",
  "QPOASES",
  "AUTO_SOLVER"
};

IntVec vars2inds(const VarVector& vars)
{
  IntVec inds(vars.size());
  for (size_t i = 0; i < inds.size(); ++i)
    inds[i] = vars[i].var_rep->index;
  return inds;
}
IntVec cnts2inds(const CntVector& cnts)
{
  IntVec inds(cnts.size());
  for (size_t i = 0; i < inds.size(); ++i)
    inds[i] = cnts[i].cnt_rep->index;
  return inds;
}

void simplify2(IntVec& inds, DblVec& vals)
{
  typedef std::map<int, double> Int2Double;
  Int2Double ind2val;
  for (unsigned i = 0; i < inds.size(); ++i)
  {
    if (vals[i] != 0.0)
      ind2val[inds[i]] += vals[i];
  }
  inds.resize(ind2val.size());
  vals.resize(ind2val.size());
  int i_new = 0;
  for (Int2Double::value_type& iv : ind2val)
  {
    inds[i_new] = iv.first;
    vals[i_new] = iv.second;
    ++i_new;
  }
}

double AffExpr::value(const double* x) const
{
  double out = constant;
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double AffExpr::value(const DblVec& x) const
{
  double out = constant;
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double QuadExpr::value(const DblVec& x) const
{
  double out = affexpr.value(x);
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}
double QuadExpr::value(const double* x) const
{
  double out = affexpr.value(x);
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}

Var Model::addVar(const std::string& name, double lb, double ub)
{
  Var v = addVar(name);
  setVarBounds(v, lb, ub);
  return v;
}
void Model::removeVar(const Var& var)
{
  VarVector vars(1, var);
  removeVars(vars);
}
void Model::removeCnt(const Cnt& cnt)
{
  CntVector cnts(1, cnt);
  removeCnts(cnts);
}

double Model::getVarValue(const Var& var) const
{
  VarVector vars(1, var);
  return getVarValues(vars)[0];
}

void Model::setVarBounds(const Var& var, double lower, double upper)
{
  DblVec lowers(1, lower), uppers(1, upper);
  VarVector vars(1, var);
  setVarBounds(vars, lowers, uppers);
}

std::ostream& operator<<(std::ostream& o, const Var& v)
{
  if (v.var_rep != NULL)
    o << v.var_rep->name;
  else
    o << "nullvar";
  return o;
}

std::ostream& operator<<(std::ostream& o, const Cnt& c)
{
  o << c.cnt_rep->expr << ((c.cnt_rep->type == EQ) ? " == 0" : " <= 0");
  return o;
}

std::ostream& operator<<(std::ostream& o, const AffExpr& e)
{
  o << e.constant;
  for (size_t i = 0; i < e.size(); ++i)
  {
    o << " + " << e.coeffs[i] << "*" << e.vars[i];
  }
  return o;
}

std::ostream& operator<<(std::ostream& o, const QuadExpr& e)
{
  o << e.affexpr;
  for (size_t i = 0; i < e.size(); ++i)
  {
    o << " + " << e.coeffs[i] << "*" << e.vars1[i] << "*" << e.vars2[i];
  }
  return o;
}

std::ostream& operator<<(std::ostream& o, const ConvexSolver& cs)
{
  int cs_ivalue_ = static_cast<int>(cs.value_);
  if (cs_ivalue_ > cs.SOLVER_NAMES_.size())
  {
    std::stringstream conversion_error;
    conversion_error << "Error converting ConvexSolver to string - "
                     << "enum value is " << cs_ivalue_ << std::endl;
    throw std::runtime_error(conversion_error.str());
  }
  o << ConvexSolver::SOLVER_NAMES_[cs_ivalue_];
  return o;
}

ConvexSolver::ConvexSolver()
{
  value_ = ConvexSolver::AUTO_SOLVER;
}

ConvexSolver::ConvexSolver(const ConvexSolver::Value& v)
{
  value_ = v;
}

ConvexSolver::ConvexSolver(const int& v)
{
  value_ = static_cast<Value>(v);
}

ConvexSolver::ConvexSolver(const std::string& s)
{
  for (unsigned int i = 0; i < ConvexSolver::SOLVER_NAMES_.size(); ++i)
  {
    if (s == ConvexSolver::SOLVER_NAMES_[i])
    {
      value_ = static_cast<ConvexSolver::Value>(i);
      return;
    }
  }
  PRINT_AND_THROW(boost::format("invalid solver name:\"%s\"") % s);
}

ConvexSolver::operator int() const
{
  return static_cast<int>(value_);
}

bool ConvexSolver::operator==(const ConvexSolver::Value& a) const
{
  return value_ == a;
}

bool ConvexSolver::operator==(const ConvexSolver& a) const
{
  return value_ == a.value_;
}

bool ConvexSolver::operator!=(const ConvexSolver& a) const
{
  return value_ != a.value_;
}

void ConvexSolver::fromJson(const Json::Value& v) {
  try
  {
    std::string ref = v.asString();
    ConvexSolver cs(ref);
    value_ = cs.value_;
  }
  catch (const std::runtime_error&)
  {
    PRINT_AND_THROW(boost::format("expected: %s, got %s") % ("string") % (v));
  }
}

std::vector<ConvexSolver> availableSolvers()
{
  std::vector<bool> has_solver(ConvexSolver::AUTO_SOLVER, false);
#ifdef HAVE_GUROBI
  has_solver[ConvexSolver::GUROBI] = true;
#endif
#ifdef HAVE_BPMPD
  has_solver[ConvexSolver::BPMPD] = true;
#endif
#ifdef HAVE_OSQP
  has_solver[ConvexSolver::OSQP] = true;
#endif
#ifdef HAVE_QPOASES
  has_solver[ConvexSolver::QPOASES] = true;
#endif
  int n_available_solvers = 0;
  for (auto i = 0; i < ConvexSolver::AUTO_SOLVER; ++i)
    if (has_solver[i])
      ++n_available_solvers;
  std::vector<ConvexSolver> available_solvers(n_available_solvers,
                                              ConvexSolver::AUTO_SOLVER);
  auto j = 0;
  for (auto i = 0; i < ConvexSolver::AUTO_SOLVER; ++i)
    if (has_solver[i])
      available_solvers[j++] = static_cast<ConvexSolver>(i);
  return available_solvers;
}

ModelPtr createModel(ConvexSolver convex_solver)
{
#ifdef HAVE_GUROBI
  extern ModelPtr createGurobiModel();
#endif
#ifdef HAVE_BPMPD
  extern ModelPtr createBPMPDModel();
#endif
#ifdef HAVE_OSQP
  extern ModelPtr createOSQPModel();
#endif
#ifdef HAVE_QPOASES
  extern ModelPtr createqpOASESModel();
#endif

  char* solver_env = getenv("TRAJOPT_CONVEX_SOLVER");

  ConvexSolver solver = convex_solver;

  if (solver == ConvexSolver::AUTO_SOLVER)
  {
    if (solver_env and std::string(solver_env) != "AUTO_SOLVER")
    {
      try
      {
        solver = ConvexSolver(std::string(solver_env));
      }
      catch (std::runtime_error&)
      {
        PRINT_AND_THROW(boost::format("invalid solver \"%s\"specified by TRAJOPT_CONVEX_SOLVER") % solver_env);
      }
    }
    else
    {
      solver = availableSolvers()[0];
    }
  }

#ifndef HAVE_GUROBI
  if (solver == ConvexSolver::GUROBI)
    PRINT_AND_THROW("you didn't build with GUROBI support");
#endif
#ifndef HAVE_BPMPD
  if (solver == ConvexSolver::BPMPD)
    PRINT_AND_THROW("you don't have BPMPD support on this platform");
#endif
#ifndef HAVE_OSQP
  if (solver == ConvexSolver::OSQP)
    PRINT_AND_THROW("you don't have OSQP support on this platform");
#endif
#ifndef HAVE_QPOASES
  if (solver == ConvexSolver::QPOASES)
    PRINT_AND_THROW("you don't have qpOASES support on this platform");
#endif

#ifdef HAVE_GUROBI
  if (solver == ConvexSolver::GUROBI)
    return createGurobiModel();
#endif
#ifdef HAVE_BPMPD
  if (solver == ConvexSolver::BPMPD)
    return createBPMPDModel();
#endif
#ifdef HAVE_OSQP
  if (solver == ConvexSolver::OSQP)
    return createOSQPModel();
#endif
#ifdef HAVE_QPOASES
  if (solver == ConvexSolver::QPOASES)
    return createqpOASESModel();
#endif
  std::stringstream solver_instatiation_error;
  solver_instatiation_error << "Failed to create solver: unknown solver "
                            << solver << std::endl;
  PRINT_AND_THROW(solver_instatiation_error.str());
  return ModelPtr();
}
}
