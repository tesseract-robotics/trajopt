#include <boost/format.hpp>
#include <iostream>
#include <map>
#include <sstream>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>
using namespace std;

namespace sco
{
vector<int> vars2inds(const vector<Var>& vars)
{
  vector<int> inds(vars.size());
  for (size_t i = 0; i < inds.size(); ++i)
    inds[i] = vars[i].var_rep->index;
  return inds;
}
vector<int> cnts2inds(const vector<Cnt>& cnts)
{
  vector<int> inds(cnts.size());
  for (size_t i = 0; i < inds.size(); ++i)
    inds[i] = cnts[i].cnt_rep->index;
  return inds;
}

void simplify2(vector<int>& inds, vector<double>& vals)
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
double AffExpr::value(const vector<double>& x) const
{
  double out = constant;
  for (size_t i = 0; i < size(); ++i)
  {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double QuadExpr::value(const vector<double>& x) const
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

Var Model::addVar(const string& name, double lb, double ub)
{
  Var v = addVar(name);
  setVarBounds(v, lb, ub);
  return v;
}
void Model::removeVar(const Var& var)
{
  vector<Var> vars(1, var);
  removeVars(vars);
}
void Model::removeCnt(const Cnt& cnt)
{
  vector<Cnt> cnts(1, cnt);
  removeCnts(cnts);
}

double Model::getVarValue(const Var& var) const
{
  VarVector vars(1, var);
  return getVarValues(vars)[0];
}

void Model::setVarBounds(const Var& var, double lower, double upper)
{
  vector<double> lowers(1, lower), uppers(1, upper);
  vector<Var> vars(1, var);
  setVarBounds(vars, lowers, uppers);
}

ostream& operator<<(ostream& o, const Var& v)
{
  if (v.var_rep != NULL)
    o << v.var_rep->name;
  else
    o << "nullvar";
  return o;
}
ostream& operator<<(ostream& o, const Cnt& c)
{
  o << c.cnt_rep->expr << ((c.cnt_rep->type == EQ) ? " == 0" : " <= 0");
  return o;
}
ostream& operator<<(ostream& o, const AffExpr& e)
{
  o << e.constant;
  for (size_t i = 0; i < e.size(); ++i)
  {
    o << " + " << e.coeffs[i] << "*" << e.vars[i];
  }
  return o;
}
ostream& operator<<(ostream& o, const QuadExpr& e)
{
  o << e.affexpr;
  for (size_t i = 0; i < e.size(); ++i)
  {
    o << " + " << e.coeffs[i] << "*" << e.vars1[i] << "*" << e.vars2[i];
  }
  return o;
}

std::vector<ConvexSolver> availableSolvers()
{
  std::vector<bool> has_solver(AUTO_SOLVER, false);
#ifdef HAVE_GUROBI
  has_solver[GUROBI] = true;
#endif
#ifdef HAVE_BPMPD
  has_solver[BPMPD] = true;
#endif
#ifdef HAVE_OSQP
  has_solver[OSQP] = true;
#endif
#ifdef HAVE_QPOASES
  has_solver[QPOASES] = true;
#endif
  int n_available_solvers = 0;
  for (auto i = 0; i < AUTO_SOLVER; ++i)
    if (has_solver[i])
      ++n_available_solvers;
  std::vector<ConvexSolver> available_solvers(n_available_solvers, AUTO_SOLVER);
  auto j = 0;
  for (auto i = 0; i < AUTO_SOLVER; ++i)
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

  if(solver == AUTO_SOLVER)
  {
    if (solver_env)
    {
      if (string(solver_env) == "GUROBI")
        solver = GUROBI;
      else if (string(solver_env) == "BPMPD")
        solver = BPMPD;
      else if (string(solver_env) == "OSQP")
        solver = OSQP;
      else if (string(solver_env) == "QPOASES")
        solver = QPOASES;
      else
        PRINT_AND_THROW(boost::format("invalid solver \"%s\"specified by TRAJOPT_CONVEX_SOLVER") % solver_env);
    }
    else
    {
      solver = availableSolvers()[0];
    }
  }

#ifndef HAVE_GUROBI
  if (solver == GUROBI)
    PRINT_AND_THROW("you didn't build with GUROBI support");
#endif
#ifndef HAVE_BPMPD
  if (solver == BPMPD)
    PRINT_AND_THROW("you don't have BPMPD support on this platform");
#endif
#ifndef HAVE_OSQP
  if (solver == OSQP)
    PRINT_AND_THROW("you don't have OSQP support on this platform");
#endif
#ifndef HAVE_QPOASES
  if (solver == QPOASES)
    PRINT_AND_THROW("you don't have qpOASES support on this platform");
#endif

#ifdef HAVE_GUROBI
  if (solver == GUROBI)
    return createGurobiModel();
#endif
#ifdef HAVE_BPMPD
  if (solver == BPMPD)
    return createBPMPDModel();
#endif
#ifdef HAVE_OSQP
  if (solver == OSQP)
    return createOSQPModel();
#endif
#ifdef HAVE_QPOASES
  if (solver == QPOASES)
    return createqpOASESModel();
#endif
  PRINT_AND_THROW("Failed to create solver");
  return ModelPtr();
}
}
