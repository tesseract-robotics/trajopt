#include <trajopt_sco/solver_interface.hpp>
#include <ostream>

void printCost(const sco::QuadExpr& objective, std::ostream& outStream);
void printAffExpr(const sco::AffExpr& expr, std::ostream& outStream);
void printConstraints(const sco::DblVec& lbs, const sco::DblVec& ubs, const sco::AffExprVector& exprs, std::ostream& outStream);
