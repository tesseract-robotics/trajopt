#include <trajopt_sco/osqp_printing.hpp>

void printAffExpr(const sco::AffExpr& expr, std::ostream& outStream)
{
    std::string separator = "";
    if(expr.constant != 0)
    {
        outStream << expr.constant;
        separator = " + ";
    }

    for (size_t i = 0; i < expr.size(); ++i)
    {
      if(expr.coeffs[i] != 0)
      {
        if(expr.coeffs[i] == 1)
        {
            outStream << separator << expr.vars[i];
        }
        else
        {
            outStream << separator << expr.coeffs[i] << " " << expr.vars[i];
        }
        separator = " + ";
      }
    }
}


void printCost(const sco::QuadExpr& objective, std::ostream& outStream)
{
    outStream << "Minimize\n";
    printAffExpr(objective.affexpr, outStream);

    outStream << " + [ ";

    std::string op = "";
    for (size_t i = 0; i < objective.size(); ++i)
    {
      if(objective.coeffs[i] != 0)
      {
        outStream << op;
        if(objective.coeffs[i] != 1)
        {
            outStream << objective.coeffs[i] << " ";
        }
        if(objective.vars1[i].var_rep->name == objective.vars2[i].var_rep->name)
        {
            outStream << objective.vars1[i] << " ^ 2";
        }
        else
        {
            outStream << objective.vars1[i] << " * " << objective.vars2[i];
        }
        op = " + ";
      }
    }

    outStream << " ] /2\n";
}


void printConstraints(const sco::DblVec& lbs, const sco::DblVec& ubs, const sco::AffExprVector& exprs, std::ostream& outStream)
{

    outStream << "Subject To\n";

    for (std::size_t i = 0; i < exprs.size(); ++i)
    {
        outStream << 'c' << i  << ": " << lbs[i] << " <= ";
        printAffExpr(exprs[i], outStream);
        outStream << " <= " << ubs[i] << '\n';
    }
}
