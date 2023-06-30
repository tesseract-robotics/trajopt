#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <cmath>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

namespace sco
{
struct Var;
struct AffExpr;
struct QuadExpr;
struct Cnt;

using DblVec = std::vector<double>;
using IntVec = std::vector<int>;
using SizeTVec = std::vector<std::size_t>;
using VarVector = std::vector<Var>;
using AffExprVector = std::vector<AffExpr>;
using QuadExprVector = std::vector<QuadExpr>;
using CntVector = std::vector<Cnt>;

// NOLINTNEXTLINE
inline double vecSum(const DblVec& v)
{
  double out = 0;
  for (double i : v)
    out += i;
  return out;
}

// NOLINTNEXTLINE
inline double vecAbsSum(const DblVec& v)
{
  double out = 0;
  for (double i : v)
    out += fabs(i);
  return out;
}

// NOLINTNEXTLINE
inline double pospart(double x) { return (x > 0) ? x : 0; }

// NOLINTNEXTLINE
inline double sq(double x) { return x * x; }

// NOLINTNEXTLINE
inline double vecHingeSum(const DblVec& v)
{
  double out = 0;
  for (double i : v)
    out += pospart(i);
  return out;
}

// NOLINTNEXTLINE
inline double vecMax(const DblVec& v) { return *std::max_element(v.begin(), v.end()); }

// NOLINTNEXTLINE
inline double vecDot(const DblVec& a, const DblVec& b)
{
  assert(a.size() == b.size());
  double out = 0;
  for (unsigned i = 0; i < a.size(); ++i)
    out += a[i] * b[i];
  return out;
}
}  // namespace sco
