#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <cassert>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace bpmpd_io
{
enum SerMode
{
  DESER,
  SER
};

template <typename T>
void ser(int fp, T& x, SerMode mode)
{
  switch (mode)
  {
    case SER:
    {
      T xcopy = x;
      ssize_t n = write(fp, &xcopy, sizeof(T));
      assert(n == sizeof(T));
      UNUSED(n);
      break;
    }
    case DESER:
    {
      ssize_t n = read(fp, &x, sizeof(T));
      assert(n == sizeof(T));
      UNUSED(n);
      break;
    }
  }
}

template <typename T>
void ser(int fp, std::vector<T>& x, SerMode mode)
{
  unsigned long size = x.size();
  ser(fp, size, mode);
  switch (mode)
  {
    case SER:
    {
      long n = write(fp, x.data(), sizeof(T) * size);
      assert(static_cast<unsigned long>(n) == sizeof(T) * size);
      UNUSED(n);
      break;
    }
    case DESER:
    {
      x.resize(size);
      long n = read(fp, x.data(), sizeof(T) * size);
      assert(static_cast<unsigned long>(n) == sizeof(T) * size);
      UNUSED(n);
      break;
    }
  }
}

struct bpmpd_input
{
  int m{ 0 }, n{ 0 }, nz{ 0 }, qn{ 0 }, qnz{ 0 };
  std::vector<int> acolcnt, acolidx;
  std::vector<double> acolnzs;
  std::vector<int> qcolcnt, qcolidx;
  std::vector<double> qcolnzs;
  std::vector<double> rhs, obj, lbound, ubound;

  bpmpd_input() = default;
  bpmpd_input(int m,
              int n,
              int nz,
              int qn,
              int qnz,
              std::vector<int> acolcnt,
              std::vector<int> acolidx,
              std::vector<double> acolnzs,
              std::vector<int> qcolcnt,
              std::vector<int> qcolidx,
              std::vector<double> qcolnzs,
              std::vector<double> rhs,
              std::vector<double> obj,
              std::vector<double> lbound,
              std::vector<double> ubound)
    : m(m)
    , n(n)
    , nz(nz)
    , qn(qn)
    , qnz(qnz)
    , acolcnt(std::move(acolcnt))
    , acolidx(std::move(acolidx))
    , acolnzs(std::move(acolnzs))
    , qcolcnt(std::move(qcolcnt))
    , qcolidx(std::move(qcolidx))
    , qcolnzs(std::move(qcolnzs))
    , rhs(std::move(rhs))
    , obj(std::move(obj))
    , lbound(std::move(lbound))
    , ubound(std::move(ubound))
  {
  }
};

const char EXIT_CHAR = 123;
const char CHECK_CHAR = 111;

inline void ser(int fp, bpmpd_input& bi, SerMode mode)
{
  char scorrect = 'z', s = (mode == SER) ? scorrect : 0;  // NOLINT
  ser(fp, s, mode);
  if (s == EXIT_CHAR)
  {
    exit(0);
  }

  ser(fp, bi.m, mode);
  ser(fp, bi.n, mode);
  ser(fp, bi.nz, mode);
  ser(fp, bi.qn, mode);
  ser(fp, bi.qnz, mode);
  ser(fp, bi.acolcnt, mode);
  ser(fp, bi.acolidx, mode);
  ser(fp, bi.acolnzs, mode);
  ser(fp, bi.qcolcnt, mode);
  ser(fp, bi.qcolidx, mode);
  ser(fp, bi.qcolnzs, mode);
  ser(fp, bi.rhs, mode);
  ser(fp, bi.obj, mode);
  ser(fp, bi.lbound, mode);
  ser(fp, bi.ubound, mode);
}

struct bpmpd_output
{
  std::vector<double> primal, dual;
  std::vector<int> status;
  int code{ 0 };
  double opt{ 0 };
  bpmpd_output() = default;
  bpmpd_output(std::vector<double> primal, std::vector<double> dual, std::vector<int> status, int code, double opt)
    : primal(std::move(primal)), dual(std::move(dual)), status(std::move(status)), code(code), opt(opt)
  {
  }
};

inline void ser(int fp, bpmpd_output& bo, SerMode mode)
{
  char scorrect = CHECK_CHAR, s = (mode == SER) ? scorrect : 0;  // NOLINT
  ser(fp, s, mode);
  if (s == EXIT_CHAR)
  {
    exit(0);
  }
  ser(fp, bo.primal, mode);
  ser(fp, bo.dual, mode);
  ser(fp, bo.status, mode);
  ser(fp, bo.code, mode);
  ser(fp, bo.opt, mode);
}
}  // namespace bpmpd_io
