/**
 * @file qp_trace.h
 * @brief Optional, environment-gated trace of the convexified QP as handed to the QP solver.
 *
 * Private to trajopt_sqp's sources (not installed). Everything here is file-local, function-body
 * only and touches no public header, so enabling or removing it has no ABI effect.
 *
 * Gate: the environment variable TRAJOPT_TRACE_DIR names a directory; when it is unset the trace
 * is disabled and every use collapses to a single static-bool check. When set, the trace appends
 * to <dir>/qp_trace_<pid>.txt: the NLP term sets (once), digests of the convexified problem
 * (P, q, A, l, u, box) for the first TRAJOPT_TRACE_MAX_QPS convexify sweeps (default 3), and the
 * OSQP-side data digests, settings, run info and primal-solution digest for the first
 * TRAJOPT_TRACE_MAX_QPS QP solves. Digests are FNV-1a over the raw arrays, so two runs can be
 * compared byte-for-byte without storing the matrices.
 *
 * Diagnostic use: proving whether two solves received the same problem (a profile/dictionary
 * mismatch, a scaling difference, a non-deterministic rho adaptation) without instrumenting the
 * caller.
 */
#ifndef TRAJOPT_SQP_QP_TRACE_H
#define TRAJOPT_SQP_QP_TRACE_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <Eigen/Sparse>

namespace trajopt_sqp::qp_trace
{
/** @brief The trace directory from TRAJOPT_TRACE_DIR, or nullptr when disabled. Evaluated once. */
inline const char* dir()
{
  static const char* const d = [] {
    const char* v = std::getenv("TRAJOPT_TRACE_DIR");
    return (v != nullptr && v[0] != '\0') ? v : nullptr;
  }();
  return d;
}

/** @brief Maximum number of QP solves / convexify sweeps traced per process (TRAJOPT_TRACE_MAX_QPS, default 3). */
inline int maxQps()
{
  static const int n = [] {
    const char* v = std::getenv("TRAJOPT_TRACE_MAX_QPS");
    return (v != nullptr) ? std::atoi(v) : 3;
  }();
  return n;
}

/** @brief Open (append) <dir>/<stem>_<pid>.txt, or nullptr when the trace is disabled or the file cannot be opened. */
inline std::FILE* open(const char* stem)
{
  const char* d = dir();
  if (d == nullptr)
    return nullptr;
  const std::string path = std::string(d) + "/" + stem + "_" + std::to_string(::getpid()) + ".txt";
  return std::fopen(path.c_str(), "a");
}

/** @brief FNV-1a over raw bytes, chainable through @p h. */
inline std::uint64_t fnv1a64(const void* data, std::size_t nbytes, std::uint64_t h = 0xcbf29ce484222325ULL)
{
  const auto* p = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < nbytes; ++i)
  {
    h ^= static_cast<std::uint64_t>(p[i]);
    h *= 0x100000001b3ULL;
  }
  return h;
}

/**
 * @brief Digest a row-major sparse matrix.
 *
 * Compressed (the normal case after setFromTriplets/makeCompressed): the three CSR arrays are
 * hashed separately (values, inner indices, outer indices). Uncompressed fallback: one chained
 * hash over (row, col, value) per nonzero in InnerIterator order, reported as @p h_val with
 * @p h_inner and @p h_outer zero. @p fmt names which path ran.
 */
inline void hashSparse(const Eigen::SparseMatrix<double, Eigen::RowMajor>& m,
                       const char*& fmt,
                       std::uint64_t& h_val,
                       std::uint64_t& h_inner,
                       std::uint64_t& h_outer)
{
  using SpMat = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  if (m.isCompressed())
  {
    fmt = "csr-arrays";
    h_val = fnv1a64(m.valuePtr(), sizeof(double) * static_cast<std::size_t>(m.nonZeros()));
    h_inner = fnv1a64(m.innerIndexPtr(), sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(m.nonZeros()));
    h_outer = fnv1a64(m.outerIndexPtr(), sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(m.outerSize() + 1));
    return;
  }
  fmt = "triplet-iter";
  std::uint64_t h = 0xcbf29ce484222325ULL;
  for (Eigen::Index k = 0; k < m.outerSize(); ++k)
  {
    for (SpMat::InnerIterator it(m, k); it; ++it)
    {
      const std::int64_t rc[2] = { static_cast<std::int64_t>(it.row()), static_cast<std::int64_t>(it.col()) };
      const double v = it.value();
      h = fnv1a64(rc, sizeof(rc), h);
      h = fnv1a64(&v, sizeof(v), h);
    }
  }
  h_val = h;
  h_inner = 0;
  h_outer = 0;
}
}  // namespace trajopt_sqp::qp_trace

#endif  // TRAJOPT_SQP_QP_TRACE_H
