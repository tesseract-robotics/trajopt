#include <Eigen/Core>
#include <vector>

namespace util {

template <typename VectorT>
VectorT fancySlice(const VectorT& x, const std::vector<int>& inds) {
  VectorT out(inds.size());
  for (int i=0; i < inds.size(); ++i) out[i] = x[inds[i]];
  return out;
}

template <typename VectorT>
std::vector<int> flatnonzero(const VectorT& x) {
  std::vector<int> out;
  for (int i=0; i < x.size(); ++i) if (x[i] != 0) out.push_back(i);
  return out;
}


}
