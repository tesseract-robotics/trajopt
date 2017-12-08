#include <vector>

namespace util {

std::vector<int> arange(int n) {
  std::vector<int> out(n);
  for (int i=0; i < n; ++i) out[i] = i;
  return out;
}


}
