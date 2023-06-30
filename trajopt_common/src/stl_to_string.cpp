#include <trajopt_common/stl_to_string.hpp>

namespace
{
template <class T>
std::string Str_impl(const std::vector<T>& x)
{
  std::stringstream ss;
  ss << "(";
  if (!x.empty())
    ss << x[0];
  for (size_t i = 1; i < x.size(); ++i)
    ss << ", " << x[i];
  ss << ")";
  return ss.str();
}
}  // namespace

namespace trajopt_common
{
std::string Str(const std::vector<double>& x) { return Str_impl(x); }
std::string Str(const std::vector<float>& x) { return Str_impl(x); }
std::string Str(const std::vector<int>& x) { return Str_impl(x); }
}  // namespace trajopt_common
