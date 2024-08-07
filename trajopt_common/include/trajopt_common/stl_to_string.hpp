#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_common
{
// std::string Str(const vector<double>& x);
// std::string Str(const vector<float>& x);
// std::string Str(const vector<int>& x);

template <class T>
std::string Str(const std::vector<T>& x)
{
  std::stringstream ss;
  ss << "(";
  if (!x.empty())
    ss << x[0];
  for (std::size_t i = 1; i < x.size(); ++i)
    ss << ", " << x[i];
  ss << ")";
  return ss.str();
}

template <class T>
std::string Str(const std::set<T>& x)
{
  std::stringstream ss;
  ss << "{";
  typename std::set<T>::const_iterator it = x.begin();
  if (x.size() > 0)
  {
    ss << *it;
    ++it;
    for (; it != x.end(); ++it)
      ss << ", " << *it;
  }
  ss << "}";
  return ss.str();
}

template <class T>
std::string Str(const T& x)
{
  std::stringstream ss;
  ss << x;
  return ss.str();
}
#define CSTR(x) trajopt_common::Str(x).c_str()

template <class K, class V>
std::string Str(const typename std::map<K, V>& x)
{
  std::stringstream ss;
  ss << "{";
  typename std::map<K, V>::const_iterator it = x.begin();
  if (x.size() > 0)
  {
    ss << Str(it->first) << " : " << Str(it->second);
    ++it;
    for (; it != x.end(); ++it)
      ss << ", " << Str(it->first) << " : " << Str(it->second);
  }
  ss << "}";
  return ss.str();
}
}  // namespace trajopt_common
