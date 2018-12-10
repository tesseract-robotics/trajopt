#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <boost/format.hpp>
#include <jsoncpp/json/json.h>
#include <sstream>
#include <string>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

namespace json_marshal
{
TRAJOPT_API void fromJson(const Json::Value& v, bool& ref);
TRAJOPT_API void fromJson(const Json::Value& v, int& ref);
TRAJOPT_API void fromJson(const Json::Value& v, double& ref);
TRAJOPT_API void fromJson(const Json::Value& v, std::string& ref);
TRAJOPT_API void fromJson(const Json::Value& v, Eigen::Vector3d& ref);
TRAJOPT_API void fromJson(const Json::Value& v, Eigen::Vector4d& ref);

template <class T>
inline void fromJson(const Json::Value& v, T& ref)
{
  ref.fromJson(v);
}

template <class T>
void fromJsonArray(const Json::Value& parent, std::vector<T>& ref)
{
  ref.clear();
  ref.reserve(parent.size());
  for (Json::Value::const_iterator it = parent.begin(); it != parent.end(); ++it)
  {
    T t;
    fromJson(*it, t);
    ref.push_back(t);
  }
}

template <class T>
void fromJsonArray(const Json::Value& parent, std::vector<T>& ref, int size)
{
  if (static_cast<int>(parent.size()) != size)
  {
    PRINT_AND_THROW(boost::format("expected list of size size %i. got: %s\n") % size % parent);
  }
  else
  {
    fromJsonArray(parent, ref);
  }
}

template <class T>
inline void fromJson(const Json::Value& v, std::vector<T>& ref)
{
  fromJsonArray(v, ref);
}

template <class T>
void childFromJson(const Json::Value& parent, T& ref, const char* name, const T& df)
{
  if (parent.isMember(name))
  {
    const Json::Value& v = parent[name];
    fromJson(v, ref);
  }
  else
  {
    ref = df;
  }
}

template <class T>
void childFromJson(const Json::Value& parent, T& ref, const char* name)
{
  if (parent.isMember(name))
  {
    const Json::Value& v = parent[name];
    fromJson(v, ref);
  }
  else
  {
    PRINT_AND_THROW(boost::format("missing field: %s") % name);
  }
}
}
