#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <stdexcept>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/json_marshal.hpp>
namespace json_marshal
{
void fromJson(const Json::Value& v, bool& ref)
{
  try
  {
    ref = v.asBool();
  }
  catch (const std::runtime_error&)
  {
    PRINT_AND_THROW(boost::format("expected: %s, got %s") % ("bool") % (v));
  }
}

void fromJson(const Json::Value& v, int& ref)
{
  try
  {
    ref = v.asInt();
  }
  catch (const std::runtime_error&)
  {
    PRINT_AND_THROW(boost::format("expected: %s, got %s") % ("int") % (v));
  }
}

void fromJson(const Json::Value& v, double& ref)
{
  try
  {
    ref = v.asDouble();
  }
  catch (const std::runtime_error&)
  {
    PRINT_AND_THROW(boost::format("expected: %s, got %s") % ("double") % (v));
  }
}

void fromJson(const Json::Value& v, std::string& ref)
{
  try
  {
    ref = v.asString();
  }
  catch (const std::runtime_error&)
  {
    PRINT_AND_THROW(boost::format("expected: %s, got %s") % ("string") % (v));
  }
}

void fromJson(const Json::Value& v, Eigen::Vector3d& ref)
{
  std::vector<double> vx;
  fromJsonArray(v, vx, 3);
  ref = Eigen::Vector3d(vx[0], vx[1], vx[2]);
}

void fromJson(const Json::Value& v, Eigen::Vector4d& ref)
{
  std::vector<double> vx;
  fromJsonArray(v, vx, 4);
  ref = Eigen::Vector4d(vx[0], vx[1], vx[2], vx[3]);
}
}
