#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_utils/stl_to_string.hpp>

namespace util
{
namespace po = boost::program_options;

struct ParameterBase
{
  using Ptr = std::shared_ptr<ParameterBase>;

  std::string m_name;
  std::string m_desc;
  ParameterBase(std::string name, std::string desc) : m_name(std::move(name)), m_desc(std::move(desc)) {}
  virtual ~ParameterBase() = default;
  ParameterBase(const ParameterBase&) = default;
  ParameterBase& operator=(const ParameterBase&) = default;
  ParameterBase(ParameterBase&&) = default;
  ParameterBase& operator=(ParameterBase&&) = default;

  virtual void addToBoost(po::options_description&) = 0;
};

template <typename T>
struct ParameterVec : ParameterBase
{
  std::vector<T>* m_value;
  ParameterVec(const std::string& name, std::vector<T>* value, const std::string& desc)
    : ParameterBase(name, desc), m_value(value)
  {
  }
  void addToBoost(po::options_description& od) override
  {
    od.add_options()(
        m_name.c_str(), po::value(m_value)->default_value(*m_value, Str(*m_value))->multitoken(), m_desc.c_str());
  }
};

template <typename T>
struct Parameter : ParameterBase
{
  T* m_value;
  Parameter(const std::string& name, T* value, const std::string& desc) : ParameterBase(name, desc), m_value(value) {}
  void addToBoost(po::options_description& od) override
  {
    od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value, Str(*m_value)), m_desc.c_str());
  }
};

struct Config
{
  std::vector<ParameterBase::Ptr> params;
  void add(ParameterBase* param) { params.push_back(ParameterBase::Ptr(param)); }
};

struct CommandParser
{
  std::vector<Config> configs;
  void addGroup(const Config& config) { configs.push_back(config); }
  CommandParser(const Config& config) { addGroup(config); }
  void read(int argc, char* argv[]);  // NOLINT
};
}  // namespace util
