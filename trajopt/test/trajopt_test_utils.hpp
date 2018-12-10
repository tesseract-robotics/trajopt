#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <json/json.h>
#include <stdexcept>
TRAJOPT_IGNORE_WARNINGS_POP

Json::Value readJsonFile(const std::string& fname)
{
  Json::Value root;
  Json::Reader reader;
  std::ifstream fh(fname.c_str());
  bool parse_success = reader.parse(fh, root);
  if (!parse_success)
    throw std::runtime_error("failed to parse " + fname);
  return root;
}
