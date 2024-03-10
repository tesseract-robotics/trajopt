#pragma once

#include <functional>
#include <memory>

namespace sco
{
class OptProb;
struct OptResults;
}  // namespace sco

namespace trajopt
{
class TrajOptProb;
std::function<void(sco::OptProb*, sco::OptResults&)> WriteCallback(std::shared_ptr<std::ofstream> file,
                                                                   const std::shared_ptr<TrajOptProb>& prob);
}  // namespace trajopt
