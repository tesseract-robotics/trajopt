#include <trajopt_common/config.hpp>

namespace trajopt_common
{
// NOLINTNEXTLINE
void CommandParser::read(int argc, char* argv[])
{
  // create boost options_description based on variables, parser
  po::options_description od;
  od.add_options()("help,h", "produce help message");
  for (auto& config : configs)
  {
    for (auto& param : config.params)
    {
      param->addToBoost(od);
    }
  }
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(od).run(), vm);
  if (vm.count("help") != 0U)
  {
    std::cout << "usage: " << argv[0] << " [options]" << '\n';
    std::cout << od << '\n';
    exit(0);
  }
  po::notify(vm);
}
}  // namespace trajopt_common
