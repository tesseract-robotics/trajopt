#include <trajopt_ros/utils/config.hpp>
using namespace std;

namespace util {

void CommandParser::read(int argc, char* argv[]) {
  // create boost options_description based on variables, parser
  po::options_description od;
  od.add_options()("help,h", "produce help message");
  for (int i=0; i < configs.size(); ++i) {
    for (int j = 0; j < configs[i].params.size(); ++j) {
      configs[i].params[j]->addToBoost(od);
    }
  }
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv)
      .options(od)
      .run()
      , vm);
  if (vm.count("help")) {
    std::cout << "usage: " << argv[0] << " [options]" << std::endl;
    std::cout << od << std::endl;
    exit(0);
  }
  po::notify(vm);
}


}
