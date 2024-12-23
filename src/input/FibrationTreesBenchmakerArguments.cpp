#include "input/FibrationTreesBenchmakerArguments.hpp"

namespace po = boost::program_options;

FibrationTreesBenchmakerArguments::FibrationTreesBenchmakerArguments() {
}

bool FibrationTreesBenchmakerArguments::Setup(const int argc, const char* argv[]) {
  po::options_description description("Allowed options");
  description.add_options()
      ("help,h", "Show help message")
      ("dry,d", "Dry run: Do only 1 run with timeout of 1 second for each scenario and planner.")
      ("filename", po::value<std::string>()->required(), "Set benchmark filename")
  ;

  po::positional_options_description p;
  p.add("filename", -1);

  po::store(po::command_line_parser(argc, argv).options(description).positional(p).run(), variables_map_);
  po::notify(variables_map_);

  if (variables_map_.count("help")) {
    std::cout << description << std::endl;
    return false;
  }

  if (!variables_map_.count("filename"))
  {
    std::cout << description << std::endl;
    return false;
  }

  return true;
};

bool FibrationTreesBenchmakerArguments::HasValue(const std::string& name) {
  return variables_map_.count(name);
}
