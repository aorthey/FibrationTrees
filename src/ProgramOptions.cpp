#include "ProgramOptions.hpp"

namespace po = boost::program_options;

ProgramOptions::ProgramOptions() {
}

bool ProgramOptions::Setup(int argc, char* argv[]) {
  po::options_description description("Allowed options");
  description.add_options()
      ("help,h", "Show help message")
      ("dry,d", "Dry run: Load everything, but do not plan or visualize")
      ("planning,p", "Plan a path")
      ("timeout,t", po::value<double>()->default_value(10.0), "Timeout for planning in seconds")
      ("benchmark,b", "Do benchmarking")
      ("filename", po::value<std::string>()->required(), "Set filename")
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

bool ProgramOptions::HasValue(const std::string& name) {
  return variables_map_.count(name);
}
