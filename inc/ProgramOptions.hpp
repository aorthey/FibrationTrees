#include <boost/program_options.hpp>

#include <iostream>

struct ProgramOptions {
 public:
  ProgramOptions();
  bool Setup(int argc, char* argv[]);

  template<typename T>
  T Get(const std::string& name);

  bool HasValue(const std::string& name);
 private:
  boost::program_options::variables_map variables_map_;
};

template<typename T>
T ProgramOptions::Get(const std::string& name) {
  if (!variables_map_.count(name))
  {
    std::cout << "Argument " << name << " does not exist." << std::endl;
    throw std::out_of_range("Name does not exist.");
  }
  return variables_map_[name].as<T>();
}
