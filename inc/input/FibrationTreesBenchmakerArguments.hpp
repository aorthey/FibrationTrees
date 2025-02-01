#include <boost/program_options.hpp>

#include <iostream>

struct FibrationTreesBenchmakerArguments {
 public:
  FibrationTreesBenchmakerArguments();
  bool Setup(const int argc, const char* argv[]);

  template<typename T>
  T Get(const std::string& name) const;

  bool HasValue(const std::string& name) const;
 private:
  boost::program_options::variables_map variables_map_;
};

template<typename T>
T FibrationTreesBenchmakerArguments::Get(const std::string& name) const {
  if (!variables_map_.count(name))
  {
    std::cout << "Argument " << name << " does not exist." << std::endl;
    throw std::out_of_range("Name does not exist.");
  }
  return variables_map_.at(name).as<T>();
}
