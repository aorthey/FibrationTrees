#include "yaml/MakeViewerFromYaml.hpp"


::osg::Vec3 MakeVector(const YAML::Node& node, const std::string& item_name) {
  if(node["viewer"]["camera"][item_name]) {
    auto item = node["viewer"]["camera"][item_name].as<std::vector<double>>();
    if(item.size() != 3) {
      throw std::out_of_range("Item " + item_name + " of camera has to be a 3D vector, but has dimension " + std::to_string(item.size()));
    }
    return ::osg::Vec3(item.at(0), item.at(1), item.at(2));
  }
}
Visualizer MakeViewerFromYamlFilename(const std::string& filename,
    const dart::simulation::WorldPtr& world) {

  YAML::Node node = YAML::LoadFile(filename);
  if(!node["viewer"]) {
    return Visualizer(world);
  }

  if(!node["viewer"]["camera"]) {
    return Visualizer(world);
  }

  CameraData data;
  if(node["viewer"]["camera"]["eye"]) {
    data.eye = MakeVector(node, "eye");
  }
  if(node["viewer"]["camera"]["up"]) {
    data.up = MakeVector(node, "up");
  }
  if(node["viewer"]["camera"]["center"]) {
    data.center = MakeVector(node, "center");
  }
  return Visualizer(world, data);
}
