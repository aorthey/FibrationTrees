#pragma once

#include <yaml-cpp/yaml.h>

#include <dart/dart.hpp>

#include "gui/Visualizer.hpp"

Visualizer MakeViewerFromYamlFilename(const std::string& filename,
    const dart::simulation::WorldPtr& world);
