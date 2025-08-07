#include "yaml/MakeProjectionsFromYaml.hpp"

#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/datastructures/projections/FiberedSubspaceProjection.h>
#include <ompl/multilevel/datastructures/projections/RNSO2ToRNProjection.h>
#include <ompl/multilevel/datastructures/projections/SE2RNToR2Projection.h>
#include <ompl/multilevel/datastructures/projections/SE2RNTimeToR2TimeProjection.h>
#include <ompl/multilevel/datastructures/projections/SE2RNToSE2Projection.h>
#include <ompl/multilevel/datastructures/projections/RNToRMProjection.h>
#include <ompl/multilevel/datastructures/projections/R3R2SO2ToR3Projection.h>
#include <ompl/multilevel/datastructures/projections/R3SO2ToR3Projection.h>

#include <ompl/multilevel/datastructures/projections/XR3R2SO2ToXR3Projection.h>
#include <ompl/multilevel/datastructures/projections/XR3SO2ToXR3Projection.h>
#include <ompl/multilevel/datastructures/projections/XSE2RNToXR2Projection.h>

#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/XTimeToXProjection.h>

#include "robots/Robot.hpp"
#include "projections/TaskSpaceProjection.hpp"

std::string GetRootRobotNameFromYamlFilename(const std::string& filename) {

  YAML::Node config = YAML::LoadFile(filename);
  const auto yaml_robots = config["robots"];

  for(const auto& yaml_robot : yaml_robots) {
    auto node = yaml_robot.second;
    if(node["root"]) {
      if(node["root"].as<bool>()) {
        return yaml_robot.first.as<std::string>();
      }
    }
  }
  OMPL_ERROR("Requires at least one root robot in yaml file.");
  throw std::out_of_range("Requires at least one root robot in yaml file.");
}

ompl::multilevel::ProjectionPtr MakeProjectionFromNode(const YAML::Node& node, const ompl::base::SpaceInformationPtr& parent, const ompl::base::SpaceInformationPtr& child, const RobotPtr& parent_robot) {
  if(!node["name"]) {
    throw std::domain_error("Requires a name to create a projection.");
  }
  const std::string name = node["name"].as<std::string>();
  if(name == "ProjectionTaskSpace") {
    return std::make_shared<TaskSpaceProjection>(parent, child, parent_robot);
  } else if(name == "Projection_RNSO2_RN") {
    return std::make_shared<ompl::multilevel::RNSO2ToRNProjection>(parent->getStateSpace(), child->getStateSpace());

  } else if(name == "Projection_R3R2SO2_R3") {
    return std::make_shared<ompl::multilevel::R3R2SO2ToR3Projection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_XR3R2SO2_XR3") {
    return std::make_shared<ompl::multilevel::XR3R2SO2ToXR3Projection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_R3SO2_R3") {
    return std::make_shared<ompl::multilevel::R3SO2ToR3Projection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_XR3SO2_XR3") {
    return std::make_shared<ompl::multilevel::XR3SO2ToXR3Projection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_XSE2RN_XR2") {
    return std::make_shared<ompl::multilevel::XSE2RNToXR2Projection>(parent->getStateSpace(), child->getStateSpace());

  } else if(name == "Projection_SE2RN_R2") {
    return std::make_shared<ompl::multilevel::SE2RNToR2Projection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_SE2RNTime_R2Time") {
    return std::make_shared<ompl::multilevel::SE2RNTimeToR2TimeProjection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_SE2RN_SE2") {
    return std::make_shared<ompl::multilevel::SE2RNToSE2Projection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_RN_RM") {
    return std::make_shared<ompl::multilevel::RNToRMProjection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_TimeBased") {
    return std::make_shared<ompl::multilevel::XTimeToXProjection>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "ProjectionSubspace") {
    return std::make_shared<ompl::multilevel::FiberedSubspaceProjection>(parent->getStateSpace(), child->getStateSpace());
  } else {
    OMPL_ERROR("Could not find a projection with name %s", name.c_str());
    throw std::domain_error("No projection with this name.");
  }
}

void MakeProjectionsFromYamlFilename(const std::string& filename, const dart::simulation::WorldPtr&, 
    const ompl::multilevel::FactoredSpaceInformationPtr& root, const RobotPtr& root_robot, 
    const std::unordered_map<std::string, RobotPtr>& child_robots) {

  YAML::Node node = YAML::LoadFile(filename);
  auto root_name = GetRootRobotNameFromYamlFilename(filename);

  const auto yaml_projections = node["projections"];
  for(const auto& yaml_projection : yaml_projections) {

    auto node = yaml_projection.second;
    std::string projection_name = node["name"].as<std::string>();
    OMPL_INFORM("Creating projection %s", projection_name.c_str());

    if(node["connection"]) {

      std::pair<std::string, std::string> connection = node["connection"].as<std::pair<std::string, std::string>>();
      OMPL_INFORM("Creating projection %s -> %s", connection.first.c_str(), connection.second.c_str());
      auto parent_robot = (connection.first == root_name ? root_robot : child_robots.at(connection.first));
      auto parent = (connection.first == root_name ? root : parent_robot->GetSpaceInformation());
      auto child_robot = child_robots.at(connection.second);
      auto child = child_robot->GetSpaceInformation();
      auto projection = MakeProjectionFromNode(node, parent, child, parent_robot);

      if(!parent->addChild(child, projection)) {
        OMPL_ERROR("Could not add projection for child %s", child->getName().c_str());
        throw std::out_of_range("Unknown projection type.");
      }
    } else if(projection_name == "ProjectionMultiRobot") {
      auto parent_name = node["parent"].as<std::string>();

      if(!node["children"]) {
        OMPL_ERROR("Multi robot projections requires children.");
        throw std::domain_error("Requires children");
      }

      auto parent_robot = (parent_name == root_name ? root_robot : child_robots.at(parent_name));
      auto parent = (parent_name == root_name ? root : parent_robot->GetSpaceInformation());

      auto children_name = node["children"].as<std::vector<std::string>>();

      size_t subspace_index = 0;

      OMPL_INFORM("Creating multi robot projection %s -> ", parent_name.c_str());

      std::vector<RobotPtr> child_robots_of_parent;
      if(children_name.size() == 1) {
        auto child_name = children_name.front();
        auto child_robot = child_robots.at(child_name);
        auto child = child_robot->GetSpaceInformation();
        OMPL_INFORM(" -> %s", child_name.c_str());

        auto projection = std::make_shared<ompl::multilevel::FiberedSubspaceProjection>(parent->getStateSpace(), child->getStateSpace());

        if(!parent->addChild(child, projection, true)) {
          throw std::runtime_error("Could not add child");
        }
        child_robots_of_parent.push_back(child_robot);
      } else {
        for(const auto& child_name : children_name) {
          auto child_robot = child_robots.at(child_name);
          auto child = child_robot->GetSpaceInformation();
          OMPL_INFORM(" -> %s", child_name.c_str());
          child_robots_of_parent.push_back(child_robot);

          auto projection = std::make_shared<ompl::multilevel::SubspaceProjection>(parent->getStateSpace(), child->getStateSpace(), subspace_index);
          subspace_index++;

          if(!parent->addChild(child, projection, false)) {
            throw std::runtime_error("Could not add child");
          }
        }
        // if(node["task_space"]) {
        //   if(node["task_space"].as<bool>()) {
        //     auto motion_validator = std::make_shared<MotionValidatorTaskSpaceMultiRobot>(parent);
        //     parent->setMotionValidator(motion_validator);
        //   }
        // }
      }

    } else {
      OMPL_ERROR("Unknown projection type %s", projection_name.c_str());
      throw std::out_of_range("Unknown projection type.");
    }
  }


}
