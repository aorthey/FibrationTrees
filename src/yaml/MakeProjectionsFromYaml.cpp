#include "yaml/MakeProjectionsFromYaml.hpp"

#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/datastructures/projections/RNSO2_RN.h>
#include <ompl/multilevel/datastructures/projections/RN_RM.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/TimeBasedProjection.h>

#include "robots/Robot.hpp"
#include "projections/ProjectionTaskSpace.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"

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
////////////////////////////////////////////////////////////////////////////////
// Make projections
////////////////////////////////////////////////////////////////////////////////

ompl::multilevel::ProjectionPtr MakeProjectionFromName(const std::string& name, const ompl::base::SpaceInformationPtr& parent, const ompl::base::SpaceInformationPtr& child, const RobotPtr& parent_robot) {
  if(name == "ProjectionTaskSpace") {
    return std::make_shared<ProjectionTaskSpace>(parent, child, parent_robot);
  } else if(name == "Projection_RNSO2_RN") {
    return std::make_shared<ompl::multilevel::Projection_RNSO2_RN>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_RN_RM") {
    return std::make_shared<ompl::multilevel::Projection_RN_RM>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_TimeBased") {
    return std::make_shared<ompl::multilevel::Projection_TimeBased>(parent->getStateSpace(), child->getStateSpace());
  } else {
    OMPL_ERROR("Could not find a projection with name %s", name.c_str());
    throw std::domain_error("No projection with this name.");
  }
}

void MakeProjectionsFromYamlFilename(const std::string& filename, const dart::simulation::WorldPtr& world, 
    const ompl::multilevel::FactoredSpaceInformationPtr& root, const RobotPtr& root_robot, 
    const std::unordered_map<std::string, RobotPtr>& child_robots) {

  YAML::Node node = YAML::LoadFile(filename);
  auto root_name = GetRootRobotNameFromYamlFilename(filename);

  const auto yaml_projections = node["projections"];
  for(const auto& yaml_projection : yaml_projections) {

    auto node = yaml_projection.second;
    std::string projection_name = node["name"].as<std::string>();
    OMPL_INFORM("Loading projection %s", projection_name.c_str());

    //if(projection_name == "ProjectionTaskSpace") {
    if(node["connection"]) {
      //Connection projection
      // if(!node["connection"]) {
      //   throw std::domain_error("No connection specified for "+projection_name);
      // }
      std::pair<std::string, std::string> connection = node["connection"].as<std::pair<std::string, std::string>>();
      OMPL_INFORM("Loading projection %s -> %s", connection.first.c_str(), connection.second.c_str());
      auto parent_robot = (connection.first == root_name ? root_robot : child_robots.at(connection.first));
      auto parent = (connection.first == root_name ? root : parent_robot->GetSpaceInformation());
      auto child_robot = child_robots.at(connection.second);
      auto child = child_robot->GetSpaceInformation();
      auto projection = MakeProjectionFromName(projection_name, parent, child, parent_robot);

      if(!parent->addChild(child, projection)) {
        OMPL_ERROR("Could not add projection for child %s", child->getName().c_str());
        throw std::out_of_range("Unknown projection type.");
      }

    } else if(projection_name == "ProjectionMultiRobot") {
      auto parent = node["parent"].as<std::string>();

      if(parent != root_name) {
        OMPL_ERROR("Multi robot projections are only possible on the root node for now.");
        throw std::out_of_range("Non-root MultiRobot projection");
      }
      if(!node["children"]) {
        OMPL_ERROR("Multi robot projections requires children.");
        throw std::domain_error("Requires children");
      }

      auto children_name = node["children"].as<std::vector<std::string>>();

      size_t subspace_index = 0;
      bool computer_fiber_space = false;

      OMPL_INFORM("Loading multi robot projection %s -> ", parent.c_str());
      std::vector<RobotPtr> child_robots_of_root;

      for(const auto& child_name : children_name) {
        auto child_robot = child_robots.at(child_name);
        auto child = child_robot->GetSpaceInformation();
        OMPL_INFORM(" -> %s", child_name.c_str());
        child_robots_of_root.push_back(child_robot);

        auto projection = std::make_shared<ompl::multilevel::Projection_Subspace>(root->getStateSpace(), child->getStateSpace(), subspace_index);
        subspace_index++;

        if(!root->addChild(child, projection, computer_fiber_space)) {
          OMPL_ERROR("Could not add child");
          throw std::out_of_range("Could not add child");
        }

      }
      auto pairwise_collision_checker = std::make_shared<DartMultiRobotCollisionChecker>(root, world, child_robots_of_root);
      root->setStateValidityChecker(pairwise_collision_checker);
      root->setStateValidityCheckingResolution(0.001);

      if(node["task_space"]) {
        if(node["task_space"].as<bool>()) {
          auto motion_validator = std::make_shared<MotionValidatorTaskSpaceMultiRobot>(root);
          root->setMotionValidator(motion_validator);
        }
      }

    } else if(projection_name == "ProjectionMultiRobotToMultiRobot") {
      throw std::invalid_argument("NYI");







    } else {
      OMPL_ERROR("Unknown projection type %s", projection_name.c_str());
      throw std::out_of_range("Unknown projection type.");
    }
  }


}
