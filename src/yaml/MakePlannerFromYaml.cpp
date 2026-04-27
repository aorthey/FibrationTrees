#include "yaml/MakePlannerFromYaml.hpp"

#include <boost/algorithm/string/predicate.hpp>

#include <ompl/multilevel/planners/RRTtask.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevelqrrt/planners/qrrt/QRRT.h>
#include "planners/DiscreteRRT.hpp"

bool HasParameter(const YAML::Node& node, const ompl::base::PlannerPtr& planner, const std::string& planner_name, const std::string& parameter_name) {
  if(!node["planner_settings"]) {
    return false;
  }

  if(!node["planner_settings"][planner_name]) {
    return false;
  }

  if(!node["planner_settings"][planner_name][parameter_name]) {
    return false;
  }

  return planner->params().hasParam(parameter_name);
}

void SetParameter(const YAML::Node& node, const ompl::base::PlannerPtr& planner, const std::string& planner_name, const std::string& parameter_name) {
  std::cout << planner_name << ", " << parameter_name << std::endl;
  if(!HasParameter(node, planner, planner_name, parameter_name)) {
    return;
  }
  auto pnode = node["planner_settings"][planner_name];
  auto parameter_value = pnode[parameter_name].as<double>();
  planner->params().setParam(parameter_name, std::to_string(parameter_value));
  std::string value_in_planner;
  if(!planner->params().getParam(parameter_name, value_in_planner))
  {
    auto msg = "Could not set parameter " + parameter_name + " to " + value_in_planner + " in planner " + planner_name;
    throw std::runtime_error(msg);
  }
  OMPL_INFORM("Setting planner parameter %s to %f", parameter_name.c_str(), std::stof(value_in_planner));
}

ompl::base::PlannerPtr MakePlannerFromYaml(const std::string& filename, const std::string& planner_name, 
    const ompl::multilevel::FactoredSpaceInformationPtr& factor, const std::unordered_map<std::string, RobotPtr>& child_robots) {
  YAML::Node node = YAML::LoadFile(filename);
  return MakePlannerFromYaml(node, planner_name, factor, child_robots);
}

int extract_discrete_rrt_timeout(const std::string& planner_name)
{
    if (!boost::starts_with(planner_name, "DiscreteRRT"))
        return -1;

    if (planner_name == "DiscreteRRT" || planner_name.length() == 11)
        return -1;

    if (planner_name[11] != '-')
        return -1;

    std::string suffix = planner_name.substr(12);
    try {
        return std::stoi(suffix);
    }
    catch (...) {
        return -1;
    }
}

ompl::base::PlannerPtr MakePlannerFromYaml(const YAML::Node& node, const std::string& planner_name, 
    const ompl::multilevel::FactoredSpaceInformationPtr& factor, const std::unordered_map<std::string, RobotPtr>& child_robots) {

  ompl::base::PlannerPtr planner;

  if(boost::starts_with(planner_name, "FibrationRrt")) {
    //Allow different endings to indicate fibration tree types
    planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);

    auto fplanner = std::static_pointer_cast<ompl::multilevel::FibrationRRT>(planner);
    fplanner->setName(planner_name);
    fplanner->setSmoothIntermediateSolutions(false);
    for(const auto& child_robot : child_robots) {
      fplanner->setLocalSmoothIntermediateSolutions(child_robot.second->GetName(), child_robot.second->ShouldSmoothPath());
    }

    SetParameter(node, planner, planner_name, "goal_bias");
    SetParameter(node, planner, planner_name, "path_restriction_sampling_bias");
    SetParameter(node, planner, planner_name, "path_restriction_surrounding_sampling_bias");
    SetParameter(node, planner, planner_name, "sampling_perturbation_bias");

    if(node["planner_settings"]) {
      if(node["planner_settings"][planner_name]) {
        if(node["planner_settings"][planner_name]["selector_function_type"]) {
          auto selector_function_type = node["planner_settings"][planner_name]["selector_function_type"].as<std::string>();
          if(selector_function_type == "exponential") {
            OMPL_INFORM("Set exponential selector function type.");
            fplanner->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kExponential);
          } else if (selector_function_type == "uniform") {
            OMPL_INFORM("Set uniform selector function type.");
            fplanner->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kUniform);
          } else if (selector_function_type == "last_level") {
            OMPL_INFORM("Set last level selector function type.");
            fplanner->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kLastLevel);
          } else {
            throw std::runtime_error("Unknown selector function type:" + selector_function_type);
          }
        }
        if(node["planner_settings"][planner_name]["use_section_search"]) {
          auto use_section_search = node["planner_settings"][planner_name]["use_section_search"].as<bool>();

          if(use_section_search) {
            fplanner->setEnableSectionSearch();
          } else {
            fplanner->setDisableSectionSearch();
          }

        }
      }
    }


  } else if(planner_name == "QRRT") {
    auto factors = factor->getAllFactors();

    std::vector<ompl::base::SpaceInformationPtr> subfactors;
    for(const auto& subfactor : factors) {
      OMPL_INFORM("Factor %s", subfactor->getName().c_str());

      if(subfactor->hasChildren() && subfactor->getTotalNumChildren() != 1) {
        auto msg = "Factor " + subfactor->getName() + " has " + 
            std::to_string(subfactor->getTotalNumChildren()) + " children, but QRRT only allows prioritizations (with 1 child per factor).";
        throw std::runtime_error(msg);
      }
      subfactors.push_back(subfactor);
    }
    std::reverse(subfactors.begin(), subfactors.end());
    planner = std::make_shared<ompl::multilevelqrrt::QRRT>(subfactors);
  ////////////////////////////////////////////////////////////////////////////////
  } else if(boost::starts_with(planner_name, "DiscreteRRT")) {
    //auto factors = factor->getAllFactors();
    auto children = factor->getChildren();

    if(children.empty()) {
        auto msg = "Factor " + factor->getName() + " has no children, but DiscreteRRT requires a decomposition.";
        throw std::runtime_error(msg);
    }
    for(const auto& child : children) {
      OMPL_INFORM("Factor %s", child->getName().c_str());

      if(child->hasChildren()) {
        auto msg = "Factor " + child->getName() + " has children, but DiscreteRRT only allows decompositions.";
        throw std::runtime_error(msg);
      }
    }
    planner = std::make_shared<ompl::geometric::DiscreteRRT>(factor);
    planner->setName(planner_name);

    auto dplanner = std::static_pointer_cast<ompl::geometric::DiscreteRRT>(planner);
    double timeout = (double) extract_discrete_rrt_timeout(planner_name);
    if (timeout >= 0) {
      dplanner->setRoadmapBuildTime(timeout);
    }
  } else if(planner_name == "RRTtask") {
    planner = std::make_shared<ompl::multilevel::RRTtask>(factor);
  } else if(planner_name == "RRT") {
    planner = std::make_shared<ompl::geometric::RRT>(factor);
    SetParameter(node, planner, planner_name, "goal_bias");
  } else if(planner_name == "EST") {
    planner = std::make_shared<ompl::geometric::EST>(factor);
  } else if(planner_name == "RRTConnect") {
    planner = std::make_shared<ompl::geometric::RRTConnect>(factor);
  } else if(planner_name == "LBTRRT") {
    planner = std::make_shared<ompl::geometric::LBTRRT>(factor);
  } else if(planner_name == "BITstar") {
    planner = std::make_shared<ompl::geometric::BITstar>(factor);
  } else if(planner_name == "BFMT") {
    planner = std::make_shared<ompl::geometric::BFMT>(factor);
  } else if(planner_name == "FMT") {
    planner = std::make_shared<ompl::geometric::FMT>(factor);
  } else  {
    throw std::runtime_error("Unknown planner name:" + planner_name);
  }

  planner->setName(planner_name);

  SetParameter(node, planner, planner_name, "range");
  return planner;
}
