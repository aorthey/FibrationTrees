#pragma once

#include <optional>

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "robots/Robot.hpp"

////////////////////////////////////////////////////////////////////////////////
// Eigen::Vector to ompl::base::State*
////////////////////////////////////////////////////////////////////////////////
ompl::base::State* AllocStateFromEigen(const RobotPtr& robot, const StateXd& v);

////////////////////////////////////////////////////////////////////////////////
// Misc
////////////////////////////////////////////////////////////////////////////////

bool SampleValidLift(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::SpaceInformationPtr& si, 
    size_t max_iterations, const ompl::base::State *xBase, ompl::base::State *xBundle);

std::optional<ompl::base::State*> ComputeValidIKState(const ompl::base::SpaceInformationPtr& si, 
    const ompl::multilevel::ProjectionPtr& projection, const State3d& point);
std::optional<ompl::base::State*> ComputeValidTotalState(const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const std::unordered_map<std::string, ompl::base::State*>& leaf_node_states);

ompl::base::PlannerTerminationCondition TimeOrSolutionPtc(const ompl::base::ProblemDefinitionPtr &pdef, double timeout);

std::vector<ompl::base::State*> MakeInterpolatedPathSegment(const ompl::base::SpaceInformation* si, const ompl::base::State *s1, const ompl::base::State *s2);
std::vector<ompl::base::State*> MakeInterpolatedPathSegment(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *s1, const ompl::base::State *s2);
