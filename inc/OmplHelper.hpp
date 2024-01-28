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
// ompl::base::State* to Eigen::Vector
////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d ProjectStateToEigenVector3d(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::State* state);
Eigen::Vector3d StateToEigenVector3d(const ompl::base::State* state);

////////////////////////////////////////////////////////////////////////////////
// Eigen::Vector to ompl::base::State*
////////////////////////////////////////////////////////////////////////////////
void EigenVector3dToState(const Eigen::Vector3d& v, ompl::base::State* state);
//void EigenVectorXdToState(const Eigen::VectorXd& v, ompl::base::State* state);

ompl::base::GoalPtr GoalFromEigen(const RobotPtr& robot, const Eigen::VectorXd& v, float threshold = 0.1);
ompl::base::State* AllocStateFromEigen(const RobotPtr& robot, const Eigen::VectorXd& v);

////////////////////////////////////////////////////////////////////////////////
// Eigen::Vector to Eigen::Vector
////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd MakeEigen(std::initializer_list<double> const &init_values);
Eigen::VectorXd MakeEigen(std::vector<double> values);

////////////////////////////////////////////////////////////////////////////////
// Misc
////////////////////////////////////////////////////////////////////////////////

bool SampleValidLift(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::SpaceInformationPtr& si, 
    size_t max_iterations, const ompl::base::State *xBase, ompl::base::State *xBundle);

std::optional<ompl::base::State*> ComputeValidIKState(const ompl::base::SpaceInformationPtr& si, 
    const ompl::multilevel::ProjectionPtr& projection, const Eigen::Vector3d& point);
std::optional<ompl::base::State*> ComputeValidTotalState(const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const std::unordered_map<std::string, ompl::base::State*>& leaf_node_states);

ompl::base::PlannerTerminationCondition TimeOrSolutionPtc(const ompl::base::ProblemDefinitionPtr &pdef, double timeout);
