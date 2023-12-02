#pragma once

#include <optional>

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

////////////////////////////////////////////////////////////////////////////////
// ompl::base::State* to Eigen::Vector
////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d StateToEigenVector3d(const ompl::base::State* state);
Eigen::VectorXd StateToEigenVectorXd(const int Ndimension, const ompl::base::State* state);
Eigen::VectorXd StateToEigenVectorXd(const ompl::base::SpaceInformation* si, const ompl::base::State* state);
Eigen::VectorXd StateToEigenVectorXd(const ompl::base::SpaceInformationPtr& si, const ompl::base::State* state);
Eigen::Vector3d ProjectStateToEigenVector3d(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::State* state);
Eigen::VectorXd LiftStateToEigenVectorXd(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::State* base_state);

////////////////////////////////////////////////////////////////////////////////
// Eigen::Vector to ompl::base::State*
////////////////////////////////////////////////////////////////////////////////
void EigenVector3dToState(const Eigen::Vector3d& v, ompl::base::State* state);
void EigenVectorXdToState(const Eigen::VectorXd& v, ompl::base::State* state);

////////////////////////////////////////////////////////////////////////////////
// Misc
////////////////////////////////////////////////////////////////////////////////
bool SampleValidLift(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::SpaceInformationPtr& si, 
    size_t max_iterations, const ompl::base::State *xBase, ompl::base::State *xBundle);
ompl::geometric::PathGeometricPtr PathFromEigenVectors(const std::vector<Eigen::VectorXd>& configs, 
    const ompl::base::SpaceInformationPtr& si);

std::optional<ompl::base::State*> ComputeValidIKState(const ompl::base::SpaceInformationPtr& si, 
    const ompl::multilevel::ProjectionPtr& projection, const Eigen::Vector3d& point);
std::optional<ompl::base::State*> ComputeValidTotalState(const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const std::unordered_map<std::string, ompl::base::State*>& leaf_node_states);
