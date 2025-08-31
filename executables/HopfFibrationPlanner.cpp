#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/special/SphereStateSpace.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <fstream>
#include <boost/bind/bind.hpp>
#include <functional>
#include <yaml-cpp/yaml.h>

#include "projections/HopfFibrationProjection.hpp"
#include "validators/HopfFibrationMotionValidator.hpp"
#include "State.hpp"
#include "FilePath.hpp"

using namespace ompl::base;
using namespace ompl::geometric;

const unsigned int kNumSamplesPerFiber = 100;
const size_t kMaximumIterations = 300;
const float kLocalRange = 10.0;

template <typename T>
struct EigenSearchTree {
  std::vector<std::pair<T, T>> edges;
  std::vector<T> nodes;
  std::optional<T> start;
  std::optional<T> goal;
};

template <typename T>
EigenSearchTree<T> PlannerDataToSearchTree(const PlannerData& data, const std::function<T(const ompl::base::State*)>& toVector) {
  auto N_edges = data.numEdges();
  auto N_vertices = data.numVertices();

  std::cout << "Found " << N_vertices << " vertices and " << N_edges << " edges." << std::endl;

  EigenSearchTree<T> tree;
  auto space = data.getSpaceInformation()->getStateSpace();
  auto start = data.getStartVertex(0);
  if(start != PlannerData::NO_VERTEX) {
    auto start_state = start.getState();
    tree.start = toVector(start_state);
  }
  auto goal = data.getGoalVertex(0);
  if(goal != PlannerData::NO_VERTEX) {
    auto goal_state = goal.getState();
    tree.goal = toVector(goal_state);
  }

  auto old_state = space->allocState();
  auto new_state = space->allocState();
  for(unsigned int i = 0; i < N_vertices; i++) {
    for(unsigned int j = 0; j < N_vertices; j++) {
      if(i==j) {
        continue;
      }
      if(!data.edgeExists(i, j)) {
        continue;
      }
      auto v1 = data.getVertex(i);
      auto v2 = data.getVertex(j);
      auto s1 = v1.getState();
      auto s2 = v2.getState();
      int nd = space->validSegmentCount(s1, s2);
      space->copyState(old_state, s1);
      if (nd > 1)
      {
        for (int k = 0; k <= nd; k++)
        {
          space->interpolate(s1, s2, (double)k / (double)nd, new_state);
          auto p1 = toVector(old_state);
          auto p2 = toVector(new_state);
          auto edge = std::make_pair(p1, p2);
          tree.edges.push_back(edge);
          space->copyState(old_state, new_state);
        }
      } else {
          auto p1 = toVector(s1);
          auto p2 = toVector(s2);
          auto edge = std::make_pair(p1, p2);
          tree.edges.push_back(edge);
      }
    }
  }
  space->freeState(old_state);
  space->freeState(new_state);
  return tree;
};

YAML::Node ToYaml(const Eigen::Vector3f& v) {
  std::stringstream ss;    
  ss << std::fixed << std::setprecision(2) << "[" << v[0] << "," << v[1] << "," << v[2] << "]";
  return YAML::Load(ss.str());
}
YAML::Node ToYaml(const Eigen::Vector4f& v) {
  std::stringstream ss;    
  ss << std::fixed << std::setprecision(2) << "[" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "]";
  return YAML::Load(ss.str());
}

YAML::Node ToYaml(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  YAML::Node node;
  node["source"] = ToYaml(a);
  node["target"] = ToYaml(b);
  return node;
}
YAML::Node ToYaml(const Eigen::Vector4f& a, const Eigen::Vector4f& b) {
  YAML::Node node;
  node["source"] = ToYaml(a);
  node["target"] = ToYaml(b);
  return node;
}

template <typename T>
YAML::Node ToYaml(const EigenSearchTree<T>& tree) {
  YAML::Node node;
  if(tree.start.has_value()) {
    node["start"] = ToYaml(tree.start.value());
  }
  if(tree.goal.has_value()) {
    node["goal"] = ToYaml(tree.goal.value());
  } else {
    node["goal"] = ToYaml(tree.start.value());
  }
  for(const auto& edge : tree.edges) {
    node["edges"].push_back(ToYaml(edge.first, edge.second));
  }
  return node;
}

YAML::Node S3EdgeToYaml(const std::shared_ptr<HopfFibrationProjection>& hopf_fibration, const ompl::base::State* s1, const ompl::base::State* s2) {
  auto bundle_space = hopf_fibration->getBundle();
  auto base_space = hopf_fibration->getBase();
  auto fiber_space = hopf_fibration->getFiberSpace();

  int nd = 0.5*bundle_space->validSegmentCount(s1, s2);

  auto bundleState = bundle_space->allocState();
  auto baseState = base_space->allocState();
  auto fiberState = fiber_space->allocState();

  YAML::Node node;
  if (nd > 1)
  {
    for (int i = 0; i <= nd; i++)
    {
      bundle_space->interpolate(s1, s2, (double)i / (double)nd, bundleState);
      hopf_fibration->project(bundleState, baseState);

      static int fiber_counter = 0;
      const auto name = "fiber" + std::to_string(fiber_counter++);
      for(unsigned int j = 0; j < kNumSamplesPerFiber; j++) {
        auto value = (double(j)/double(kNumSamplesPerFiber)) * 2.0 * M_PI - M_PI;
        fiberState->as<ompl::base::SO2StateSpace::StateType>()->value = value;

        if(!fiber_space->satisfiesBounds(fiberState)) {
          OMPL_ERROR("Invalid bounds");
          fiber_space->printState(fiberState);
          throw "InvalidBounds";
        }
        hopf_fibration->lift(baseState, fiberState, bundleState);

        auto v = hopf_fibration->toVector(bundleState);
        node[name]["states"].push_back(ToYaml(v));
      }
    }
  }

  fiber_space->freeState(fiberState);
  base_space->freeState(baseState);
  bundle_space->freeState(bundleState);

  return node;
}

std::vector<YAML::Node> S2EdgeToYaml(const std::shared_ptr<HopfFibrationProjection>& hopf_fibration, const ompl::base::State* s1, const ompl::base::State* s2) {
  auto bundle_space = hopf_fibration->getBundle();
  auto base_space = hopf_fibration->getBase();
  auto fiber_space = hopf_fibration->getFiberSpace();

  int nd = base_space->validSegmentCount(s1, s2);

  auto bundleState = bundle_space->allocState();
  auto baseState = base_space->allocState();
  auto fiberState = fiber_space->allocState();

  std::vector<YAML::Node> nodes;

  static int fiber_counter = 0;

  if (nd > 1) {
    for (int i = 0; i <= nd; i++) {
      base_space->interpolate(s1, s2, (double)i / (double)nd, baseState);

      const auto name = "fiber" + std::to_string(fiber_counter++);
      YAML::Node fiber_node;
      for (unsigned int j = 0; j < kNumSamplesPerFiber; j++) {
        auto value = (double(j) / double(kNumSamplesPerFiber)) * 2.0 * M_PI - M_PI;
        fiberState->as<ompl::base::SO2StateSpace::StateType>()->value = value;

        if (!fiber_space->satisfiesBounds(fiberState)) {
          OMPL_ERROR("Invalid bounds");
          fiber_space->printState(fiberState);
          throw "InvalidBounds";
        }
        hopf_fibration->lift(baseState, fiberState, bundleState);

        auto v = hopf_fibration->toVector(bundleState);
        fiber_node["states"].push_back(ToYaml(v));
      }
      nodes.push_back(fiber_node);
    }
  }

  fiber_space->freeState(fiberState);
  base_space->freeState(baseState);
  bundle_space->freeState(bundleState);

  return nodes;
}

ompl::base::StateSamplerPtr allocateHopfSphereSampler(const ompl::base::StateSpacePtr& space) {

  //Note: Theta is the azimuthal angle [-pi,+pi] (longitude, going around the equator), 
  //      and phi is the polar angle [0, +pi] (latitude)
  class HopfSphereSampler : public ompl::base::StateSampler {
    public:
      HopfSphereSampler() = delete;
      HopfSphereSampler(const ompl::base::StateSpacePtr& space): ompl::base::StateSampler(space.get()) {}
      void sampleUniform(ompl::base::State *state) override {
        static size_t counter = 0;
        const double theta = _thetaphi.at(counter).first;
        const double phi = _thetaphi.at(counter).second;
        counter++;
        if (counter >= _thetaphi.size()) {
          counter = _thetaphi.size() - 1;
        }
        state->as<ompl::base::SphereStateSpace::StateType>()->setThetaPhi(theta, phi);
        if(!space_->satisfiesBounds(state)) {
          OMPL_ERROR("Invalid bounds");
          space_->printState(state);
          throw "InvalidBounds";
        }

      }
      void sampleUniformNear(ompl::base::State*, const ompl::base::State *, double) override {
        throw "NYI";
      }
      void sampleGaussian(ompl::base::State*, const ompl::base::State*, double) override {
        throw "NYI";
      }
    private:
  // sphere_start->as<SphereStateSpace::StateType>()->setThetaPhi(-0.9*M_PI, 0.1*M_PI);
  // sphere_goal->as<SphereStateSpace::StateType>()->setThetaPhi(+0.9*M_PI, 0.9*M_PI);

      //Vertical slices
  // sphere_start->as<SphereStateSpace::StateType>()->setThetaPhi(-0.8*M_PI, 0.5*M_PI);
  // sphere_goal->as<SphereStateSpace::StateType>()->setThetaPhi(+0.3*M_PI, 0.5*M_PI);
      std::vector<std::pair<double, double>> _thetaphi = {
          //################################################################################
          {-0.8*M_PI, +0.5*M_PI},
          {-0.7*M_PI, +0.5*M_PI},
          {-0.6*M_PI, +0.5*M_PI},
          {-0.5*M_PI, +0.5*M_PI},
          {-0.4*M_PI, +0.5*M_PI},
          {-0.3*M_PI, +0.5*M_PI},
          {-0.2*M_PI, +0.5*M_PI},
          {-0.1*M_PI, +0.5*M_PI},
          {-0.0*M_PI, +0.5*M_PI},
          {+0.1*M_PI, +0.5*M_PI},
          {+0.2*M_PI, +0.5*M_PI},
          {+0.3*M_PI, +0.5*M_PI},
          {+0.4*M_PI, +0.5*M_PI},
          {+0.5*M_PI, +0.5*M_PI},
          {+0.6*M_PI, +0.5*M_PI},
          {+0.7*M_PI, +0.5*M_PI},
          {+0.8*M_PI, +0.5*M_PI},
          {+0.9*M_PI, +0.5*M_PI},
          ////################################################################################
          // {+0.5*M_PI, +0.2*M_PI},
          // {+0.5*M_PI, +0.3*M_PI},
          // {+0.5*M_PI, +0.4*M_PI},
          // {+0.5*M_PI, +0.5*M_PI},
          // {+0.5*M_PI, +0.6*M_PI},
          // {+0.5*M_PI, +0.7*M_PI},
          // {+0.5*M_PI, +0.8*M_PI},
          // {+0.5*M_PI, +0.9*M_PI},
          ////################################################################################
          //{+0.9*M_PI, 0.6*M_PI},
          //{+0.7*M_PI, 0.6*M_PI},
          //{+0.5*M_PI, 0.6*M_PI},
          //{+0.3*M_PI, 0.6*M_PI},
          //{+0.1*M_PI, 0.6*M_PI},
          //{+0.1*M_PI, 0.6*M_PI},
          //{-0.0*M_PI, 0.6*M_PI},
          //{-0.1*M_PI, 0.6*M_PI},
          //{-0.2*M_PI, 0.6*M_PI},
          //{-0.3*M_PI, 0.6*M_PI},
          //{-0.4*M_PI, 0.6*M_PI},
      };

      //Horizontal slices
      //std::vector<std::pair<double, double>> _thetaphi = {
      //    {-0.9*M_PI, 0.3*M_PI},
      //    {-0.7*M_PI, 0.3*M_PI},
      //    {-0.5*M_PI, 0.3*M_PI},
      //    {-0.3*M_PI, 0.3*M_PI},
      //    {-0.1*M_PI, 0.3*M_PI},
      //    {+0.1*M_PI, 0.3*M_PI},
      //    {+0.3*M_PI, 0.3*M_PI},
      //    {+0.5*M_PI, 0.3*M_PI},
      //    {+0.7*M_PI, 0.3*M_PI},
      //    {+0.9*M_PI, 0.3*M_PI},
      //    //################################################################################
      //    {+0.9*M_PI, 0.4*M_PI},
      //    {+0.9*M_PI, 0.5*M_PI},
      //    //################################################################################
      //    {+0.9*M_PI, 0.6*M_PI},
      //    {+0.7*M_PI, 0.6*M_PI},
      //    {+0.5*M_PI, 0.6*M_PI},
      //    {+0.3*M_PI, 0.6*M_PI},
      //    {+0.1*M_PI, 0.6*M_PI},
      //    {-0.1*M_PI, 0.6*M_PI},
      //    {-0.3*M_PI, 0.6*M_PI},
      //    {-0.5*M_PI, 0.6*M_PI},
      //    {-0.6*M_PI, 0.6*M_PI},
      //    {-0.7*M_PI, 0.6*M_PI},
      //    {-0.8*M_PI, 0.6*M_PI},
      //    {-0.9*M_PI, 0.6*M_PI},
      //    //################################################################################
      //    // {-0.9*M_PI, 0.9*M_PI},
      //    // {-0.7*M_PI, 0.9*M_PI},
      //    // {-0.5*M_PI, 0.9*M_PI},
      //    // {-0.3*M_PI, 0.9*M_PI},
      //    // {-0.1*M_PI, 0.9*M_PI},
      //    // {+0.1*M_PI, 0.9*M_PI},
      //    // {+0.3*M_PI, 0.9*M_PI},
      //    // {+0.5*M_PI, 0.9*M_PI},
      //    // {+0.7*M_PI, 0.9*M_PI},
      //    // {+0.9*M_PI, 0.9*M_PI}
      //};
  };
  return std::make_shared<HopfSphereSampler>(space);
}


int main()
{
  auto total_space = std::make_shared<ompl::base::SO3StateSpace>();
  auto base_space = std::make_shared<ompl::base::SphereStateSpace>();

  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(total_space);
  auto child = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(base_space);

  //Set custom sampling sequence for sphere
  child->getStateSpace()->setStateSamplerAllocator(
      std::bind(&allocateHopfSphereSampler, base_space));


  // Add custom state validity checker for the sphere space
  // child->setStateValidityChecker(
  //   [](const ompl::base::State* state) -> bool {
  //     const auto* sphere_state = state->as<ompl::base::SphereStateSpace::StateType>();
  //     double theta = sphere_state->getTheta();
  //     double phi = sphere_state->getPhi();
      
  //     // Create 16 equally distributed spherical obstacles
  //     const int num_obstacles = 8;
  //     const double obstacle_radius = 0.1; // Size of each obstacle
      
  //     // Calculate positions of obstacles
  //     for (int i = 0; i < num_obstacles; i++) {
  //       // Calculate theta and phi for this obstacle
  //       double obs_theta = (2.0 * M_PI * i) / num_obstacles;
  //       double obs_phi = (M_PI * (i % 4)) / 4.0; // Distribute in 4 layers
        
  //       // Calculate spherical distance between current state and obstacle center
  //       double dtheta = std::abs(theta - obs_theta);
  //       if (dtheta > M_PI) dtheta = 2.0 * M_PI - dtheta;
  //       double dphi = std::abs(phi - obs_phi);
        
  //       // Check if point is inside obstacle
  //       if (std::sqrt(dtheta * dtheta + dphi * dphi) < obstacle_radius) {
  //         return false;
  //       }
  //     }
      
  //     return true;
  //   }
  // );

  auto hopf_fibration = std::make_shared<HopfFibrationProjection>(factor, child);
  factor->addChild(child, hopf_fibration);

  auto hopf_motion_validator = std::make_shared<HopfFibrationMotionValidator>(factor, hopf_fibration);
  factor->setMotionValidator(hopf_motion_validator);
  factor->printFactorization(std::cout);

  auto start = factor->allocState();
  auto sphere_start = child->allocState();
  auto goal = total_space->allocState();
  auto sphere_goal = child->allocState();

  // sphere_start->as<SphereStateSpace::StateType>()->setThetaPhi(-0.9*M_PI, 0.3*M_PI);
  // sphere_goal->as<SphereStateSpace::StateType>()->setThetaPhi(+0.9*M_PI, 0.9*M_PI);

  // Horizontal Slices
  // sphere_start->as<SphereStateSpace::StateType>()->setThetaPhi(-0.9*M_PI, 0.3*M_PI);
  // sphere_goal->as<SphereStateSpace::StateType>()->setThetaPhi(-0.9*M_PI, 0.6*M_PI);

  // Vertical Slices
  //sphere_goal->as<SphereStateSpace::StateType>()->setThetaPhi(+0.3*M_PI, 0.5*M_PI);
  sphere_start->as<SphereStateSpace::StateType>()->setThetaPhi(-0.8*M_PI, +0.5*M_PI);
  sphere_goal->as<SphereStateSpace::StateType>()->setThetaPhi( +0.9*M_PI, +0.5*M_PI);
  hopf_fibration->lift(sphere_start, start);
  hopf_fibration->lift(sphere_goal, goal);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->setStartAndGoalStates(start, goal, 0.05);

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setSeed(0);

  planner->setLocalRange(factor->getName(), kLocalRange);
  planner->setLocalGoalBias(factor->getName(), 0.5);
  planner->setSmoothIntermediateSolutions(false);
  planner->setLocalPathRestrictionSamplingBias(factor->getName(), 1.0);
  planner->setLocalPathRestrictionSurroundingSamplingBias(factor->getName(), 0.01);
  planner->setLocalSamplingPerturbationBias(factor->getName(), 0.01);
  planner->setDisableSectionSearch();

  planner->setLocalRange(child->getName(), 100.0);
  planner->setLocalGoalBias(child->getName(), 0.0);
  planner->setLocalPathRestrictionSamplingBias(child->getName(), 1.0);
  planner->setLocalPathRestrictionSurroundingSamplingBias(child->getName(), 0.0);
  planner->setLocalSamplingPerturbationBias(child->getName(), 0.0);

  planner->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kLastLevel);

  //////////////////////////////////////////////////////////////////////////////////////
  //////////Planning
  //////////////////////////////////////////////////////////////////////////////////////
  ompl::base::IterationTerminationCondition itc(kMaximumIterations);
  auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

  ompl::base::PlannerStatus solved = planner->solve(ptc);

  if(!solved) {
    OMPL_ERROR("Found no solution");
  }

  ompl::base::PlannerData sphere_data(child);
  planner->getPlannerData(sphere_data);
  auto sphere_tree = PlannerDataToSearchTree<Eigen::Vector3f>(sphere_data, std::bind( &ompl::base::SphereStateSpace::toVector, base_space, std::placeholders::_1 ));

  ompl::base::PlannerData so3_data(factor);
  planner->getPlannerData(so3_data);
  auto so3_tree = PlannerDataToSearchTree<Eigen::Vector3f>(so3_data, std::bind( &HopfFibrationProjection::toVector, hopf_fibration, std::placeholders::_1 ));

  auto filename = GetDataFolder() + "hopffibration_trees.dat";

  YAML::Node node;
  node["name"] = filename;
  node["trees"]["sphere_tree"] = ToYaml(sphere_tree);
  node["trees"]["so3_tree"] = ToYaml(so3_tree);

  //////////////////////////////////////////////////////////////////////////////////////
  //Samples Fibers belonging to vertices and edges on sphere_tree
  //////////////////////////////////////////////////////////////////////////////////////
  // auto N_vertices = sphere_data.numVertices();

  // for(unsigned int i = 0; i < N_vertices; i++) {
  //   for(unsigned int j = 0; j < N_vertices; j++) {
  //     if(i==j) {
  //       continue;
  //     }
  //     if(!sphere_data.edgeExists(i, j)) {
  //       continue;
  //     }
  //     auto v1 = sphere_data.getVertex(i);
  //     auto v2 = sphere_data.getVertex(j);
  //     node["fibers"].push_back(S2EdgeToYaml(hopf_fibration, v1.getState(), v2.getState()));
  //     child->printState(v1.getState());
  //     child->printState(v2.getState());
  //   }
  // }

  // auto N_vertices = so3_data.numVertices();

  // size_t edge_counter = 0;
  // for(unsigned int i = 0; i < N_vertices; i++) {
  //   for(unsigned int j = 0; j < N_vertices; j++) {
  //     if(i==j) {
  //       continue;
  //     }
  //     if(!so3_data.edgeExists(i, j)) {
  //       continue;
  //     }
  //     auto v1 = so3_data.getVertex(i);
  //     auto v2 = so3_data.getVertex(j);
  //     node["fibers"].push_back(S3EdgeToYaml(hopf_fibration, v1.getState(), v2.getState()));
  //     edge_counter++;
  //   }
  // }
  // std::cout << "Created " << edge_counter << " fibers." << std::endl;

  const auto sphere_path = planner->getProblemDefinition(child->getName())->getSolutionPath()->as<ompl::geometric::PathGeometric>();
  auto states = sphere_path->getStates();
  static int fiber_counter = 0;
  for(unsigned int i = 1; i < states.size(); i++) {
    auto s1 = states.at(i-1);
    auto s2 = states.at(i);
    auto nodes = S2EdgeToYaml(hopf_fibration, s1, s2);
    for(const auto& fiber_node : nodes) {
      YAML::Node named_fiber;
      named_fiber["fiber" + std::to_string(fiber_counter++)] = fiber_node;
      node["fibers"].push_back(named_fiber);
    }
  }
  // for(unsigned int i = 0; i < N_vertices; i++) {
  //   for(unsigned int j = 0; j < N_vertices; j++) {
  //     if(i==j) {
  //       continue;
  //     }
  //     if(!sphere_data.edgeExists(i, j)) {
  //       continue;
  //     }
  //     auto v1 = sphere_data.getVertex(i);
  //     auto v2 = sphere_data.getVertex(j);
  //     node["fibers"].push_back(S2EdgeToYaml(hopf_fibration, v1.getState(), v2.getState()));
  //     child->printState(v1.getState());
  //     child->printState(v2.getState());
  //   }
  // }

  auto N_vertices = so3_data.numVertices();
  auto tmp = child->allocState();
  for(unsigned int i = 0; i < N_vertices; i++) {
    for(unsigned int j = 0; j < N_vertices; j++) {
      if(i==j || !so3_data.edgeExists(i, j)) {
        continue;
      }
      auto v1 = so3_data.getVertex(i);
      hopf_fibration->project(v1.getState(), tmp);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////
  //Write YAML to file
  //////////////////////////////////////////////////////////////////////////////////////
  std::ofstream file;
  file.open(filename);
  file << node;
  file.close();
  std::cout << "Wrote PlannerData to file " << filename << std::endl;

  factor->freeState(start);
  child->freeState(sphere_start);
  total_space->freeState(goal);
  child->freeState(sphere_goal);
  return 0;
}

