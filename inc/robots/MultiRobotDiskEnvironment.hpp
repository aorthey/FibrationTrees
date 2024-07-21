#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RN_RM.h>

using namespace ompl::base;
using namespace ompl::multilevel;

class MultiRobotDiskEnvironment {
 public:
  explicit MultiRobotDiskEnvironment(size_t N_robots, float radius_disk_robot) {
    N_dimension_ = N_robots * 2;
    N_robots_ = N_robots;
    radius_disk_robot_ = radius_disk_robot;
  }

  static bool robotsCollide(const size_t nRobots, const double values[])
  {
    for(size_t robot1 = 0; robot1 < nRobots; robot1++)
    {
      const double& r1x = values[robot1*2];
      const double& r1y = values[robot1*2+1];
      if(std::sqrt(std::pow(r1x, 2)+std::pow(r1y, 2)) < 0.5) {
        return true;
      }
      for(size_t robot2 = 0; robot2 < nRobots; robot2++)
      {
        if(robot1 == robot2) {
          continue;
        }
        const double& r2x = values[robot2*2];
        const double& r2y = values[robot2*2+1];

        float distance = std::sqrt(std::pow(r1x-r2x, 2) + std::pow(r1y-r2y, 2));
        if(distance < 2*radius_disk_robot_) {
          return true;
        }
      }
    }
    return false;
  }

  static bool isStateValid_ComponentSpace(const State *state)
  {
      const auto *RN = state->as<RealVectorStateSpace::StateType>();
      return !robotsCollide(N_robots_, RN->values);
  }

  ScopedState<> CreateStartStates(const ompl::base::SpaceInformationPtr& si)
  {
    ScopedState<> state(si->getStateSpace());

    float step_size = 2*M_PI / (N_robots_);

    for(size_t k = 0; k < N_robots_; k++)
    {
      float position = k * step_size;
      state[2*k] = std::cos(position);
      state[2*k+1] = std::sin(position);
      OMPL_INFORM("Start state for robot %d is at (%f,%f)", k, state[2*k], state[2*k+1]);
    }
    return state;
  }

  ScopedState<> CreateGoalStates(const ompl::base::SpaceInformationPtr& si)
  {
    ScopedState<> state(si->getStateSpace());

    float step_size = 2*M_PI / (N_robots_);

    for(size_t k = 0; k < N_robots_; k++)
    {
      float position = k * step_size;
      if(position > M_PI) position -= M_PI;
      else position += M_PI;
      state[2*k] = std::cos(position);
      state[2*k+1] = std::sin(position);
      OMPL_INFORM("Goal state for robot %d is at (%f,%f)", k, state[2*k], state[2*k+1]);
    }
    return state;
  }

  FactoredSpaceInformationPtr constructComponentSpace()
  {
      auto component_space = std::make_shared<RealVectorStateSpace>(N_dimension_);
      component_space->setBounds(lower_bound_, upper_bound_);
      component_space->setName("SpaceComponentSpace");
      auto factor(std::make_shared<FactoredSpaceInformation>(component_space));
      factor->setStateValidityChecker(MultiRobotDiskEnvironment::isStateValid_ComponentSpace);
      return factor;
  }

  FactoredSpaceInformationPtr constructDecompositionFactorTree()
  {
      auto factor = constructComponentSpace();

      for(size_t k = 0; k < N_robots_; k++)
      {
        auto robot_space(std::make_shared<RealVectorStateSpace>(2));
        robot_space->setBounds(lower_bound_, upper_bound_);
        robot_space->setName("SpaceRobot"+std::to_string(k));
        auto robot_factor(std::make_shared<FactoredSpaceInformation>(robot_space));
        robot_factor->setStateValidityChecker(
          [](const State *state) -> bool
          {
            return true;
          }
        );
        auto projection = std::make_shared<Projection_RN_RM>(factor->getStateSpace(), robot_space, std::vector<size_t>({2*k, 2*k+1}));
        factor->addChild(robot_factor, projection);
      }
      factor->setStateValidityChecker(
        [](const State *state) -> bool
        {
            const auto *RN = state->as<RealVectorStateSpace::StateType>();
            return !robotsCollide(N_robots_, RN->values);
        }
      );
      return factor;
  }

  FactoredSpaceInformationPtr constructPrioritizedFactorTree()
  {
      auto factor = constructComponentSpace();

      FactoredSpaceInformationPtr current = factor;

      for(size_t k = 0; k < N_robots_ - 1; k++)
      {
        auto subdimension = N_dimension_ - 2*(k+1);
        auto robot_space(std::make_shared<RealVectorStateSpace>(subdimension));
        robot_space->setBounds(lower_bound_, upper_bound_);
        std::string name = "SpaceRobot";
        const auto Nrobots = N_robots_ - k - 1;
        for(size_t j = 0; j < Nrobots; j++)
        {
          name+= std::to_string(j);
        }
        robot_space->setName(name);
        auto robot_factor(std::make_shared<FactoredSpaceInformation>(robot_space));

        robot_factor->setStateValidityChecker(
          [Nrobots](const State *state) -> bool
          {
              const auto *RN = state->as<RealVectorStateSpace::StateType>();
              return !robotsCollide(Nrobots, RN->values);
          }
        );

        auto projection = std::make_shared<Projection_RN_RM>(current->getStateSpace(), robot_space);
        current->addChild(robot_factor, projection);
        current = robot_factor;
      }
      return factor;
  }

 private:

  float lower_bound_{-1};
  float upper_bound_{+1};
  size_t N_dimension_;
  static size_t N_robots_;
  static float radius_disk_robot_;
};

size_t MultiRobotDiskEnvironment::N_robots_ = 1;
float MultiRobotDiskEnvironment::radius_disk_robot_ = 0.1;
