#include "dart/dart.hpp"
#include "dart/gui/osg/osg.hpp"
#include "dart/utils/urdf/urdf.hpp"

#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "TaskSpaceProjection.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FactorRRT.h>

void AddOmplPathToWorld(const ompl::base::PathPtr& path, const ompl::multilevel::ProjectionPtr& projection, const dart::simulation::WorldPtr& world) {
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());
  auto states = pgeo.getStates();
  for(size_t k =1; k < states.size(); k++) {
    auto s1 = states.at(k-1);
    auto s2 = states.at(k);
    auto v1 = ProjectStateToEigenVector3d(projection, s1);
    auto v2 = ProjectStateToEigenVector3d(projection, s2);
    world->addSimpleFrame(createLineSegmentFrame(v1, v2));
  }
}

class PathReplayWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  PathReplayWorldNode(
      dart::simulation::WorldPtr world,
      const dart::dynamics::SkeletonPtr& manipulator,
      const ompl::base::PathPtr& path,
      const CollisionCheckerPtr& collision_checker)
    : dart::gui::osg::RealTimeWorldNode(std::move(world)),
      manipulator_(std::move(manipulator)), 
      path_(path),
      collision_checker_(collision_checker)
  {
    const auto& si = path_->getSpaceInformation();

    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    pgeo.interpolate(100);
    OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());
    auto states = pgeo.getStates();
    lengths_.clear();
    for(size_t k = 1; k < states.size(); k++) {
      auto s1 = states.at(k-1);
      auto s2 = states.at(k);
      lengths_.push_back(si->distance(s1, s2));
    }
    current_index_ = 0;
    current_position_ = 0.0f;
    step_size_ = 0.0025;

    tmpState_ = si->allocState();
    pause_ = true;
    reverse_ = false;

  }

  ~PathReplayWorldNode() {
    const auto& si = path_->getSpaceInformation();
    si->freeState(tmpState_);
  }

  void customPreRefresh()
  {
  }

  void customPostRefresh()
  {
  }

  void customPreStep()
  {
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path_.get());
    auto states = pgeo.getStates();

    const auto& si = path_->getSpaceInformation();

    if(current_index_ < 0) {
      auto s1 = states.front();
      Eigen::VectorXd config = StateToEigenVectorXd(si, s1);
      manipulator_->setConfiguration(config);
      return;
    }
    if(current_index_ > lengths_.size() - 1) {
      auto s1 = states.back();
      Eigen::VectorXd config = StateToEigenVectorXd(si, s1);
      manipulator_->setConfiguration(config);
      return;
    }

    const auto current_length = lengths_.at(current_index_);

    auto s1 = states.at(current_index_);
    auto s2 = states.at(current_index_+1);
    si->getStateSpace()->interpolate(s1, s2, current_position_ / current_length, tmpState_);
    Eigen::VectorXd config = StateToEigenVectorXd(si, tmpState_);
    manipulator_->setConfiguration(config);
  }

  void customPostStep()
  {
    if(collision_checker_->IsInCollision(getWorld())) {
      std::cout << "Manipulator " << manipulator_->getName() << " is in collision at config " << manipulator_->getConfiguration().mPositions << std::endl;
    }
    if(pause_) {
      return;
    }
    if(current_index_ < 0) {
      current_index_ = 0;
      current_position_ = 0.0f;
      reverse_ = false;
    }
    if(current_index_ > lengths_.size() - 1) {
      reverse_ = true;
      current_index_ = lengths_.size() - 1;
      current_position_ = lengths_.at(current_index_);
    }

    const auto current_length = lengths_.at(current_index_);
    //std::cout << "Current pos : " << current_index_ << "/" << lengths_.size() - 1 << ", " << current_position_ << "/" << current_length << std::endl;
    current_position_ += (reverse_ ? -step_size_ : +step_size_);

    if(current_position_ > current_length) {
      current_position_ = 0.0f;
      current_index_++;
    }
    if(current_position_ < 0.0f) {
      current_index_--;
      if(current_index_ >= 0) {
        current_position_ = lengths_.at(current_index_);
      }
    }
  }

  void toggleStartStop() {
    pause_ = !pause_;
  }
  void toggleReverse() {
    reverse_ = !reverse_;
  }

protected:
  dart::dynamics::SkeletonPtr manipulator_;
  ompl::base::PathPtr path_;
  CollisionCheckerPtr collision_checker_;

  std::vector<float> lengths_;
  int current_index_;
  float current_position_;
  float step_size_;
  ompl::base::State* tmpState_;

  bool pause_;
  bool reverse_;
};

class PathReplayEventHandler : public osgGA::GUIEventHandler
{
public:
  PathReplayEventHandler(PathReplayWorldNode* worldNode)
  {
    mWorldNode = worldNode;
  }

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      if (ea.getKey() == 's')
      {
        std::cout << "Pressed s key" << std::endl;
        mWorldNode->toggleStartStop();
      }
      if (ea.getKey() == 'r')
      {
        std::cout << "Pressed r key" << std::endl;
        mWorldNode->toggleReverse();
      }
    }
    return false;
  }

private:
  PathReplayWorldNode* mWorldNode;
};


Eigen::VectorXd GetPositionAtIKPose(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::Vector3d& translation, const std::string& body_name) {
  const auto& old_translation = skeleton->getBodyNode(body_name)->getTransform().translation();

  if(skeleton->getBodyNode(body_name)==nullptr) {
    std::cout << "Could not get body " << body_name << " in " << skeleton->getName() << std::endl;
    return skeleton->getConfiguration().mPositions;
  }

  auto config = GetRandomPosition(skeleton);
  skeleton->setConfiguration(config);

  auto ik = skeleton->getBodyNode(body_name)->getOrCreateIK();
  ik->getTarget()->setTranslation(translation);

  if(ik->solveAndApply(config, true)) {
    std::cout << "Found ik solution" << std::endl;
  }

  const auto& current_translation = skeleton->getBodyNode(body_name)->getTransform().translation();
  std::cout << "Old translation     : " << old_translation << std::endl;
  std::cout << "Desired Translation : " << translation << std::endl;
  std::cout << "New Translation     : " << current_translation << std::endl;
  std::cout << "IK Error            : " << (translation - current_translation).norm() << std::endl;

  return ik->getPositions();
}

int main(int argc, char* argv[]) {

  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr manipulator
    = loader.parseSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
    // = loader.parseSkeleton("/home/aorthey/git/MotionExplorer/data/robots/kuka_iiwa/model.urdf");

  manipulator->setName("manipulator");
  manipulator->setMobile(false);
  manipulator->setSelfCollisionCheck(false);
  manipulator->setAdjacentBodyCheck(true);
  std::vector<size_t> indices = {0,1,2,3,4,5};
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  manipulator->setPositionLowerLimits(indices, zero);
  manipulator->setPositionUpperLimits(indices, zero);

  PrintSkeletonInfo(manipulator);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto numLinks = manipulator->getNumDofs();
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(numLinks));
  ompl::base::RealVectorBounds bounds(numLinks);
  auto lb = manipulator->getPositionLowerLimits();
  auto ub = manipulator->getPositionUpperLimits();
  for(size_t k =0; k< numLinks; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));

  ompl::base::StateSpacePtr spaceR3(new ompl::base::RealVectorStateSpace(3));
  ompl::base::RealVectorBounds boundsWorkspace(3);
  boundsWorkspace.setLow(-2);
  boundsWorkspace.setHigh(+2);
  spaceR3->as<ompl::base::RealVectorStateSpace>()->setBounds(boundsWorkspace);

  auto child(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(spaceR3));
  //TODO: implement collision checker for both factors
  // child->setStateValidityChecker(std::make_shared<SE2CollisionChecker>(child, &world));
  // child->setStateValidityCheckingResolution(0.001);

  ompl::multilevel::ProjectionPtr projection = std::make_shared<ProjectionJointSpaceToR3>(space, spaceR3, manipulator);
  factor->addChild(child, projection);
  // factor->setStateValidityChecker(std::make_shared<PlanarManipulatorCollisionChecker>(factor, manipulator, &world));
  // factor->setStateValidityCheckingResolution(0.001);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::State *task_start = child->allocState();
  ompl::base::State *task_goal = child->allocState();
  double *start_angles = task_start->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double *goal_angles = task_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  start_angles[0] = +0.8;
  start_angles[1] = +0.3;
  start_angles[2] = 0.3;
  goal_angles[0] = +0.3;
  goal_angles[1] = +0.8;
  goal_angles[2] = 0.3;

  ompl::base::State *start = factor->allocState();
  ompl::base::State *goal = factor->allocState();
  projection->lift(task_start, start);
  projection->lift(task_goal, goal);

  child->freeState(task_start);
  child->freeState(task_goal);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoalState(goal, 1e-3);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr floor = createFloor();
  dart::dynamics::SkeletonPtr c1 = createCylinder(Eigen::Vector3d(+0.5, +0.5, 0), 0.25, 1.5);
  dart::dynamics::SkeletonPtr c2 = createCylinder(Eigen::Vector3d(-0.5, -0.5, 0), 0.25, 1.5);

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator);
  world->addSkeleton(floor);
  world->addSkeleton(c1);
  world->addSkeleton(c2);

  auto start_vector = ProjectStateToEigenVector3d(projection, start);
  auto goal_vector = ProjectStateToEigenVector3d(projection, goal);
  world->addSimpleFrame(createSphereFrame(start_vector));
  world->addSimpleFrame(createSphereFrame(goal_vector));

  factor->freeState(start);
  factor->freeState(goal);

  ////////////////////////////////////////////////////////////////////////////////
  ////Planning
  ////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FactorRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();

  float timeout = 0.1;
  ompl::base::PlannerStatus status = planner->Planner::solve(timeout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Collision checking
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> g1 = {manipulator};
  std::vector<dart::dynamics::SkeletonPtr> g2 = {floor, c1, c2};
  CollisionCheckerPtr collision_checker = std::make_shared<CollisionChecker>(g1, g2);
  if(collision_checker->IsInCollision(world)) {
    std::cout << "Manipulator " << manipulator->getName() << " is in collision at config " << manipulator->getConfiguration().mPositions << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////

  if (!(status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
      status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION))
  {
      return 1;
  }

  dart::gui::osg::Viewer viewer;

  ompl::base::PathPtr path = pdef->getSolutionPath();
  osg::ref_ptr<PathReplayWorldNode> node = new PathReplayWorldNode(world, manipulator, path, collision_checker);

  viewer.addWorldNode(node);

  //TODO: Visualize endeffector path
  AddOmplPathToWorld(path, projection, world);

  viewer.addInstructionText("Press space to play planned path.\n");
  viewer.addEventHandler(new PathReplayEventHandler(node.get()));

  viewer.setUpViewInWindow(0, 0, 640, 480);
  viewer.simulate(true);

  const auto& eye = ::osg::Vec3(3, 0, 2);
  const auto& center = ::osg::Vec3(0, 0, 0.5);
  const auto& up = ::osg::Vec3(0, 0, 1);

  viewer.getCameraManipulator()->setHomePosition(eye, center, up);
  viewer.setCameraManipulator(viewer.getCameraManipulator()); //update 

  viewer.run();

  return 0;
}
