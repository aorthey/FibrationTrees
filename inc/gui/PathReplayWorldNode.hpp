#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/external/imgui/imgui.h>
#include <ompl/base/Path.h>
#include <ompl/base/PlannerData.h>

#include "CollisionChecker.hpp"
#include "EigenPath.hpp"

const State3d kRoadmapColorVertex = State3d(0.2, 0.8, 0.2);
const double kRoadmapLineWidth = 3.0;
const double kPathLineWidth = 5.0;

const double kDefaultStepSize = 0.001;

const double kMaxStepSize = 0.1;
const double kMinStepSize = 0.00001;

struct KeyPressEvent {
  char key;
  std::string description;
  std::function<void()> function;
};

class PathReplayWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  PathReplayWorldNode() = delete;
  explicit PathReplayWorldNode(dart::simulation::WorldPtr world);

  ~PathReplayWorldNode();

  void customPreRefresh() override;
  void customPostRefresh() override;
  void customPreStep() override;
  void customPostStep() override;
  void toggleStartStop();
  void toggleReverse();

  void decreaseSpeed();
  void increaseSpeed();

  void setEndTime(double end_time);

  void togglePlannerDataVisibility();
  void toggleSolutionPathVisibility();
  void toggleFrameVisibility(const std::vector<std::string>& frame_names);

  double getCurrentPosition() const;
  double getStepSize() const;
  bool isRunning() const;
  std::string getCurrentJointConfiguration() const;

  void AddPlannerData (const RobotPtr& robot, const ompl::base::PlannerData& data);
  void AddPath(const RobotPtr& robot, const ompl::base::PathPtr& path, const State3d& color);

  void SetCollisionChecker(const CollisionCheckerPtr& collision_checker);

  void setupViewer() override;

  std::vector<KeyPressEvent> GetKeyPressEvents() const;
  void CreateKeyPressEvents();
  void PrintKeyPressEvents() const;

protected:
  std::vector<KeyPressEvent> events_;

  std::vector<std::string> planner_data_frames_;
  std::vector<std::string> solution_path_frames_;

  std::vector<std::pair<RobotPtr, std::shared_ptr<EigenPath>>> robot_and_path_;
  CollisionCheckerPtr collision_checker_;

  double step_size_{kDefaultStepSize};

  double start_time_;
  double end_time_;
  double current_time_;

  bool pause_;
  bool reverse_;

  std::optional<osg::Matrixd> view_matrix_;
};

