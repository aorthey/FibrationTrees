#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/external/imgui/imgui.h>
#include <ompl/base/Path.h>
#include <ompl/base/PlannerData.h>

#include "CollisionChecker.hpp"
#include "EigenPath.hpp"

const State3d kRoadmapColorVertex = State3d(0.2, 0.8, 0.2);
const float kRoadmapLineWidth = 3.0;
const float kPathLineWidth = 5.0;

const float kDefaultStepSize = 0.001;

const float kMaxStepSize = 0.1;
const float kMinStepSize = 0.00001;

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

  void togglePlannerDataVisibility();
  void toggleSolutionPathVisibility();
  void toggleFrameVisibility(const std::vector<std::string>& frame_names);

  float getCurrentPosition() const;
  float getStepSize() const;
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

  float step_size_{kDefaultStepSize};
  float path_position_;
  bool pause_;
  bool reverse_;
};

class PathReplayEventHandler : public osgGA::GUIEventHandler
{
public:
  PathReplayEventHandler(PathReplayWorldNode* world_node);

  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

private:
  PathReplayWorldNode* world_node_;
};

class TextWidget : public dart::gui::osg::ImGuiWidget
{
public:
  TextWidget(dart::gui::osg::ImGuiViewer* viewer, PathReplayWorldNode* world_node);

  void render() override;

protected:
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer_;
  PathReplayWorldNode* world_node_;
};
