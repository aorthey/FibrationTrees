#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/external/imgui/imgui.h>
#include <ompl/base/Path.h>
#include <ompl/base/PlannerData.h>

#include "CollisionChecker.hpp"
#include "EigenPath.hpp"
#include "OmplPath.hpp"

const Eigen::Vector3d kRoadmapColorVertex = Eigen::Vector3d(0.2, 0.8, 0.2);
const Eigen::Vector3d kPathColorProjected = Eigen::Vector3d(0.8, 0.2, 0.8);
const float kRoadmapLineWidth = 3.0;
const float kPathLineWidth = 5.0;

const float kStepSize = 0.001;
typedef EigenPath PathType;

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

  void togglePlannerDataVisibility();
  void toggleSolutionPathVisibility();
  void toggleFrameVisibility(const std::vector<std::string>& frame_names);

  float getCurrentPosition() const;
  bool isRunning() const;
  std::string getCurrentJointConfiguration() const;

  void AddPlannerData (const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PlannerData& data);
  void AddSolutionPath(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PathPtr& path);

  void setupViewer() override;

  std::vector<KeyPressEvent> GetKeyPressEvents() const;
  void CreateKeyPressEvents();

protected:
  std::vector<KeyPressEvent> events_;

  std::vector<std::string> planner_data_frames_;
  std::vector<std::string> solution_path_frames_;

  std::vector<std::pair<dart::dynamics::SkeletonPtr, std::shared_ptr<PathType>>> skeleton_and_path_;

  float step_size_{kStepSize};
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
