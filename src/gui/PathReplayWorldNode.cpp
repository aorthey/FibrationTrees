#include "gui/PathReplayWorldNode.hpp"

#include <ompl/geometric/PathGeometric.h>

#include "OmplHelper.hpp"
#include "DartHelper.hpp"

PathReplayWorldNode::PathReplayWorldNode(dart::simulation::WorldPtr world)
  : dart::gui::osg::RealTimeWorldNode(std::move(world))
{
  pause_ = true;
  reverse_ = false;
  CreateKeyPressEvents();
  simulate(false);
}

PathReplayWorldNode::~PathReplayWorldNode() {
}

const float kVisualizationStepSize = 0.1; //was 0.01

std::vector<State3d> MakeEdgeVertices(const RobotPtr& robot, const ompl::base::SpaceInformationPtr& si, const ompl::base::State* s1, const ompl::base::State* s2, std::vector<StateXd>& configs) {
  const auto L = si->distance(s1, s2);

  std::vector<State3d> vertices;
  if(L < kVisualizationStepSize) {
    return vertices;
  }

  auto sout = si->allocState();
  for(double d = 0.0; d < L + kVisualizationStepSize; d+= kVisualizationStepSize) {
    si->getStateSpace()->interpolate(s1, s2, d/L, sout);
    const auto config = robot->StateToEigen(sout);
    configs.push_back(config);
    const auto frames = robot->GetFK(config);
    for(const auto& frame : frames) {
      vertices.push_back(frame);
    }
  }
  si->freeState(sout);
  return vertices;
}

void PathReplayWorldNode::AddPath(const RobotPtr& robot, const ompl::base::PathPtr& ompl_path, 
    const State3d& color) {
  if(ompl_path == nullptr) {
    return;
  }
  auto path = std::make_shared<EigenPath>(robot, ompl_path);

  const float t_start = path->GetStartTime();
  const float t_end = path->GetEndTime();

  //Move along path, and compute Tcp positions. Then create line segments from them to display.

  std::vector<std::vector<State3d>> vertices;
  const auto s = path->GetConfigAt(t_start);
  const auto frames = robot->GetFK(s);
  for(size_t k = 0; k < frames.size(); k++) {
    std::vector<State3d> vector;
    vertices.push_back(vector);
  }

  const float kTimingStepSize = 0.01;
  for(float t = t_start; t < t_end+kTimingStepSize; t+=kTimingStepSize) {
    const auto s1 = path->GetConfigAt(t);
    const auto frames = robot->GetFK(s1);
    for(size_t k = 0; k < frames.size(); k++) {
      vertices.at(k).push_back(frames.at(k));
    }
  }
  for(const auto& vertices_frame : vertices) {
    auto frame_line = getWorld()->addSimpleFrame(createLineSegmentFrame(vertices_frame, color, kPathLineWidth));
    solution_path_frames_.push_back(frame_line);
  }
  if(robot_and_path_.empty()) {
    start_time_ = path->GetStartTime();
    end_time_ = path->GetEndTime();
  } else {
    start_time_ = std::min(start_time_, path->GetStartTime());
    end_time_ = std::max(end_time_, path->GetEndTime());
  }
  current_time_ = start_time_;
  robot_and_path_.push_back(std::make_pair(robot, path));
}

void PathReplayWorldNode::AddPlannerData(const RobotPtr& robot, const ompl::base::PlannerData& data) {
  const auto& si = data.getSpaceInformation();
  const auto Nvertices = data.numVertices();
  unsigned int counter = 0;
  for(unsigned int vindex = 0; vindex < Nvertices; vindex++) {
    for(unsigned int windex = 0; windex < Nvertices; windex++) {
      if(!data.edgeExists(vindex, windex)) {
        continue;
      }
      counter++;
      const auto s1 = data.getVertex(vindex).getState();
      const auto s2 = data.getVertex(windex).getState();

      std::vector<StateXd> configs;
      auto vertices = MakeEdgeVertices(robot, si, s1, s2, configs);
      auto frame_line = getWorld()->addSimpleFrame(createLineSegmentFrame(vertices, kRoadmapColorVertex, kRoadmapLineWidth));
      planner_data_frames_.push_back(frame_line);
    }
  }
  OMPL_INFORM("Added %d vertices and %d(%d) edges.", Nvertices, counter, data.numEdges());
  togglePlannerDataVisibility();
}

void PathReplayWorldNode::SetCollisionChecker(const CollisionCheckerPtr& collision_checker) {
  collision_checker_ = collision_checker;
}

void PathReplayWorldNode::toggleSolutionPathVisibility() {
  return toggleFrameVisibility(solution_path_frames_);
}
void PathReplayWorldNode::togglePlannerDataVisibility() {
  return toggleFrameVisibility(planner_data_frames_);
}

void PathReplayWorldNode::toggleFrameVisibility(const std::vector<std::string>& frame_names) {
  for(const auto& name : frame_names) {
    auto frame = getWorld()->getSimpleFrame(name);
    if(frame == nullptr) {
      continue;
    }
    if(!frame->hasVisualAspect()) {
      continue;
    }
    if(frame->getVisualAspect()->isHidden()) {
      frame->getVisualAspect()->show();
    } else {
      frame->getVisualAspect()->hide();
    }
  }
}

void PathReplayWorldNode::customPreRefresh()
{
}

void PathReplayWorldNode::customPostRefresh()
{
}

void PathReplayWorldNode::customPreStep()
{
  for(const auto& [robot, path] : robot_and_path_) {
    auto config = path->GetConfigAt(current_time_);
    robot->SetConfiguration(config);
  }
  if(collision_checker_ != nullptr) {
    if(collision_checker_->IsInCollision()) {
      collision_checker_->PrintCollisionInfo();
    }
  }
}

void PathReplayWorldNode::customPostStep()
{
  if(robot_and_path_.empty()) {
    return;
  }
  if(pause_) {
    return;
  }

  current_time_ += (reverse_ ? -step_size_ : +step_size_);

  if(current_time_ > end_time_) {
    reverse_ = true;
    current_time_ = end_time_;
  }
  if(current_time_ < start_time_) {
    reverse_ = false;
    current_time_ = start_time_;
  }
}

void PathReplayWorldNode::decreaseSpeed() {
  step_size_ *= 0.5;
  if(step_size_ < kMinStepSize) {
    step_size_ = kMinStepSize;
  }
}

void PathReplayWorldNode::increaseSpeed() {
  step_size_ *= 2.0;
  if(step_size_ > kMaxStepSize) {
    step_size_ = kMaxStepSize;
  }
}

void PathReplayWorldNode::toggleStartStop() {
  pause_ = !pause_;
}
void PathReplayWorldNode::toggleReverse() {
  reverse_ = !reverse_;
}

std::string PathReplayWorldNode::getCurrentJointConfiguration() const {
  std::string s;
  std::string delim = "";
  for(const auto& [_, path] : robot_and_path_) {
    auto config = path->GetConfigAt(current_time_);
    s += std::to_string(config);
  }
  return s;
}

float PathReplayWorldNode::getCurrentPosition() const {
  return current_time_;
}

float PathReplayWorldNode::getStepSize() const {
  return step_size_;
}

bool PathReplayWorldNode::isRunning() const {
  return !pause_;
}

void PathReplayWorldNode::setupViewer() {
  if (mViewer)
  {
    for(const auto& event : events_) {
      mViewer->addInstructionText(std::string{event.key} + ": " +event.description + ".\n");
    }
  }
}

std::vector<KeyPressEvent> PathReplayWorldNode::GetKeyPressEvents() const {
  return events_;
}

void PathReplayWorldNode::PrintKeyPressEvents() const {
  std::cout << "Key options " << std::endl;
  for(const auto& event : events_) {
    std::cout << std::string{event.key} << ": " << event.description << std::endl;
  }
  std::cout << "Current view: " << std::to_string(mViewer->getVerticalFieldOfView()) << std::endl;
}

void PathReplayWorldNode::setEndTime(float end_time) {
  end_time_ = end_time;
}

void PathReplayWorldNode::CreateKeyPressEvents() {
  events_.push_back({'h', "display key options", [&](){PrintKeyPressEvents();}});
  events_.push_back({'s', "play/pause planned path", [&](){toggleStartStop();}});
  events_.push_back({'r', "reverse execution direction", [&](){toggleReverse();}});
  events_.push_back({'n', "decrease execution speed", [&](){decreaseSpeed();}});
  events_.push_back({'m', "increase execution speed", [&](){increaseSpeed();}});
  events_.push_back({'1', "show/hide solution path", [&](){toggleSolutionPathVisibility();}});
  events_.push_back({'2', "show/hide planner data", [&](){togglePlannerDataVisibility();}});
  events_.push_back({'8', "store planner path", 
      [&](){
        for(const auto& [robot, path] : robot_and_path_) {
          auto name = "eigen_path_"+robot->GetName();
          path->Save(name);
          OMPL_INFORM("Save path %s", name.c_str()); 
        }
      }});
  events_.push_back({'9', "load planner path", 
      [&](){
        for(const auto& [robot, path] : robot_and_path_) {
          auto name = "fail_eigen_path_"+robot->GetName();
          path->Load(name);
          OMPL_INFORM("Load path %s", name.c_str()); 
        }
      }});
  events_.push_back({'p', "output current robot info", 
      [&](){
        for(const auto& [robot, _] : robot_and_path_) {
          PrintSkeletonInfo(robot->GetSkeleton());
        }
      }});
}

PathReplayEventHandler::PathReplayEventHandler(PathReplayWorldNode* world_node)
{
  world_node_ = world_node;
}

bool PathReplayEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) 
{
  if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
  {
    const auto events = world_node_->GetKeyPressEvents();
    for(const auto& event : events) {
      if(ea.getKey() != event.key) {
        continue;
      }
      event.function();
      return true;
    }
  }
  return false;
}

TextWidget::TextWidget(
      dart::gui::osg::ImGuiViewer* viewer, PathReplayWorldNode* world_node)
    : viewer_(viewer),
      world_node_(world_node)
{
}

void TextWidget::render()
{
  ImGui::SetNextWindowPos(ImVec2(10, 20));
  ImGui::SetNextWindowSize(ImVec2(240, 320));
  ImGui::SetNextWindowBgAlpha(0.5f);
  if (!ImGui::Begin(
          "Path Control",
          nullptr,
          ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
              | ImGuiWindowFlags_HorizontalScrollbar))
  {
    ImGui::End();
    return;
  }

  //ImGui::Text("Path position: %.2f (%s)\nPath speed: %f)", 
      //world_node_->getCurrentPosition(), 
  //ImGui::Text("%s", viewer_->getInstructions().c_str());
  ImGui::Text("Path position: %.2f (%s)\nPath speed: %f)", 
      world_node_->getCurrentPosition(), 
      (world_node_->isRunning() ? "running" : "pause"),
      world_node_->getStepSize());
  //ImGui::Text("Config : %s", world_node_->getCurrentJointConfiguration().c_str());
  ImGui::End();
}
