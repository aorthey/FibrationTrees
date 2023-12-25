#include "gui/PathReplayWorldNode.hpp"

#include <ompl/geometric/PathGeometric.h>

#include "OmplHelper.hpp"
#include "DartHelper.hpp"

PathReplayWorldNode::PathReplayWorldNode(dart::simulation::WorldPtr world)
  : dart::gui::osg::RealTimeWorldNode(std::move(world))
{
  pause_ = true;
  reverse_ = false;
  path_position_ = 0.0f;
  CreateKeyPressEvents();
}

PathReplayWorldNode::~PathReplayWorldNode() {
}

const float kVisualizationStepSize = 0.01; //was 0.01

std::vector<Eigen::Vector3d> MakeEdgeVertices(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::SpaceInformationPtr& si, const ompl::base::State* s1, const ompl::base::State* s2, std::vector<Eigen::VectorXd>& configs) {
  const auto L = si->distance(s1, s2);

  std::vector<Eigen::Vector3d> vertices;
  auto sout = si->allocState();
  for(double d = 0.0; d < L + kVisualizationStepSize; d+= kVisualizationStepSize) {
    si->getStateSpace()->interpolate(s1, s2, d/L, sout);
    const auto config = StateToEigenVectorXd(si, sout);
    configs.push_back(config);
    const auto v = GetFK(skeleton, config);
    vertices.push_back(v);
  }
  si->freeState(sout);
  return vertices;
}

// void PathReplayWorldNode::AddPath(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PathPtr& ompl_path, 
//     const Eigen::Vector3d& color) {
//   if(ompl_path == nullptr) {
//     return;
//   }
//   auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(ompl_path->getSpaceInformation());
//   ompl::geometric::PathGeometric &path = *static_cast<ompl::geometric::PathGeometric *>(ompl_path.get());

//   auto states = path.getStates();

//   std::vector<Eigen::VectorXd> configs;
//   std::cout << "Path has " << states.size() << " states." << std::endl;
//   for(size_t k = 1; k < states.size(); k++) {
//     auto s1 = states.at(k - 1);
//     auto s2 = states.at(k);

//     std::vector<Eigen::VectorXd> edge_configs;
//     std::cout << "Create edge " << k << "/" << states.size() - 1 << std::endl;
//     auto vertices = MakeEdgeVertices(skeleton, factor, s1, s2, edge_configs);
//     configs.insert(configs.end(), edge_configs.begin(), edge_configs.end());

//     auto frame_line = getWorld()->addSimpleFrame(createLineSegmentFrame(vertices, color, kPathLineWidth));
//     solution_path_frames_.push_back(frame_line);
//   }
//   std::cout << "Found " << configs.size() << " configs for path with " << states.size() << " states." << std::endl;
//   auto eigen_path = std::make_shared<PathType>(configs);
//   skeleton_and_path_.push_back(std::make_pair(skeleton, eigen_path));
// }
void PathReplayWorldNode::AddPath(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PathPtr& ompl_path, 
    const Eigen::Vector3d& color) {
  if(ompl_path == nullptr) {
    return;
  }
  auto path = std::make_shared<PathType>(ompl_path);

  const float L = path->GetLength();
  std::vector<Eigen::Vector3d> vertices;
  for(float d = 0; d < L+kVisualizationStepSize; d+=kVisualizationStepSize) {
    const auto s1 = path->GetConfigAt(d / L);
    const auto v1 = GetFK(skeleton, s1);
    vertices.push_back(v1);
  }
  auto frame_line = getWorld()->addSimpleFrame(createLineSegmentFrame(vertices, color, kPathLineWidth));
  solution_path_frames_.push_back(frame_line);
  skeleton_and_path_.push_back(std::make_pair(skeleton, path));
}

void PathReplayWorldNode::AddPlannerData(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PlannerData& data) {
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

      std::vector<Eigen::VectorXd> configs;
      auto vertices = MakeEdgeVertices(skeleton, si, s1, s2, configs);
      auto frame_line = getWorld()->addSimpleFrame(createLineSegmentFrame(vertices, kRoadmapColorVertex, kRoadmapLineWidth));
      planner_data_frames_.push_back(frame_line);
    }
  }
  OMPL_INFORM("Added %d vertices and %d(%d) edges.", Nvertices, counter, data.numEdges());
  togglePlannerDataVisibility();
}

void PathReplayWorldNode::AddMultiRobotPlannerData(const std::unordered_map<std::string, dart::dynamics::SkeletonPtr>& skeletons, const ompl::base::PlannerData& data) {
  const auto& factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(data.getSpaceInformation());
  const auto Nvertices = data.numVertices();
  unsigned int counter = 0;
  auto sout = factor->allocState();

  auto childStates = factor->allocChildStates();

  auto children = factor->getChildren();

  for(unsigned int vindex = 0; vindex < Nvertices; vindex++) {
    for(unsigned int windex = 0; windex < Nvertices; windex++) {
      if(!data.edgeExists(vindex, windex)) {
        continue;
      }
      counter++;
      const auto s1 = data.getVertex(vindex).getState();
      const auto s2 = data.getVertex(windex).getState();

      const auto step_size = 0.01f;
      const auto L = factor->distance(s1, s2);

      std::unordered_map<std::string, std::vector<Eigen::Vector3d>> vertices;
      for(const auto& childState : childStates) {
        const auto& name = childState.first;
        vertices[name] = std::vector<Eigen::Vector3d>{};
      }

      for(double d = 0.0; d < L + step_size; d+= step_size) {
        factor->getStateSpace()->interpolate(s1, s2, d/L, sout);

        //convert to child nodes
        factor->project(sout, childStates);
        for(const auto& childState : childStates) {
          const auto& name = childState.first;

          const auto config = StateToEigenVectorXd(factor->getChild(name), childState.second);
          const auto v = GetFK(skeletons.at(name), config);

          vertices.at(name).push_back(v);
        }
      }
      for(const auto& childState : childStates) {
        auto frame_line = getWorld()->addSimpleFrame(createLineSegmentFrame(vertices.at(childState.first), kRoadmapColorVertex, kRoadmapLineWidth));
        planner_data_frames_.push_back(frame_line);
      }
    }
  }
  factor->freeState(sout);
  factor->freeChildStates(childStates);
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
  for(const auto& pair : skeleton_and_path_) {
    const auto& manipulator = pair.first;
    const auto& path = pair.second;
    auto config = path->GetConfigAt(path_position_);
    manipulator->setConfiguration(config);
  }
  if(collision_checker_ != nullptr) {
    if(collision_checker_->IsInCollision(getWorld())) {
      collision_checker_->ColorAllCollisionBodies(getWorld());
    } else {
      collision_checker_->ResetColors(getWorld());
    }
  }
}

void PathReplayWorldNode::customPostStep()
{
  if(skeleton_and_path_.empty()) {
    return;
  }
  if(pause_) {
    return;
  }
  path_position_ += (reverse_ ? -step_size_ : +step_size_);
  if(path_position_ > 1.0f) {
    reverse_ = true;
    path_position_ = 1.0f;
  }
  if(path_position_ < 0.0f) {
    reverse_ = false;
    path_position_ = 0.0f;
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
  for(const auto& pair : skeleton_and_path_) {
    const auto& manipulator = pair.first;
    const auto& path = pair.second;
    auto config = path->GetConfigAt(path_position_);

    for(size_t k =0; k < config.size();k++) {
      s+= delim + std::to_string(config[k]);
      delim = ", \n";
    }
  }
  return s;
}

float PathReplayWorldNode::getCurrentPosition() const {
  return path_position_;
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

void PathReplayWorldNode::CreateKeyPressEvents() {
  events_.push_back({'s', "play/pause planned path", [&](){toggleStartStop();}});
  events_.push_back({'r', "reverse execution direction", [&](){toggleReverse();}});
  events_.push_back({'n', "decrease execution speed", [&](){decreaseSpeed();}});
  events_.push_back({'m', "increase execution speed", [&](){increaseSpeed();}});
  events_.push_back({'1', "show/hide solution path", [&](){toggleSolutionPathVisibility();}});
  events_.push_back({'2', "show/hide planner data", [&](){togglePlannerDataVisibility();}});
  events_.push_back({'8', "store planner path", 
      [&](){
        for(const auto& [skeleton, path] : skeleton_and_path_) {
          auto name = "eigen_path_"+skeleton->getName();
          path->Save(name);
          OMPL_INFORM("Save path %s", name.c_str()); 
        }
      }});
  events_.push_back({'9', "load planner path", 
      [&](){
        for(const auto& [skeleton, path] : skeleton_and_path_) {
          auto name = "fail_eigen_path_"+skeleton->getName();
          path->Load(name);
          OMPL_INFORM("Load path %s", name.c_str()); 
        }
      }});
  events_.push_back({'p', "output current skeleton info", 
      [&](){
        for(const auto& [skeleton, _] : skeleton_and_path_) {
          PrintSkeletonInfo(skeleton);
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
    auto events = world_node_->GetKeyPressEvents();
    for(const auto& event : events) {
      if(ea.getKey() != event.key) {
        continue;
      }
      event.function();
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

  ImGui::Text("%s", viewer_->getInstructions().c_str());
  ImGui::Text("Path position: %.2f (%s)\nPath speed: %f)", 
      world_node_->getCurrentPosition(), 
      (world_node_->isRunning() ? "running" : "pause"),
      world_node_->getStepSize());
  ImGui::Text("Config : %s", world_node_->getCurrentJointConfiguration().c_str());
  ImGui::End();
}
