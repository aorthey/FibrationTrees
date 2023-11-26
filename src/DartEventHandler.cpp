#include "DartEventHandler.hpp"

#include <ompl/geometric/PathGeometric.h>

#include "OmplHelper.hpp"
#include "DartHelper.hpp"

PathReplayWorldNode::PathReplayWorldNode(
    dart::simulation::WorldPtr world,
    dart::dynamics::SkeletonPtr manipulator,
    const ompl::base::PathPtr& path,
    const CollisionCheckerPtr& collision_checker)
  : dart::gui::osg::RealTimeWorldNode(std::move(world)),
    manipulator_(std::move(manipulator)), 
    // path_(path),
    collision_checker_(collision_checker)
{
  // const auto& si = path->getSpaceInformation();

  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());
  pgeo.interpolate(100);
  auto states = pgeo.getStates();
  // std::vector<Eigen::VectorXd> configs;
  // std::cout << "----------------------------------------" << std::endl;
  // for(size_t k = 0; k < states.size(); k++) {
  //   Eigen::VectorXd config = StateToEigenVectorXd(si, states.at(k));
  //   configs.push_back(config);
  //   std::cout << config << std::endl;
  // }
  // eigen_path_ = std::make_shared<EigenPath>(configs);
  path_position_ = 0.0f;

  ompl_path_ = std::make_shared<OmplPath>(path);
  // lengths_.clear();
  // for(size_t k = 1; k < states.size(); k++) {
  //   auto s1 = states.at(k-1);
  //   auto s2 = states.at(k);
  //   auto d = si->distance(s1, s2);
  //   lengths_.push_back(d);
  //   std::cout << d << ", ";
  // }
  // std::cout << std::endl;
  // current_index_ = 0;
  // current_position_ = 0.0f;
  // step_size_ = kStepSize;

  // tmpState_ = si->allocState();
  pause_ = true;
  reverse_ = false;

}

PathReplayWorldNode::~PathReplayWorldNode() {
  // const auto& si = path_->getSpaceInformation();
  // si->freeState(tmpState_);
}

void PathReplayWorldNode::customPreRefresh()
{
}

void PathReplayWorldNode::customPostRefresh()
{
}

void PathReplayWorldNode::customPreStep()
{
  auto config = ompl_path_->GetConfigAt(path_position_);
  manipulator_->setConfiguration(config);
  // std::cout << config << std::endl;
  // ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path_.get());
  // auto states = pgeo.getStates();

  // const auto& si = path_->getSpaceInformation();


  // if(current_index_ < 0) {
  //   auto s1 = states.front();
  //   Eigen::VectorXd config = StateToEigenVectorXd(si, s1);
  //   manipulator_->setConfiguration(config);
  //   return;
  // }
  // if(current_index_ > lengths_.size() - 1) {
  //   auto s1 = states.back();
  //   Eigen::VectorXd config = StateToEigenVectorXd(si, s1);
  //   manipulator_->setConfiguration(config);
  //   return;
  // }

  // const auto current_length = lengths_.at(current_index_);

  // auto s1 = states.at(current_index_);
  // auto s2 = states.at(current_index_+1);

  // if(current_length > std::numeric_limits<double>::epsilon()) {
  //   si->getStateSpace()->interpolate(s1, s2, current_position_ / current_length, tmpState_);
  //   Eigen::VectorXd config = StateToEigenVectorXd(si, tmpState_);
  //   manipulator_->setConfiguration(config);
  // } else {
  //   Eigen::VectorXd config = StateToEigenVectorXd(si, s1);
  //   manipulator_->setConfiguration(config);
  // }

  // auto endeffector = manipulator_->getBodyNode(manipulator_->getNumBodyNodes() - 1)->getName();
  // auto frame =manipulator_->getBodyNode(endeffector)->getTransform().translation();
  // getWorld()->addSimpleFrame(createSphereFrame(frame));

  // if(collision_checker_->IsInCollision(getWorld())) {
  //   std::cout << "Config in collision: " << manipulator_->getConfiguration().mPositions << std::endl;
  // }
}

void PathReplayWorldNode::customPostStep()
{
  if(pause_) {
    return;
  }
  path_position_ += (reverse_ ? -kStepSize : +kStepSize);
  if(path_position_ > 1.0f) {
    reverse_ = true;
    path_position_ = 1.0f;
  }
  if(path_position_ < 0.0f) {
    reverse_ = false;
    path_position_ = 0.0f;
  }
  //if(current_index_ < 0) {
  //  current_index_ = 0;
  //  current_position_ = 0.0f;
  //  reverse_ = false;
  //}
  //if(current_index_ > lengths_.size() - 1) {
  //  reverse_ = true;
  //  current_index_ = lengths_.size() - 1;
  //  current_position_ = lengths_.at(current_index_);
  //}

  //const auto current_length = lengths_.at(current_index_);
  ////std::cout << "Current pos : " << current_index_ << "/" << lengths_.size() - 1 << ", " << current_position_ << "/" << current_length << std::endl;
  //current_position_ += (reverse_ ? -step_size_ : +step_size_);

  //if(current_position_ > current_length) {
  //  current_position_ = 0.0f;
  //  current_index_++;
  //}
  //if(current_position_ < 0.0f) {
  //  current_index_--;
  //  if(current_index_ >= 0) {
  //    current_position_ = lengths_.at(current_index_);
  //  }
  //}
}

void PathReplayWorldNode::toggleStartStop() {
  pause_ = !pause_;
}
void PathReplayWorldNode::toggleReverse() {
  reverse_ = !reverse_;
}

std::string PathReplayWorldNode::getCurrentJointConfiguration() const {
  auto config = ompl_path_->GetConfigAt(path_position_);
  std::string s;
  std::string delim = "";

  for(size_t k =0; k < config.size();k++) {
    s+= delim + std::to_string(config[k]);
    delim = ", \n";
  }
  return s;
}

float PathReplayWorldNode::getCurrentPosition() const {
  return path_position_;
  // float total = std::accumulate(lengths_.begin(), lengths_.end(), 0.0f);
  // if(current_index_ < 0) {
  //   return 0.0f;
  // }
  // if(current_index_ > lengths_.size() - 1) {
  //   return 1.0;
  // }
  // float current = std::accumulate(lengths_.begin(), lengths_.begin()+current_index_, 0.0f) + current_position_;
  // if(total < 1e-6) {
  //   return 0.0f;
  // }
  // return current / total;
}

PathReplayEventHandler::PathReplayEventHandler(PathReplayWorldNode* worldNode)
{
  mWorldNode = worldNode;
}

bool PathReplayEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) 
{
  if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
  {
    if (ea.getKey() == 's')
    {
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

TextWidget::TextWidget(
      dart::gui::osg::ImGuiViewer* viewer, PathReplayWorldNode* worldNode)
    : viewer_(viewer),
      mWorldNode(worldNode)
{
}

void TextWidget::render()
{
  ImGui::SetNextWindowPos(ImVec2(10, 20));
  ImGui::SetNextWindowSize(ImVec2(240, 320));
  ImGui::SetNextWindowBgAlpha(0.5f);
  if (!ImGui::Begin(
          "Tinkertoy Control",
          nullptr,
          ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
              | ImGuiWindowFlags_HorizontalScrollbar))
  {
    ImGui::End();
    return;
  }
  ImGui::Text("%s", viewer_->getInstructions().c_str());
  ImGui::Text("%.2f", mWorldNode->getCurrentPosition());
  ImGui::Text("Config : %s", mWorldNode->getCurrentJointConfiguration().c_str());
  ImGui::End();
}
