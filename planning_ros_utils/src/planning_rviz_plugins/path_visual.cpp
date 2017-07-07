#include "path_visual.h"

namespace planning_rviz_plugins {

PathVisual::PathVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

PathVisual::~PathVisual() { scene_manager_->destroySceneNode(frame_node_); }

void PathVisual::setMessage(const vec_Vec3f &path) {
  nodes_.clear();
  line_.reset(new rviz::BillboardLine(scene_manager_, frame_node_));

  if (path.empty())
    return;

  nodes_.resize(path.size());

  for (auto &it : nodes_)
    it.reset(new rviz::Shape(rviz::Shape::Type::Sphere, scene_manager_,
                             frame_node_));

  int cnt = 0;
  for (const auto &node : path) {
    Ogre::Vector3 pos(node(0), node(1), node(2));
    line_->addPoint(pos);
    nodes_[cnt]->setPosition(pos);
    cnt++;
  }
}

void PathVisual::setNumLines(int n)
{
  line_->setNumLines(n);
}

void PathVisual::addMessage(const vec_Vec3f &path) {
  if (path.empty())
    return;

  unsigned int prev_size = nodes_.size();
  nodes_.resize(prev_size + path.size());

  for (unsigned int i =  prev_size; i < nodes_.size(); i++)
    nodes_[i].reset(new rviz::Shape(rviz::Shape::Type::Sphere, scene_manager_,
                             frame_node_));

	line_->newLine();
  int cnt = prev_size;
  for (const auto &node : path) {
    Ogre::Vector3 pos(node(0), node(1), node(2));
    line_->addPoint(pos);
    nodes_[cnt]->setPosition(pos);
    cnt++;
  }
}

void PathVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void PathVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void PathVisual::setLineColor(float r, float g, float b, float a) {
  line_->setColor(r, g, b, a);
}

void PathVisual::setNodeColor(float r, float g, float b, float a) {
  for (auto &it : nodes_)
    it->setColor(r, g, b, a);
}

void PathVisual::setLineScale(float s) { line_->setLineWidth(s); }

void PathVisual::setNodeScale(float s) {
  for (auto &it : nodes_)
    it->setScale(Ogre::Vector3(s, s, s));
}
}
