#include "polyhedra_display.h"

namespace mav_rviz_plugins {

PolyhedraDisplay::PolyhedraDisplay() {
  mesh_color_property_ =
      new rviz::ColorProperty("MeshColor", QColor(0, 170, 255), "Mesh color.",
                              this, SLOT(updateMeshColorAndAlpha()));
  bound_color_property_ =
      new rviz::ColorProperty("BoundColor", QColor(255, 0, 0), "Bound color.",
                              this, SLOT(updateBoundColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 0.2,
      "0 is fully transparent, 1.0 is fully opaque, only affect mesh", this,
      SLOT(updateMeshColorAndAlpha()));

  scale_property_ = new rviz::FloatProperty("Scale", 0.1, "bound scale.", this,
                                            SLOT(updateScale()));

  vs_scale_property_ = new rviz::FloatProperty("VsScale", 1.0, "Vs scale.", this,
                                               SLOT(updateVsScale()));

  vs_color_property_ =
    new rviz::ColorProperty("VsColor", QColor(0, 255, 0), "Vs color.",
                            this, SLOT(updateVsColorAndAlpha()));



  state_property_ = new rviz::EnumProperty(
      "State", "Mesh", "A Polygon can be represented as two states: Mesh and "
                       "Bound, this option allows selecting visualizing Polygon"
                       "in corresponding state",
      this, SLOT(updateState()));
  state_property_->addOption("Mesh", 0);
  state_property_->addOption("Bound", 1);
  state_property_->addOption("Both", 2);
  state_property_->addOption("Vs", 3);

  visuals_mesh_.rset_capacity(1);
  visuals_bound_.rset_capacity(1);
  visuals_vector_.rset_capacity(1);
}

void PolyhedraDisplay::onInitialize() { MFDClass::onInitialize(); }

PolyhedraDisplay::~PolyhedraDisplay() {}

void PolyhedraDisplay::reset() {
  MFDClass::reset();
  visuals_mesh_.clear();
  visuals_bound_.clear();
  visuals_vector_.clear();
}

void PolyhedraDisplay::updateMeshColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_mesh_.size(); i++)
    visuals_mesh_[i]->setColor(color.r, color.g, color.b, alpha);
}

void PolyhedraDisplay::updateBoundColorAndAlpha() {
  float alpha = 1.0;
  Ogre::ColourValue color = bound_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_bound_.size(); i++)
    visuals_bound_[i]->setColor(color.r, color.g, color.b, alpha);
}

void PolyhedraDisplay::processMessage(
    const planning_ros_msgs::Polyhedra::ConstPtr &msg) {
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  vertices_.clear();
  passes_.clear();
  vs_.clear();

  for(const auto& polyhedron: msg->polyhedra){
    Polyhedron p;
    for(unsigned int i = 0; i < polyhedron.points.size(); i++){
      Vec3f pt(polyhedron.points[i].x,
               polyhedron.points[i].y,
               polyhedron.points[i].z);
      Vec3f n(polyhedron.normals[i].x,
              polyhedron.normals[i].y,
              polyhedron.normals[i].z);
      vs_.push_back(std::make_pair(pt, n));
      if(polyhedron.passes.empty())
        p.push_back(Face(pt, n));
      else
        p.push_back(Face(pt, n, polyhedron.passes[i]));
    }
    std::pair<BoundVec3f, std::vector<bool>> bds = cal_extreme_points(p);
    vertices_.insert(vertices_.end(), bds.first.begin(), bds.first.end());
    passes_.insert(passes_.end(), bds.second.begin(), bds.second.end());
  }

  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void PolyhedraDisplay::visualizeMesh() {
  boost::shared_ptr<MeshVisual> visual_mesh;
  if (visuals_mesh_.full())
    visual_mesh = visuals_mesh_.front();
  else
    visual_mesh.reset(new MeshVisual(context_->getSceneManager(), scene_node_));

 
  visual_mesh->setMessage(vertices_, passes_);
  visual_mesh->setFramePosition(position_);
  visual_mesh->setFrameOrientation(orientation_);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();
  visual_mesh->setColor(color.r, color.g, color.b, alpha);
  visuals_mesh_.push_back(visual_mesh);
}

void PolyhedraDisplay::visualizeBound() {
  boost::shared_ptr<BoundVisual> visual_bound;
  if (visuals_bound_.full())
    visual_bound = visuals_bound_.front();
  else
    visual_bound.reset(
        new BoundVisual(context_->getSceneManager(), scene_node_));

  visual_bound->setMessage(vertices_);
  visual_bound->setFramePosition(position_);
  visual_bound->setFrameOrientation(orientation_);

  Ogre::ColourValue color = bound_color_property_->getOgreColor();
  visual_bound->setColor(color.r, color.g, color.b, 1.0);
  float scale = scale_property_->getFloat();
  visual_bound->setScale(scale);

  visuals_bound_.push_back(visual_bound);
}

void PolyhedraDisplay::visualizeVs() {
  boost::shared_ptr<VectorVisual> visual_vector;
  if (visuals_vector_.full())
    visual_vector = visuals_vector_.front();
  else
    visual_vector.reset(new VectorVisual(context_->getSceneManager(), scene_node_));

  visual_vector->setMessage(vs_);
  visual_vector->setFramePosition(position_);
  visual_vector->setFrameOrientation(orientation_);

  Ogre::ColourValue color = vs_color_property_->getOgreColor();
  visual_vector->setColor(color.r, color.g, color.b, 1.0);

  visuals_vector_.push_back(visual_vector);
}

void PolyhedraDisplay::visualizeMessage(int state) {
  switch (state) {
  case 0:
    visuals_bound_.clear();
    visualizeMesh();
    break;
  case 1:
    visuals_mesh_.clear();
    visualizeBound();
    break;
  case 2:
    visualizeMesh();
    visualizeBound();
    break;
  case 3:
    visuals_vector_.clear();
    visualizeMesh();
    visualizeBound();
    visualizeVs();
    break;
  default:
    std::cout << "Invalid State: " << state << std::endl;
  }
}

void PolyhedraDisplay::updateState() {
  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void PolyhedraDisplay::updateScale() {
  float s = scale_property_->getFloat();
  for (size_t i = 0; i < visuals_bound_.size(); i++)
    visuals_bound_[i]->setScale(s);
}

void PolyhedraDisplay::updateVsScale() {
  float s = vs_scale_property_->getFloat();
  for (size_t i = 0; i < visuals_vector_.size(); i++)
    visuals_vector_[i]->setScale(s);
}

void PolyhedraDisplay::updateVsColorAndAlpha() {
  float alpha = 1.0;
  Ogre::ColourValue color = vs_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_vector_.size(); i++)
    visuals_vector_[i]->setColor(color.r, color.g, color.b, alpha);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_rviz_plugins::PolyhedraDisplay, rviz::Display)
