#include "mesh_display.h"

namespace planning_rviz_plugins {

MeshDisplay::MeshDisplay() {
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
      "State", "Mesh", "A Mesh can be represented as two states: Mesh and "
                       "Bound, this option allows selecting visualizing mesh"
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

void MeshDisplay::onInitialize() { MFDClass::onInitialize(); }

MeshDisplay::~MeshDisplay() {}

void MeshDisplay::reset() {
  MFDClass::reset();
  visuals_mesh_.clear();
  visuals_bound_.clear();
  visuals_vector_.clear();
}

void MeshDisplay::updateMeshColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_mesh_.size(); i++)
    visuals_mesh_[i]->setColor(color.r, color.g, color.b, alpha);
}

void MeshDisplay::updateBoundColorAndAlpha() {
  float alpha = 1.0;
  Ogre::ColourValue color = bound_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_bound_.size(); i++)
    visuals_bound_[i]->setColor(color.r, color.g, color.b, alpha);
}


void MeshDisplay::updateVsScale() {
  float s = vs_scale_property_->getFloat();
  for (size_t i = 0; i < visuals_vector_.size(); i++)
    visuals_vector_[i]->setScale(s);
}

void MeshDisplay::updateVsColorAndAlpha() {
  float alpha = 1.0;
  Ogre::ColourValue color = vs_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_vector_.size(); i++)
    visuals_vector_[i]->setColor(color.r, color.g, color.b, alpha);
}

void MeshDisplay::processMessage(const planning_ros_msgs::Mesh::ConstPtr &msg) {
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  vertices_.clear();
  vs_.clear();

  for(int i = 0; i < (int)msg->faces.size(); i++) {
    vec_Vec3f vs;
    Vec3f c = Vec3f::Zero();
    for(const auto& it: msg->faces[i].vertices) {
      Vec3f pt(it.x, it.y, it.z);
      c += pt;
      vs.push_back(pt);
    }
    c /= vs.size();
    Vec3f n(0, 0, 1);
    if(i < (int)msg->norms.size())
      n = Vec3f(msg->norms[i].x,
          msg->norms[i].y,
          msg->norms[i].z);
    vs_.push_back(std::make_pair(c, n));
    vertices_.push_back(vs);
  }

  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void MeshDisplay::visualizeMesh() {
  boost::shared_ptr<MeshVisual> visual_mesh;
  if (visuals_mesh_.full())
    visual_mesh = visuals_mesh_.front();
  else
    visual_mesh.reset(new MeshVisual(context_->getSceneManager(), scene_node_));

 
  visual_mesh->setMessage(vertices_);
  visual_mesh->setFramePosition(position_);
  visual_mesh->setFrameOrientation(orientation_);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();
  visual_mesh->setColor(color.r, color.g, color.b, alpha);
  visuals_mesh_.push_back(visual_mesh);
}

void MeshDisplay::visualizeBound() {
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


void MeshDisplay::visualizeVs() {
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


void MeshDisplay::visualizeMessage(int state) {
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
      visuals_mesh_.clear();
      visuals_bound_.clear();
      visualizeMesh();
      visualizeBound();
      break;
    case 3:
      visuals_vector_.clear();
      visualizeMesh();
      visualizeVs();
      break;
    default:
      std::cout << "Invalid State: " << state << std::endl;
  }
}

void MeshDisplay::updateState() {
  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void MeshDisplay::updateScale() {
  float s = scale_property_->getFloat();
  for (size_t i = 0; i < visuals_bound_.size(); i++)
    visuals_bound_[i]->setScale(s);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::MeshDisplay, rviz::Display)
