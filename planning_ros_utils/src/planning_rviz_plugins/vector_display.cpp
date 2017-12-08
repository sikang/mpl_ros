#include <tf/transform_listener.h>
#include "vector_display.h"

namespace planning_rviz_plugins {
  VectorDisplay::VectorDisplay() {
    line_color_property_ =
      new rviz::ColorProperty("LineColor", QColor(204, 51, 204), "Color to draw the line.",
                              this, SLOT(updateLineColorAndAlpha()));
    vs_color_property_ =
      new rviz::ColorProperty("VsColor", QColor(85, 85, 255), "Color to draw the node.",
                              this, SLOT(updateNodeColorAndAlpha()));
    line_scale_property_ =
      new rviz::FloatProperty("LineScale", 0.1, "0.1 is the default value.",
                              this, SLOT(updateLineScale()));
    vs_scale_property_ =
      new rviz::FloatProperty("VsScale", 0.3, "0.15 is the default value.",
                              this, SLOT(updateNodeScale()));

    state_property_ = new rviz::EnumProperty(
        "State", "Both", "Arrows can be represented as three states: vectors, path and both",
        this, SLOT(updateState()));
    state_property_->addOption("Vs", 0);
    state_property_->addOption("Path", 1);
    state_property_->addOption("Both", 2);

    visuals_vs_.rset_capacity(1);
    visuals_path_.rset_capacity(1);
  }

  void VectorDisplay::onInitialize() {
    MFDClass::onInitialize();
  }

  VectorDisplay::~VectorDisplay() {}

  void VectorDisplay::reset() {
    MFDClass::reset();
    visuals_path_.clear();
    visuals_vs_.clear();
  }

  void VectorDisplay::updateLineColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = line_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_path_.size(); i++) {
      visuals_path_[i]->setLineColor(color.r, color.g, color.b, alpha);
      visuals_path_[i]->setNodeColor(color.r, color.g, color.b, alpha);
    }
  }

  void VectorDisplay::updateVsColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = vs_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_vs_.size(); i++)
     visuals_vs_[i]->setColor(color.r, color.g, color.b, alpha);
  }

  void VectorDisplay::updateLineScale() {
    float s = line_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_path_.size(); i++) {
      visuals_path_[i]->setLineScale(s);
      visuals_path_[i]->setNodeScale(s/2);
    }
  }

  void VectorDisplay::updateVsScale() {
    float s = vs_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_vs_.size(); i++)
      visuals_vs_[i]->setScale(s);
  }

  void VectorDisplay::updateState() {
    int state = state_property_->getOptionInt();
    visualizeMessage(state);
  }

  void VectorDisplay::processMessage(const planning_ros_msgs::Arrows::ConstPtr &msg) {
    if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    path_.clear();
    vs_.clear();
    for(int i = 0; i < (int) msg->points.size(); i++) {
      Vec3f pt(msg->points[i].x,
          msg->points[i].y,
          msg->points[i].z);
      Vec3f dir(msg->directions[i].x,
          msg->directions[i].y,
          msg->directions[i].z);
      path_.push_back(pt);
      vs_.push_back(std::make_pair(pt, dir));
    }

    int state = state_property_->getOptionInt();
    visualizeMessage(state);
  }

  void VectorDisplay::visualizeMessage(int state) {
    switch (state) {
      case 0:
        visuals_vs_.clear();
        visualizeVs();
        break;
      case 1:
        visuals_path_.clear();
        visualizePath();
        break;
      case 2:
        visuals_vs_.clear();
        visuals_path_.clear();
        visualizeVs();
        visualizePath();
        break;
     default:
        std::cout << "Invalid State: " << state << std::endl;
    }
  }

  void VectorDisplay::visualizePath() {
    boost::shared_ptr<PathVisual> visual;
    if (visuals_path_.full())
      visual = visuals_path_.front();
    else
      visual.reset(new PathVisual(context_->getSceneManager(), scene_node_));

    visual->setMessage(path_);
    visual->setFramePosition(position_);
    visual->setFrameOrientation(orientation_);

    float line_scale = line_scale_property_->getFloat();
    visual->setLineScale(line_scale);
    visual->setNodeScale(line_scale / 2);

    Ogre::ColourValue line_color = line_color_property_->getOgreColor();
    visual->setLineColor(line_color.r, line_color.g, line_color.b, 1);
    visual->setNodeColor(line_color.r, line_color.g, line_color.b, 1);

    visuals_path_.push_back(visual);
  }


  void VectorDisplay::visualizeVs() {
    boost::shared_ptr<VectorVisual> visual;
    if (visuals_vs_.full())
      visual = visuals_vs_.front();
    else
      visual.reset(new VectorVisual(context_->getSceneManager(), scene_node_));

    visual->setMessage(vs_);
    visual->setFramePosition(position_);
    visual->setFrameOrientation(orientation_);

    Ogre::ColourValue color = vs_color_property_->getOgreColor();
    visual->setColor(color.r, color.g, color.b, 1.0);
    float s = vs_scale_property_->getFloat();
    visual->setScale(s);

    visuals_vs_.push_back(visual);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::VectorDisplay, rviz::Display)
