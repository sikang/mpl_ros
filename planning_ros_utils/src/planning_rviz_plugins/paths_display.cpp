#include <tf/transform_listener.h>
#include "paths_display.h"

namespace planning_rviz_plugins {
  PathsDisplay::PathsDisplay() {
    line_color_property_ =
      new rviz::ColorProperty("LineColor", QColor(204, 51, 204), "Color to draw the line.",
                              this, SLOT(updateLineColorAndAlpha()));
    node_color_property_ =
      new rviz::ColorProperty("NodeColor", QColor(85, 85, 255), "Color to draw the node.",
                              this, SLOT(updateNodeColorAndAlpha()));
    line_scale_property_ =
      new rviz::FloatProperty("LineScale", 0.1, "0.1 is the default value.",
                              this, SLOT(updateLineScale()));
    node_scale_property_ =
      new rviz::FloatProperty("NodeScale", 0.15, "0.15 is the default value.",
                              this, SLOT(updateNodeScale()));
    history_length_property_ =
      new rviz::IntProperty("History Length", 1, "Number of msg to display.",
                            this, SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(10000);

    id_property_ =
      new rviz::EnumProperty("ID", "0", "Visualize individual path using its id, 0 is the default",
                             this, SLOT(updateID()));
  }

  void PathsDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
  }

  PathsDisplay::~PathsDisplay() {}

  void PathsDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
  }

  void PathsDisplay::updateLineColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = line_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setLineColor(color.r, color.g, color.b, alpha);
  }

  void PathsDisplay::updateNodeColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = node_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setNodeColor(color.r, color.g, color.b, alpha);
  }

  void PathsDisplay::updateLineScale() {
    float s = line_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setLineScale(s);
  }

  void PathsDisplay::updateNodeScale() {
    float s = node_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setNodeScale(s);
  }

  void PathsDisplay::updateHistoryLength() {
    visuals_.rset_capacity(history_length_property_->getInt());
  }

  void PathsDisplay::processMessage(
                                    const planning_ros_msgs::Paths::ConstPtr &msg) {
    if (!context_->getFrameManager()->getTransform(
                                                   msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    paths_ = *msg;

    id_property_->clearOptions();
    id_property_->addOption("All", -1);
    for (int i = 0; i < (int)paths_.paths.size(); i++)
      id_property_->addOption(QString::number(i), i);

    int id = id_property_->getOptionInt();
    visualizeMessage(id);
  }



  void PathsDisplay::visualizeMessage(int id) {
    if (id >= (int)paths_.paths.size())
      return;

    boost::shared_ptr<PathVisual> visual;
    if (visuals_.full())
      visual = visuals_.front();
    else
      visual.reset(new PathVisual(context_->getSceneManager(), scene_node_));

    bool set = false;
    if(id >= 0){
      set = true;
      visual->setMessage(mav_to_path(paths_.paths[id]));
      visual->setFramePosition(position_);
      visual->setFrameOrientation(orientation_);
    }
    else{
      for (int i = 0; i < (int)paths_.paths.size(); i++)
      {
        if(i == 0){
          set = true;
          visual->setMessage(mav_to_path(paths_.paths[i]));
          visual->setFramePosition(position_);
          visual->setFrameOrientation(orientation_);
        }
        visual->addMessage(mav_to_path(paths_.paths[i]));
      }
    }

    if(set){
      float line_scale = line_scale_property_->getFloat();
      visual->setLineScale(line_scale);
      float node_scale = node_scale_property_->getFloat();
      visual->setNodeScale(node_scale);

      Ogre::ColourValue line_color = line_color_property_->getOgreColor();
      visual->setLineColor(line_color.r, line_color.g, line_color.b, 1);
      Ogre::ColourValue node_color = node_color_property_->getOgreColor();
      visual->setNodeColor(node_color.r, node_color.g, node_color.b, 1);
    }
    visuals_.push_back(visual);
  }

  void PathsDisplay::updateID() {
    int id = id_property_->getOptionInt();
    visualizeMessage(id);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::PathsDisplay, rviz::Display)
