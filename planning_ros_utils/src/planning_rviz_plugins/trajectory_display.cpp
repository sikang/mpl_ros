#include <tf/transform_listener.h>
#include "trajectory_display.h"

namespace mav_rviz_plugins {
  TrajectoryDisplay::TrajectoryDisplay() {
    num_property_ =
      new rviz::IntProperty("Num of samples", 20, "Number of samples to display.",
          this, SLOT(updateNum()));
    pos_color_property_ =
      new rviz::ColorProperty("PosColor", QColor(204, 51, 204), "Color to draw the Pos.",
          this, SLOT(updatePosColorAndAlpha()));
    vel_color_property_ =
      new rviz::ColorProperty("VelColor", QColor(85, 85, 255), "Color to draw the Vel.",
          this, SLOT(updateVelColorAndAlpha()));
    acc_color_property_ =
      new rviz::ColorProperty("AccColor", QColor(10, 200, 55), "Color to draw the Acc.",
          this, SLOT(updateAccColorAndAlpha()));
    pos_scale_property_ =
      new rviz::FloatProperty("PosScale", 0.02, "0.02 is the default value.",
          this, SLOT(updatePosScale()));
    vel_scale_property_ =
      new rviz::FloatProperty("VelScale", 0.02, "0.02 is the default value.",
          this, SLOT(updateVelScale()));
    acc_scale_property_ =
      new rviz::FloatProperty("AccScale", 0.02, "0.02 is the default value.",
          this, SLOT(updateAccScale()));
    vel_vis_property_ =
      new rviz::BoolProperty("Visualize Vel", 0, "Visualize Vel?",
          this, SLOT(updateVelVis()));
    acc_vis_property_ =
      new rviz::BoolProperty("Visualize Acc", 0, "Visualize Acc?",
          this, SLOT(updateAccVis()));
    history_length_property_ =
      new rviz::IntProperty("History Length", 1, "Number of msg to display.",
          this, SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(10000);
  }

  void TrajectoryDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
  }

  TrajectoryDisplay::~TrajectoryDisplay() {}

  void TrajectoryDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
  }

  void TrajectoryDisplay::updateVelVis() {
    bool vis = vel_vis_property_->getBool();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setVelVis(vis);
    visualizeMessage();
  }

  void TrajectoryDisplay::updateAccVis() {
    bool vis = acc_vis_property_->getBool();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setAccVis(vis);
    visualizeMessage();
  }

  void TrajectoryDisplay::updatePosColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = pos_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setPosColor(color.r, color.g, color.b, alpha);
  }

  void TrajectoryDisplay::updateVelColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = vel_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setVelColor(color.r, color.g, color.b, alpha);
  }

  void TrajectoryDisplay::updateAccColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = acc_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setAccColor(color.r, color.g, color.b, alpha);
  }

  void TrajectoryDisplay::updatePosScale() {
    float s = pos_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setPosScale(s);
  }

  void TrajectoryDisplay::updateVelScale() {
    float s = vel_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setVelScale(s);
  }

  void TrajectoryDisplay::updateAccScale() {
    float s = acc_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setAccScale(s);
  }

  void TrajectoryDisplay::updateNum() {
    float n = num_property_->getInt();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setNum(n);
  }

  void TrajectoryDisplay::updateHistoryLength() {
    visuals_.rset_capacity(history_length_property_->getInt());
  }

  void TrajectoryDisplay::processMessage(
                                   const planning_ros_msgs::Trajectory::ConstPtr &msg) {
    if (!context_->getFrameManager()->getTransform(
                                                   msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    trajectory_ = *msg;

    visualizeMessage();
  }

  void TrajectoryDisplay::visualizeMessage() {
    boost::shared_ptr<TrajectoryVisual> visual;
    if (visuals_.full())
      visual = visuals_.front();
    else
      visual.reset(new TrajectoryVisual(context_->getSceneManager(), scene_node_));

    float n = num_property_->getInt();
    visual->setNum(n);

    bool vel_vis = vel_vis_property_->getBool();
    visual->setVelVis(vel_vis);

    bool acc_vis = acc_vis_property_->getBool();
    visual->setAccVis(acc_vis);

    visual->setMessage(trajectory_);
    visual->setFramePosition(position_);
    visual->setFrameOrientation(orientation_);

    float pos_scale = pos_scale_property_->getFloat();
    visual->setPosScale(pos_scale);
    float vel_scale = vel_scale_property_->getFloat();
    visual->setVelScale(vel_scale);
    float acc_scale = acc_scale_property_->getFloat();
    visual->setAccScale(acc_scale);

    Ogre::ColourValue pos_color = pos_color_property_->getOgreColor();
    visual->setPosColor(pos_color.r, pos_color.g, pos_color.b, 1);
    Ogre::ColourValue vel_color = vel_color_property_->getOgreColor();
    visual->setVelColor(vel_color.r, vel_color.g, vel_color.b, 1);
    Ogre::ColourValue acc_color = acc_color_property_->getOgreColor();
    visual->setAccColor(acc_color.r, acc_color.g, acc_color.b, 1);

    visuals_.push_back(visual);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_rviz_plugins::TrajectoryDisplay, rviz::Display)
