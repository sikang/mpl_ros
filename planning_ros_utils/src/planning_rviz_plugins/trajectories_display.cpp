#include <tf/transform_listener.h>
#include "trajectories_display.h"

namespace planning_rviz_plugins {
  TrajectoriesDisplay::TrajectoriesDisplay() {
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
    jrk_color_property_ =
      new rviz::ColorProperty("JrkColor", QColor(200, 20, 55), "Color to draw the Acc.",
          this, SLOT(updateJrkColorAndAlpha()));
    pos_scale_property_ =
      new rviz::FloatProperty("PosScale", 0.02, "0.02 is the default value.",
          this, SLOT(updatePosScale()));
    vel_scale_property_ =
      new rviz::FloatProperty("VelScale", 0.02, "0.02 is the default value.",
          this, SLOT(updateVelScale()));
    acc_scale_property_ =
      new rviz::FloatProperty("AccScale", 0.02, "0.02 is the default value.",
          this, SLOT(updateAccScale()));
    jrk_scale_property_ =
      new rviz::FloatProperty("JrkScale", 0.02, "0.02 is the default value.",
          this, SLOT(updateJrkScale()));
    vel_vis_property_ =
      new rviz::BoolProperty("Visualize Vel", 0, "Visualize Vel?",
          this, SLOT(updateVelVis()));
    acc_vis_property_ =
      new rviz::BoolProperty("Visualize Acc", 0, "Visualize Acc?",
          this, SLOT(updateAccVis()));
    jrk_vis_property_ =
      new rviz::BoolProperty("Visualize Jrk", 0, "Visualize Jrk?",
          this, SLOT(updateJrkVis()));
    history_length_property_ =
      new rviz::IntProperty("History Length", 1, "Number of msg to display.",
          this, SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(10000);
 }

  void TrajectoriesDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
  }

  TrajectoriesDisplay::~TrajectoriesDisplay() {}

  void TrajectoriesDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
  }

  void TrajectoriesDisplay::updateVelVis() {
    bool vis = vel_vis_property_->getBool();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setVelVis(vis);
    visualizeMessage();
  }

  void TrajectoriesDisplay::updateAccVis() {
    bool vis = acc_vis_property_->getBool();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setAccVis(vis);
    visualizeMessage();
  }

  void TrajectoriesDisplay::updateJrkVis() {
    bool vis = jrk_vis_property_->getBool();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setJrkVis(vis);
    visualizeMessage();
  }

  void TrajectoriesDisplay::updatePosColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = pos_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setPosColor(color.r, color.g, color.b, alpha);
  }

  void TrajectoriesDisplay::updateVelColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = vel_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setVelColor(color.r, color.g, color.b, alpha);
  }

  void TrajectoriesDisplay::updateAccColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = acc_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setAccColor(color.r, color.g, color.b, alpha);
  }

  void TrajectoriesDisplay::updateJrkColorAndAlpha() {
    float alpha = 1;
    Ogre::ColourValue color = jrk_color_property_->getOgreColor();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setJrkColor(color.r, color.g, color.b, alpha);
  }

  void TrajectoriesDisplay::updatePosScale() {
    float s = pos_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setPosScale(s);
  }

  void TrajectoriesDisplay::updateVelScale() {
    float s = vel_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setVelScale(s);
  }

  void TrajectoriesDisplay::updateAccScale() {
    float s = acc_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setAccScale(s);
  }

  void TrajectoriesDisplay::updateJrkScale() {
    float s = jrk_scale_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
      visuals_[i]->setJrkScale(s);
  }

  void TrajectoriesDisplay::updateHistoryLength() {
    visuals_.rset_capacity(history_length_property_->getInt());
  }

  void TrajectoriesDisplay::processMessage(const planning_ros_msgs::Trajectories::ConstPtr &msg) {
    if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
          msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    trajectories_ = *msg;

    visualizeMessage();
  }

  void TrajectoriesDisplay::updateNum() {
    visualizeMessage();
  }

  void TrajectoriesDisplay::visualizeMessage() {
    if (trajectories_.trajectories.empty())
      return;

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

    bool jrk_vis = jrk_vis_property_->getBool();
    visual->setJrkVis(jrk_vis);

    visual->setMessage(trajectories_.trajectories.front());
    for(int i = 1; i < (int)trajectories_.trajectories.size(); i++)
      visual->addMessage(trajectories_.trajectories[i]);

    visual->setFramePosition(position_);
    visual->setFrameOrientation(orientation_);

    float pos_scale = pos_scale_property_->getFloat();
    visual->setPosScale(pos_scale);
    float vel_scale = vel_scale_property_->getFloat();
    visual->setVelScale(vel_scale);
    float acc_scale = acc_scale_property_->getFloat();
    visual->setAccScale(acc_scale);
    float jrk_scale = jrk_scale_property_->getFloat();
    visual->setJrkScale(jrk_scale);

    Ogre::ColourValue pos_color = pos_color_property_->getOgreColor();
    visual->setPosColor(pos_color.r, pos_color.g, pos_color.b, 1);
    Ogre::ColourValue vel_color = vel_color_property_->getOgreColor();
    visual->setVelColor(vel_color.r, vel_color.g, vel_color.b, 1);
    Ogre::ColourValue acc_color = acc_color_property_->getOgreColor();
    visual->setAccColor(acc_color.r, acc_color.g, acc_color.b, 1);
    Ogre::ColourValue jrk_color = jrk_color_property_->getOgreColor();
    visual->setJrkColor(jrk_color.r, jrk_color.g, jrk_color.b, 1);

    visuals_.push_back(visual);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::TrajectoriesDisplay, rviz::Display)
