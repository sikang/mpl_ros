#include <tf/transform_listener.h>
#include "ellipsoids_display.h"

namespace planning_rviz_plugins {

EllipsoidsDisplay::EllipsoidsDisplay() {
  color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
                                            "Color to draw the ellipsoid.",
                                            this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));

  history_length_property_ =
      new rviz::IntProperty("History Length", 1, "Number of msg to display.",
                            this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(10000);
}

void EllipsoidsDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateHistoryLength();
}

EllipsoidsDisplay::~EllipsoidsDisplay() {}

void EllipsoidsDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

void EllipsoidsDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
}

void EllipsoidsDisplay::updateHistoryLength() {
  visuals_.rset_capacity(history_length_property_->getInt());
}

void EllipsoidsDisplay::processMessage(
    const planning_ros_msgs::Ellipsoids::ConstPtr &msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  boost::shared_ptr<EllipsoidsVisual> visual;
  if (visuals_.full())
    visual = visuals_.front();
  else
    visual.reset(
        new EllipsoidsVisual(context_->getSceneManager(), scene_node_));

  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visuals_.push_back(visual);
}

} // end namespace planning_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::EllipsoidsDisplay, rviz::Display)
