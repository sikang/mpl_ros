#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>
#include "map_display.h"

namespace planning_rviz_plugins {

MapDisplay::MapDisplay()
    : point_cloud_common_(new rviz::PointCloudCommon(this)) {
  queue_size_property_ = new rviz::IntProperty(
      "Queue Size", 10,
      "Advanced: set the size of the incoming PointCloud message queue. "
      " Increasing this is useful if your incoming TF data is delayed "
      "significantly "
      "from your PointCloud data, but it can greatly increase memory usage if "
      "the messages are big.",
      this, SLOT(updateQueueSize()));

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.

  state_property_ = new rviz::EnumProperty(
      "State", "Occupied", "VoxelMap has three states: Occupied, Free and "
                           "Unknown, this option allows selecting visualizing "
                           "voxels in corresponding state",
      this, SLOT(updateState()));
  state_property_->addOption("Occupied", 0);
  state_property_->addOption("Free", 1);
  state_property_->addOption("Unknown", 2);
  state_property_->addOption("Bound", 3);

  scale_property_ = new rviz::FloatProperty(
      "Scale", 0.2, "0.2 is default value.", this, SLOT(updateScale()));

  update_nh_.setCallbackQueue(point_cloud_common_->getCallbackQueue());
  visuals_.rset_capacity(1);
  map_util_.reset(new MPL::VoxelMapUtil());
}

MapDisplay::~MapDisplay() { delete point_cloud_common_; }

void MapDisplay::onInitialize() {
  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void MapDisplay::updateQueueSize() {
  tf_filter_->setQueueSize((uint32_t)queue_size_property_->getInt());
}

BoundVec3f MapDisplay::getSpace() {
  Vec3i dim = map_util_->getDim();
  Vec3f ori = map_util_->getOrigin();
  float res = map_util_->getRes();

  float x = dim(0) * res;
  float y = dim(1) * res;
  float z = dim(2) * res;
  float ox = ori(0);
  float oy = ori(1);
  float oz = ori(2);
  vec_Vec3f cs;
  cs.resize(8);
  cs[0](0) = ox + x;
  cs[0](1) = oy;
  cs[0](2) = oz;

  cs[1](0) = ox + x;
  cs[1](1) = oy + y;
  cs[1](2) = oz;

  cs[2](0) = ox;
  cs[2](1) = oy + y;
  cs[2](2) = oz;

  cs[3](0) = ox;
  cs[3](1) = oy;
  cs[3](2) = oz;

  cs[4](0) = ox;
  cs[4](1) = oy;
  cs[4](2) = oz + z;

  cs[5](0) = ox;
  cs[5](1) = oy + y;
  cs[5](2) = oz + z;

  cs[6](0) = ox + x;
  cs[6](1) = oy + y;
  cs[6](2) = oz + z;

  cs[7](0) = ox + x;
  cs[7](1) = oy;
  cs[7](2) = oz + z;

  /*
   *    7------6
   *   /|     /|
   *  / |    / |
   * 8------5  |
   * |  2---|--3
   * | /    | /
   * |/     |/
   * 1------4
   *
   *
   */


  BoundVec3f bs(6);
  vec_Vec3f f1(4), f2(4), f3(4), f4(4), f5(4), f6(4);

  f1[0] = cs[0];
  f1[1] = cs[1];
  f1[2] = cs[2];
  f1[3] = cs[3];

  f2[0] = cs[4];
  f2[1] = cs[5];
  f2[2] = cs[6];
  f2[3] = cs[7];

  f3[0] = cs[0];
  f3[1] = cs[3];
  f3[2] = cs[4];
  f3[3] = cs[7];

  f4[0] = cs[1];
  f4[1] = cs[2];
  f4[2] = cs[5];
  f4[3] = cs[6];

  f5[0] = cs[0];
  f5[1] = cs[1];
  f5[2] = cs[6];
  f5[3] = cs[7];

  f6[0] = cs[2];
  f6[1] = cs[3];
  f6[2] = cs[4];
  f6[3] = cs[5];

  bs[0] = f1;
  bs[1] = f2;
  bs[2] = f3;
  bs[3] = f4;
  bs[4] = f5;
  bs[5] = f6;

  return bs;
}


void MapDisplay::processMessage(
    const planning_ros_msgs::VoxelMapConstPtr &msg) {
  header_ = msg->header;
  setMap(map_util_.get(), *msg);

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          header_.frame_id, header_.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              header_.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  BoundVec3f bound  = getSpace();
  boost::shared_ptr<BoundVisual> visual;
  if (visuals_.full())
    visual = visuals_.front();
  else
    visual.reset(new BoundVisual(context_->getSceneManager(), scene_node_));

  // Now set or update the contents of the chosen visual.
  visual->setMessage(bound);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  visual->setColor(0.0, 0.0, 0, 1.0);
  float scale = scale_property_->getFloat();
  visual->setScale(scale);

  visuals_.push_back(visual);

  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void MapDisplay::visualizeMessage(int state) {
  switch (state) {
  case 0: {
    sensor_msgs::PointCloud cloud = vec_to_cloud(map_util_->getCloud());
    cloud.header = header_;
    point_cloud_common_->addMessage(
        boost::make_shared<sensor_msgs::PointCloud>(cloud));
    break;
  }
  case 1: {
    sensor_msgs::PointCloud cloud = vec_to_cloud(map_util_->getFreeCloud());
    cloud.header = header_;
    point_cloud_common_->addMessage(
        boost::make_shared<sensor_msgs::PointCloud>(cloud));
    break;
  }
  case 2: {
    sensor_msgs::PointCloud cloud = vec_to_cloud(map_util_->getUnknownCloud());
    cloud.header = header_;
    point_cloud_common_->addMessage(
        boost::make_shared<sensor_msgs::PointCloud>(cloud));
    break;
  }
  case 3: {
    point_cloud_common_->reset();
    break;
  }
  default:
    std::cout << "Invalid State: " << state << std::endl;
  }
}

void MapDisplay::updateState() {
  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void MapDisplay::update(float wall_dt, float ros_dt) {
  point_cloud_common_->update(wall_dt, ros_dt);
}

void MapDisplay::updateScale() {
  float scale = scale_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++)
    visuals_[i]->setScale(scale);
}

void MapDisplay::reset() {
  MFDClass::reset();
  point_cloud_common_->reset();
  visuals_.clear();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::MapDisplay, rviz::Display)
