#include <ros/ros.h>
#include <ros_utils/data_ros_utils.h>
#include <mapping_utils/voxel_grid_rgb.h>
#include <mapping_utils/tf_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Publisher map_pub;
ros::Publisher cloud_pub;

std::string world_frame_;
std::unique_ptr<VoxelGridRGB> voxel_grid_;

void cloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  geometry_msgs::Pose cloud_to_world;
  static TFListener tf_listener;

  //ROS_INFO_THROTTLE(1, "Time diff: %f", (msg->header.stamp - ros::Time::now()).toSec());

  ros::Time msg_time = msg->header.stamp;
  if(!tf_listener.getPose(msg_time, msg->header.frame_id,
        world_frame_, cloud_to_world))
  {
    ROS_WARN_THROTTLE(1, "Time diff: %f", (msg_time - ros::Time::now()).toSec());
    return;
  }

  ros::Time t0 = ros::Time::now();

  Aff3f Tlw = toTF(cloud_to_world);

  // update map
  voxel_grid_->addCloud(Tlw, *msg);

  planning_ros_msgs::VoxelMap map = voxel_grid_->getMap();
  map.header.stamp = msg->header.stamp;
  map.header.frame_id = world_frame_;
  map_pub.publish(map);

  sensor_msgs::PointCloud cloud = voxel_grid_->getCloud();
  cloud.header.stamp = msg->header.stamp;
  cloud.header.frame_id = world_frame_;
  cloud_pub.publish(cloud);

  ROS_INFO_THROTTLE(1.0, "Time to create map %f",
      (ros::Time::now() - t0).toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "merge_maps");
  ros::NodeHandle nh("~");

  nh.param("world_frame", world_frame_, std::string("map"));

  ros::Subscriber cloud_sub = nh.subscribe("cloud_in", 1, cloudCallback, ros::TransportHints().tcpNoDelay());
  map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);

  double res;
  nh.param("resolution", res, 0.1);

  double origin_x, origin_y, origin_z;
  double range_x, range_y, range_z;
  nh.param("origin_x", origin_x, 0.0);
  nh.param("origin_y", origin_y, 0.0);
  nh.param("origin_z", origin_z, 0.0);
  nh.param("range_x", range_x, 10.0);
  nh.param("range_y", range_y, 10.0);
  nh.param("range_z", range_z, 3.0);
  Vec3f dim(range_x, range_y, range_z);
  Vec3f ori(origin_x, origin_y, origin_z);
  voxel_grid_.reset(new VoxelGridRGB(ori, dim, res));

  ros::spin();

  return 0;
}
