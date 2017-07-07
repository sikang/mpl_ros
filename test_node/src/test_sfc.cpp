#include "bag_reader.hpp"
#include <decomp_util/ellipse_decomp.h>
#include <ros_utils/data_ros_utils.h>
// #include <ros_utils/primitive_ros_utils.h>
#include <ros_utils/mapping_ros_utils.h>

std_msgs::Header header_;
std::shared_ptr<MPL::VoxelMapUtil> map_util_;
std::unique_ptr<EllipseDecomp> decomp_util_;
ros::Publisher poly_pub;

void pathCallack(const nav_msgs::Path::ConstPtr& msg) {
  vec_Vec3f path = path_to_eigen(*msg);
  decomp_util_->decomp(path);
  planning_ros_msgs::Polyhedra poly_msg = polyhedra_to_mav(decomp_util_->get_polyhedra());
  poly_msg.header = header_;
  poly_pub.publish(poly_msg); 
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Subscriber path_sub = nh.subscribe("waypoints", 1, pathCallack);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher map_pub =
    nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
 
  poly_pub = nh.advertise<planning_ros_msgs::Polyhedra>("polyhedra", 1, true);

  header_.frame_id = std::string("map");
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));

  planning_ros_msgs::VoxelMap map = read_bag(file_name, topic_name);
  map.header = header_;
  map_pub.publish(map);

  map_util_.reset(new MPL::VoxelMapUtil());
  setMap(map_util_.get(), map);

  decomp_util_.reset(new EllipseDecomp(map_util_->getOrigin(),
        map_util_->getDim().cast<decimal_t>()*map_util_->getRes(), true));
  decomp_util_->set_obstacles(map_util_->getCloud());

  sensor_msgs::PointCloud cloud_msg  = vec_to_cloud(map_util_->getCloud());
  cloud_msg.header = header_;
  cloud_pub.publish(cloud_msg);

  ros::spin();

  return 0;
}
