#include "bag_reader.hpp"
#include <ros_utils/data_ros_utils.h>
#include <ros_utils/primitive_ros_utils.h>
#include <ros_utils/mapping_ros_utils.h>
#include <primitive/poly_solver.h>
#include <planner/mp_map_util.h>

using namespace MPL;
std_msgs::Header header_;
std::shared_ptr<MPL::VoxelMapUtil> map_util_;
std::vector<ros::Publisher> traj_pub;
std::vector<ros::Publisher> cloud_pub;
std::unique_ptr<MPMapUtil> planner_util_;

void solve(const Waypoint& start, const Waypoint& goal) {
  ros::Time t0 = ros::Time::now();

  planner_util_->setEpsilon(1.0);

  planner_util_->setVmax(2.0);
  planner_util_->setAmax(1.0);
  planner_util_->setDt(1.0);

  bool valid = planner_util_->plan(start, goal);
  /*
  Trajectory traj = planner_util_->getTraj();
  planning_ros_msgs::Trajectory primitives = toROSMsg(traj);
  primitives.header = header_;
  traj_pub[0].publish(primitives);
  */

  sensor_msgs::PointCloud ps = vec_to_cloud(planner_util_->getPs());
  ps.header = header_;
  cloud_pub[0].publish(ps);


  if(!valid) {
    ROS_WARN("Takes %f sec for nx_planning", (ros::Time::now() - t0).toSec());
    return;
  }
  ROS_INFO("Takes %f sec for nx_planning", (ros::Time::now() - t0).toSec());


  Trajectory traj = planner_util_->getTraj();
  planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
  traj_msg.header = header_;
  traj_pub[0].publish(traj_msg);

  printf("Nodes: %zu, J: %f\n", planner_util_->getPs().size(), traj.J(1));
}


void process_path(const vec_Vec3f& path) {
  if(path.size() < 2)
    return;

  Vec3f p1 = path.front();
  Vec3f p2 = path.back();

  Waypoint start;
  start.pos = p1;
  start.vel = Vec3f(0, 0, 0);
  start.acc = Vec3f(0, 0, 0);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false;

  Waypoint goal;
  goal.pos = p2;
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;

  solve(start, goal);
}

void pathCallack(const nav_msgs::Path::ConstPtr& msg) {
  vec_Vec3f path = path_to_eigen(*msg);
  process_path(path);
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Subscriber path_sub = nh.subscribe("waypoints", 1, pathCallack);
  //ros::Subscriber pose_sub = nh.subscribe("pose", 1, poseCallback);
  ros::Publisher map_pub =
    nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher traj_pub1 = nh.advertise<planning_ros_msgs::Trajectory>("trajectory1", 1, true);
  ros::Publisher traj_pub2 = nh.advertise<planning_ros_msgs::Trajectory>("trajectory2", 1, true);
  ros::Publisher cloud_pub1 = nh.advertise<sensor_msgs::PointCloud>("cloud1", 1, true);
  ros::Publisher cloud_pub2 = nh.advertise<sensor_msgs::PointCloud>("cloud2", 1, true);
  cloud_pub.push_back(cloud_pub1);
  cloud_pub.push_back(cloud_pub2);
  traj_pub.push_back(traj_pub1);
  traj_pub.push_back(traj_pub2);

  header_.frame_id = std::string("map");
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  planning_ros_msgs::VoxelMap map = read_bag(file_name, topic_name);
  map.header = header_;
  map_pub.publish(map);


  map_util_.reset(new MPL::VoxelMapUtil);
  setMap(map_util_.get(), map);

  map_util_->freeUnKnown();
  map_util_->dilate(0.2, 0.1);
  map_util_->dilating();

  planner_util_.reset(new MPMapUtil(true));

  planner_util_->setMapUtil(map_util_);

  vec_Vec3f path;
  path.push_back(Vec3f(12.5, 1.4, 0));
  path.push_back(Vec3f(6.3, 16.6, 0));
  //process_path(path);
  //planner_util_->info();
  ros::spin();

  return 0;
}
