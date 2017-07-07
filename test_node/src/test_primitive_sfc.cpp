#include "bag_reader.hpp"
#include <decomp_util/ellipse_decomp.h>
#include <ros_utils/data_ros_utils.h>
#include <ros_utils/primitive_ros_utils.h>
#include <ros_utils/mapping_ros_utils.h>
#include <primitive/poly_solver.h>
#include <planner/mp_map_util.h>
#include <planner/mp_sfc_util.h>
#include <planner/nx_jps_3d.h>

std_msgs::Header header_;
std::shared_ptr<MPL::VoxelMapUtil> map_util_;
std::shared_ptr<JPS::VoxelMapUtil> jps_map_util_;
ros::Publisher path_pub;
ros::Publisher poly_pub;
std::vector<ros::Publisher> traj_pub;
std::vector<ros::Publisher> cloud_pub;
std::unique_ptr<MPMapUtil> planner_map_util_;
std::unique_ptr<MPSFCUtil> planner_sfc_util_;
std::unique_ptr<NXJPS3DUtil> jps_;
std::unique_ptr<EllipseDecomp> decomp_util_;

void solve(const Waypoint& start, const Waypoint& goal) {
  ros::Time t0 = ros::Time::now();

  bool valid = jps_->plan(start.pos, goal.pos);

  planning_ros_msgs::Path path = path_to_mav(jps_->getPath());
  path.header = header_;
  path_pub.publish(path);

  if(!valid) {
    ROS_WARN("Takes %f sec for nx_planning", (ros::Time::now() - t0).toSec());
    return;
  }
  ROS_INFO("Takes %f sec for nx_planning", (ros::Time::now() - t0).toSec());
  t0 = ros::Time::now();

  decomp_util_->decomp(jps_->getPath());
  planning_ros_msgs::Polyhedra poly_msg = polyhedra_to_mav(decomp_util_->get_polyhedra());
  poly_msg.header = header_;
  poly_pub.publish(poly_msg); 

  planner_sfc_util_->setMap(decomp_util_->get_polyhedra());
  planner_sfc_util_->setEpsilon(1.0);

  planner_sfc_util_->setVmax(4.0);
  planner_sfc_util_->setAmax(5.0);
  planner_sfc_util_->setDt(1.0);

  if(planner_sfc_util_->plan(start, goal)) {
    Trajectory traj = planner_sfc_util_->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header_;
    traj_pub[0].publish(traj_msg);
  }
  ROS_INFO("Takes %f sec for sfc planning", (ros::Time::now() - t0).toSec());

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
  path_pub = nh.advertise<planning_ros_msgs::Path>("path", 1, true);
  poly_pub = nh.advertise<planning_ros_msgs::Polyhedra>("polyhedra", 1, true);
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


  map_util_.reset(new MPL::VoxelMapUtil());
  setMap(map_util_.get(), map);

  map_util_->freeUnKnown();
  map_util_->dilate(0.2, 0.1);
  map_util_->dilating();


  jps_map_util_.reset(new JPS::VoxelMapUtil());
  toJPSMapUtil(map_util_.get(), jps_map_util_.get());

  planner_map_util_.reset(new MPMapUtil(true));
  planner_map_util_->setMapUtil(map_util_);

  planner_sfc_util_.reset(new MPSFCUtil(true));

  jps_.reset(new NXJPS3DUtil(true));
  jps_->setMapUtil(jps_map_util_.get());

  decomp_util_.reset(new EllipseDecomp(map_util_->getOrigin(),
        map_util_->getDim().cast<decimal_t>()*map_util_->getRes(), true));
  decomp_util_->set_obstacles(map_util_->getCloud());

  vec_Vec3f path;
  path.push_back(Vec3f(12.5, 1.4, 0));
  path.push_back(Vec3f(6.3, 16.6, 0));
  //process_path(path);
  //planner_util_->info();
  ros::spin();

  return 0;
}
