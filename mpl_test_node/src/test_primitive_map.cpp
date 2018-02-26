#include "bag_reader.hpp"
#include <ros/ros.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <motion_primitive_library/primitive/poly_solver.h>
#include <motion_primitive_library/planner/mp_map_util.h>

using namespace MPL;

std_msgs::Header header_;
ros::Publisher cloud_pub_;
std::vector<ros::Publisher> traj_pub_;

std::unique_ptr<MPMap3DUtil> planner_;

void setMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

void getMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, planning_ros_msgs::VoxelMap& map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  decimal_t res = map_util->getRes();

  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.resolution = res;

  map.data = map_util->getMap();
}


bool solve(const Waypoint3& start, const Waypoint3& goal) {
  ros::Time t0 = ros::Time::now();
  bool valid = planner_->plan(start, goal);

  //Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_->getCloseSet());
  ps.header = header_;
  cloud_pub_.publish(ps);

  if(!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());
    return false;
  }
  else
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());

  //Publish trajectory
  auto traj = planner_->getTraj();
  planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
  traj_msg.header = header_;
  traj_pub_[0].publish(traj_msg);

  printf("================== Traj -- total J: %f, total time: %f\n", traj.J(2), traj.getTotalTime());
  return true;
}


int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher traj_pub1 = nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher traj_pub2 = nh.advertise<planning_ros_msgs::Trajectory>("trajectory_refined", 1, true);
  traj_pub_.push_back(traj_pub1);
  traj_pub_.push_back(traj_pub2);

  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);

  header_.frame_id = std::string("map");
  //Read map from bag file
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  planning_ros_msgs::VoxelMap map = read_bag<planning_ros_msgs::VoxelMap>(file_name, topic_name, 0).back();


  //Initialize map util 
  std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
  setMap(map_util, map);

  //Free unknown space and dilate obstacles
  map_util->freeUnknown();
  //map_util->dilate(0.2, 0.1);
  //map_util->dilating();


  //Publish the dilated map for visualization
  getMap(map_util, map);
  map.header = header_;
  map_pub.publish(map);


  //Set start and goal
  double start_x, start_y, start_z;
  nh.param("start_x", start_x, 12.5);
  nh.param("start_y", start_y, 1.4);
  nh.param("start_z", start_z, 0.0);
  double start_vx, start_vy, start_vz;
  nh.param("start_vx", start_vx, 0.0);
  nh.param("start_vy", start_vy, 0.0);
  nh.param("start_vz", start_vz, 0.0);
  double goal_x, goal_y, goal_z;
  nh.param("goal_x", goal_x, 6.4);
  nh.param("goal_y", goal_y, 16.6);
  nh.param("goal_z", goal_z, 0.0);
 
  Waypoint3 start;
  start.pos = Vec3f(start_x, start_y, start_z);
  start.vel = Vec3f(start_vx, start_vy, start_vz);
  start.acc = Vec3f(0, 0, 0);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false;

  Waypoint3 goal;
  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;



  //Initialize planner
  double dt, v_max, a_max, u_max;
  int max_num, num;
  bool use_3d;
  nh.param("dt", dt, 1.0);
  nh.param("v_max", v_max, 2.0);
  nh.param("a_max", a_max, 1.0);
  nh.param("u_max", u_max, 1.0);
  nh.param("max_num", max_num, -1);
  nh.param("num", num, 1);
  nh.param("use_3d", use_3d, false);

  vec_Vec3f U;
  const decimal_t du = u_max / num;
  if(use_3d) {
    decimal_t du_z = u_max / num;
    for(decimal_t dx = -u_max; dx <= u_max; dx += du ) 
      for(decimal_t dy = -u_max; dy <= u_max; dy += du )
        for(decimal_t dz = -u_max; dz <= u_max; dz += du_z ) //here we reduce the z control
          U.push_back(Vec3f(dx, dy, dz));
  }
  else {
    for(decimal_t dx = -u_max; dx <= u_max; dx += du ) 
      for(decimal_t dy = -u_max; dy <= u_max; dy += du )
        U.push_back(Vec3f(dx, dy, 0));
  }
 

  planner_.reset(new MPMap3DUtil(true));
  planner_->setMapUtil(map_util); // Set collision checking function
  planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner_->setVmax(v_max); // Set max velocity
  planner_->setAmax(a_max); // Set max acceleration (as control input)
  planner_->setUmax(u_max);// 2D discretization with 1
  planner_->setDt(dt); // Set dt for each primitive
  planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
  planner_->setU(U);// 2D discretization with 1
  planner_->setTol(1, 1, 1); // Tolerance for goal region


  //Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header = header_;
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
  pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2); 
  sg_pub.publish(sg_cloud);

  //Planning thread!
  if(solve(start, goal)) {

    //Get intermediate waypoints
    vec_E<Waypoint3> waypoints = planner_->getWs();
    //Get time allocation
    std::vector<decimal_t> dts;
    dts.resize(waypoints.size() - 1, dt);

    //Generate higher order polynomials
    PolySolver3 poly_solver(2,3);
    poly_solver.solve(waypoints, dts);

    auto traj_refined = Trajectory3(poly_solver.getTrajectory()->toPrimitives());

    //Publish refined trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj_refined);
    traj_msg.header = header_;
    traj_pub_[1].publish(traj_msg);

    printf("================ Refined traj -- total J: %f, total time: %f\n", traj_refined.J(2), traj_refined.getTotalTime());
  }



  ros::spin();

  return 0;
}
