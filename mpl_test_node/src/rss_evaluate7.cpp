#include "bag_reader.hpp"
#include <ros/ros.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <ros_utils/data_ros_utils.h>
#include <ros_utils/primitive_ros_utils.h>
#include <primitive/poly_solver.h>
#include <mapping_utils/voxel_grid.h>
#include <planner/mp_map_util.h>

using namespace MPL;

std::unique_ptr<MPMapUtil> planner_;
std::unique_ptr<VoxelGrid> voxel_mapper_;

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


int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  std::vector<ros::Publisher> traj_pubs;
  int traj_num = 5;

  for(int i = 0; i < traj_num; i++) {
    ros::Publisher traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("traj"+std::to_string(i), 1, true);
    traj_pubs.push_back(traj_pub);
  }
  ros::Publisher traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("traj"+std::to_string(traj_num), 1, true);
  traj_pubs.push_back(traj_pub);

  std_msgs::Header header;
  header.frame_id = std::string("map");
  //Read map from bag file
  std::string file_name, topic_name, cloud_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  planning_ros_msgs::VoxelMap map = read_bag<planning_ros_msgs::VoxelMap>(file_name, topic_name, 0).back();
  nh.param("cloud_name", cloud_name, std::string("cloud"));
  sensor_msgs::PointCloud cloud = read_bag<sensor_msgs::PointCloud>(file_name, cloud_name, 0).back();

  double res = map.resolution;
  Vec3f origin(map.origin.x, map.origin.y, map.origin.z);
  Vec3f dim(map.dim.x * res, map.dim.y * res, map.dim.z * res);

  voxel_mapper_.reset(new VoxelGrid(origin, dim, res));
  voxel_mapper_->addCloud(cloud_to_vec(cloud));

  //Initialize map util 
  std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
  setMap(map_util, map);

 
  //map_util->dilate(0.2, 0.1);
  //map_util->dilating();

  Vec3f p1(4, 5, 0.2);
  Vec3f p2(12, 5, 0.2);
  vec_Vec3i pns = map_util->rayTrace(p1, p2);

  for(int i = 1; i < 5; i++) {
    p1(1) -= 0.1, p2(1) -= 0.1;
    vec_Vec3i pns1 = map_util->rayTrace(p1, p2);
    pns.insert(pns.end(), pns1.begin(), pns1.end());
  }

  Vec3f p3(0, 8, 0.2);
  Vec3f p4(2, 8, 0.2);
  vec_Vec3i pns2 = map_util->rayTrace(p3, p4);
  pns.insert(pns.end(), pns2.begin(), pns2.end());

  for(int i = 1; i < 5; i++) {
    p3(1) -= 0.1, p4(1) -= 0.1;
    vec_Vec3i pns1 = map_util->rayTrace(p3, p4);
    pns.insert(pns.end(), pns1.begin(), pns1.end());
  }


  Vec3f p5(12.75, 9.55, 0.2);
  Vec3f p6(12.75, 12.5, 0.2);
  vec_Vec3i pns3 = map_util->rayTrace(p5, p6);
  pns.insert(pns.end(), pns3.begin(), pns3.end());

  for(int i = 1; i < 5; i++) {
    p5(0) -= 0.1, p6(0) -= 0.1;
    vec_Vec3i pns1 = map_util->rayTrace(p5, p6);
    pns.insert(pns.end(), pns1.begin(), pns1.end());
  }

  for(const auto& pn: pns) {
    if(!map_util->isOccupied(pn)) 
      voxel_mapper_->fill(pn(0), pn(1));
  }



  for(const auto& pn: pns) {
    if(!map_util->isOccupied(pn)) 
      voxel_mapper_->fill(pn(0), pn(1));
  }

  map = voxel_mapper_->getMap();

  setMap(map_util, map);
  //Free unknown space and dilate obstacles
  map_util->freeUnKnown();
 
  //map_util->freeUnKnown();

  //Publish the dilated map for visualization
  getMap(map_util, map);
  map.header = header;
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
  double start_ax, start_ay, start_az;
  nh.param("start_ax", start_ax, 0.0);
  nh.param("start_ay", start_ay, 0.0);
  nh.param("start_az", start_az, 0.0);
 
  double goal_x, goal_y, goal_z;
  nh.param("goal_x", goal_x, 6.4);
  nh.param("goal_y", goal_y, 16.6);
  nh.param("goal_z", goal_z, 0.0);
 
  Waypoint start;
  start.pos = Vec3f(start_x, start_y, start_z);
  start.vel = Vec3f(start_vx, start_vy, start_vz);
  start.acc = Vec3f(start_ax, start_ay, start_az);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true;

  Waypoint goal;
  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;

 //Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header = header;
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start.pos(0), pt1.y = start.pos(1), pt1.z = start.pos(2)+0.5;
  pt2.x = goal.pos(0), pt2.y = goal.pos(1), pt2.z = goal.pos(2)+0.5;
  sg_cloud.points.push_back(pt1);
  sg_cloud.points.push_back(pt2); 
  sg_pub.publish(sg_cloud);



  //Initialize planner
  double dt, v_max, a_max, j_max, u_max;
  int max_num, ndt;
  bool use_3d;
  nh.param("dt", dt, 1.0);
  nh.param("ndt", ndt, -1);
  nh.param("v_max", v_max, 2.0);
  nh.param("a_max", a_max, 1.0);
  nh.param("j_max", j_max, 1.0);
  nh.param("u_max", u_max, 1.0);
  nh.param("max_num", max_num, -1);
  nh.param("use_3d", use_3d, false);

  int init_i = 3;
  for(int i = init_i; i < init_i + traj_num * 2; i+=2) {
    planner_.reset(new MPMapUtil(true));
    planner_->setMapUtil(map_util); // Set collision checking function
    planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
    planner_->setVmax(v_max); // Set max velocity
    planner_->setAmax(a_max); // Set max acceleration (as control input)
    planner_->setJmax(j_max); // Set max acceleration (as control input)
    planner_->setUmax(u_max);// 2D discretization with 1
    planner_->setDt(dt); // Set dt for each primitive
    planner_->setTmax(dt*i); // Set dt for each primitive
    planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
    planner_->setU(1, false);// 2D discretization with 1
    planner_->setTol(1, 1, 1); // Tolerance for goal region
    planner_->setLPAstar(true); // Use LPAstar


    //Planning thread!
    ros::Time t0 = ros::Time::now();
    bool valid = planner_->plan(start, goal);
    ROS_WARN("Time: %f, Expansions: %d", (ros::Time::now() - t0).toSec(), planner_->getExpandedNum());
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(planner_->getTraj());
    traj_msg.header = header;
    traj_pubs[(i-init_i)/2].publish(traj_msg);
  }

  planner_.reset(new MPMapUtil(true));
  planner_->setMapUtil(map_util); // Set collision checking function
  planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner_->setVmax(v_max); // Set max velocity
  planner_->setAmax(a_max); // Set max acceleration (as control input)
  planner_->setJmax(j_max); // Set max acceleration (as control input)
  planner_->setUmax(u_max);// 2D discretization with 1
  planner_->setDt(dt); // Set dt for each primitive
  planner_->setTmax(-1); // Set dt for each primitive
  planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
  planner_->setU(1, false);// 2D discretization with 1
  planner_->setTol(1, 1, 1); // Tolerance for goal region
  planner_->setLPAstar(true); // Use LPAstar

  ros::Time t0 = ros::Time::now();
  bool valid = planner_->plan(start, goal);
  ROS_WARN("Time: %f, Expansions: %d", (ros::Time::now() - t0).toSec(), planner_->getExpandedNum());
  Trajectory traj = planner_->getTraj();
  for(auto &seg: traj.segs)
    seg.prs_[2].c(5) += 0.1;

  planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
  traj_msg.header = header;
  traj_pubs[traj_num].publish(traj_msg);


  ros::spin();

  return 0;
}
