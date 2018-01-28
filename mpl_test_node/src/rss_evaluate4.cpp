#include "bag_reader.hpp"
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <ros_utils/data_ros_utils.h>
#include <ros_utils/primitive_ros_utils.h>
#include <primitive/poly_solver.h>
#include <mapping_utils/voxel_grid.h>
#include <planner/mp_map_util.h>
#include <fstream>

using namespace MPL;

std::unique_ptr<VoxelGrid> voxel_mapper_;
std::shared_ptr<MPL::VoxelMapUtil> map_util_;

Vec3i dim_;
Waypoint start_, goal_;
std_msgs::Header header;

std::unique_ptr<MPMapUtil> planner_;
std::unique_ptr<MPMapUtil> replan_planner_;

ros::Publisher map_pub;
ros::Publisher sg_pub;
ros::Publisher changed_prs_pub;
std::vector<ros::Publisher> prs_pub;
std::vector<ros::Publisher> traj_pub;
std::vector<ros::Publisher> linked_cloud_pub;
std::vector<ros::Publisher> close_cloud_pub;
std::vector<ros::Publisher> open_cloud_pub;
std::vector<ros::Publisher> expanded_cloud_pub;

std::ofstream myfile;
int addition_num_ = 50;
int obs_number_ = 0;
double density_ = 0;

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

Vec3i generate_point() {
  static std::random_device rd;  //Will be used to obtain a seed for the random number engine
  static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<int> dis_x(0, dim_(0)-1);
  std::uniform_int_distribution<int> dis_y(0, dim_(1)-1);
  std::uniform_int_distribution<int> dis_z(0, dim_(2)-1);

  return Vec3i(dis_x(gen), dis_y(gen), dis_z(gen));
}



void plan(bool record) {
  static bool terminate = false;
  if(terminate)
    return;
  ros::Time t0 = ros::Time::now();
  double dt0 = 0;
  bool valid = planner_->plan(start_, goal_);
  if(!valid) {
    ROS_ERROR("Failed! Takes %f sec for planning", (ros::Time::now() - t0).toSec());
    terminate = true;
  }
  else{
    dt0 = (ros::Time::now() - t0).toSec();
    if(record)
      ROS_WARN("Succeed! Takes %f sec for normal planning, openset: [%zu], closeset (expanded): [%zu](%zu), total: [%zu]", 
          dt0, planner_->getOpenSet().size(), planner_->getCloseSet().size(), 
          planner_->getExpandedNodes().size(),
          planner_->getOpenSet().size() + planner_->getCloseSet().size());
  }

  t0 = ros::Time::now();
  double dt1 = 0;
  valid = replan_planner_->plan(start_, goal_);
  if(!valid) {
    ROS_ERROR("Failed! Takes %f sec for planning", (ros::Time::now() - t0).toSec());
  }
  else{
    dt1 = (ros::Time::now() - t0).toSec();

    if(record)
      ROS_WARN("Succeed! Takes %f sec for LPA* planning, openset: [%zu], closeset (expanded): [%zu](%zu), total: [%zu]", 
          dt1, replan_planner_->getOpenSet().size(), replan_planner_->getCloseSet().size(), 
          replan_planner_->getExpandedNodes().size(),
          replan_planner_->getOpenSet().size() + replan_planner_->getCloseSet().size());

  }

  replan_planner_->getLinkedNodes();
  if(record) {
    myfile << std::to_string(density_) + "," + std::to_string(dt0) + "," +
      std::to_string(dt1) + "," + std::to_string(planner_->getExpandedNum()) + "," + std::to_string(replan_planner_->getExpandedNum()) + "\n";
    printf(ANSI_COLOR_CYAN "==========================================\n\n" ANSI_COLOR_RESET);
  }
}

void add_points(int num) {
  int cnt = 0;
  vec_Vec3i new_obs;
  while(cnt < num) {
    Vec3i pn = generate_point();
    if(map_util_->isFree(pn)) {
      const Vec3f pt = map_util_->intToFloat(pn);
      if((pt - start_.pos).topRows(2).norm() < 0.5 ||
          (pt - goal_.pos).topRows(2).norm() < 0.5)
        continue;
      for(int i = 0; i < dim_(2); i++)
        new_obs.push_back(Vec3i(pn(0), pn(1), i));
      voxel_mapper_->fill(pn(0), pn(1));
      cnt ++;
    }
  }

  planning_ros_msgs::VoxelMap map = voxel_mapper_->getMap();
  setMap(map_util_, map);

  if(replan_planner_->initialized()) 
    toPrimitivesROSMsg(replan_planner_->updateBlockedNodes(new_obs));

  obs_number_ += cnt;
  density_ = (float) obs_number_ / (dim_(0) * dim_(1));
  printf("Density: %f\n", density_);
}

void clear_points(int num) {
  int cnt = 0;
  vec_Vec3i new_obs;
  while(cnt < num) {
    Vec3i pn = generate_point();
    if(map_util_->isOccupied(pn)) {
      const Vec3f pt = map_util_->intToFloat(pn);
      for(int i = 0; i < dim_(2); i++)
        new_obs.push_back(Vec3i(pn(0), pn(1), i));
      voxel_mapper_->clear(pn(0), pn(1));
      cnt ++;
    }
  }

  planning_ros_msgs::VoxelMap map = voxel_mapper_->getMap();
  setMap(map_util_, map);

  if(replan_planner_->initialized()) 
    toPrimitivesROSMsg(replan_planner_->updateClearedNodes(new_obs));

  obs_number_ -= cnt;
  density_ = (float) obs_number_ / (dim_(0) * dim_(1));
  printf("Density: %f\n", density_);
}




int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);

  ros::Publisher prs_pub0 = nh.advertise<planning_ros_msgs::Primitives>("primitives0", 1, true);
  ros::Publisher prs_pub1 = nh.advertise<planning_ros_msgs::Primitives>("primitives1", 1, true);
  prs_pub.push_back(prs_pub0), prs_pub.push_back(prs_pub1);

  ros::Publisher traj_pub0 = nh.advertise<planning_ros_msgs::Trajectory>("trajectory0", 1, true);
  ros::Publisher traj_pub1 = nh.advertise<planning_ros_msgs::Trajectory>("trajectory1", 1, true);
  traj_pub.push_back(traj_pub0), traj_pub.push_back(traj_pub1);

  ros::Publisher close_cloud_pub0 = nh.advertise<sensor_msgs::PointCloud>("close_cloud0", 1, true);
  ros::Publisher close_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud>("close_cloud1", 1, true);
  close_cloud_pub.push_back(close_cloud_pub0), close_cloud_pub.push_back(close_cloud_pub1);

  ros::Publisher open_cloud_pub0 = nh.advertise<sensor_msgs::PointCloud>("open_set0", 1, true);
  ros::Publisher open_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud>("open_set1", 1, true);
  open_cloud_pub.push_back(open_cloud_pub0), open_cloud_pub.push_back(open_cloud_pub1);

  ros::Publisher linked_cloud_pub0 = nh.advertise<sensor_msgs::PointCloud>("linked_pts0", 1, true);
  ros::Publisher linked_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud>("linked_pts1", 1, true);
  linked_cloud_pub.push_back(linked_cloud_pub0), linked_cloud_pub.push_back(linked_cloud_pub1);

  ros::Publisher expanded_cloud_pub0 = nh.advertise<sensor_msgs::PointCloud>("expanded_cloud0", 1, true);
  ros::Publisher expanded_cloud_pub1 = nh.advertise<sensor_msgs::PointCloud>("expanded_cloud1", 1, true);
  expanded_cloud_pub.push_back(expanded_cloud_pub0), expanded_cloud_pub.push_back(expanded_cloud_pub1);

  header.frame_id = std::string("map");

  Vec3f ori, dim;
  nh.param("origin_x", ori(0), 0.0);
  nh.param("origin_y", ori(1), 2.5);
  nh.param("origin_z", ori(2), 0.0);

  nh.param("range_x", dim(0), 10.0);
  nh.param("range_y", dim(1), 5.0);
  nh.param("range_z", dim(2), 1.0);

  double res;
  nh.param("resolution", res, 0.1);

  dim_(0) = dim(0) / res;
  dim_(1) = dim(1) / res;
  dim_(2) = dim(2) / res;

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
 
  start_.pos = Vec3f(start_x, start_y, start_z);
  start_.vel = Vec3f(start_vx, start_vy, start_vz);
  start_.acc = Vec3f(0, 0, 0);
  start_.use_pos = true;
  start_.use_vel = true;
  start_.use_acc = true;
  start_.use_jrk = false;

  goal_.pos = Vec3f(goal_x, goal_y, goal_z);
  goal_.vel = Vec3f(0, 0, 0);
  goal_.acc = Vec3f(0, 0, 0);
  goal_.use_pos = start_.use_pos;
  goal_.use_vel = start_.use_vel;
  goal_.use_acc = start_.use_acc;
  goal_.use_jrk = start_.use_jrk;


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


  myfile.open("record-clear-"+std::to_string(addition_num_)+".csv");
  myfile << "Obstacle Density,NormalAstar Time,LAstar Time,NormalAstar Expansion,LPAstar Expansion\n";

  int base_num = 0.2 * dim_(0) * dim_(1);

  while (base_num > addition_num_) {
    for(int i = 0; i < 20; i++) {
      voxel_mapper_.reset(new VoxelGrid(ori, dim, res));
      map_util_.reset(new MPL::VoxelMapUtil);

      planning_ros_msgs::VoxelMap map = voxel_mapper_->getMap();
      setMap(map_util_, map);
      map_util_->freeUnKnown();

      planner_.reset(new MPMapUtil(false));

      planner_->setMapUtil(map_util_); // Set collision checking function
      planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
      planner_->setVmax(v_max); // Set max velocity
      planner_->setAmax(a_max); // Set max acceleration
      planner_->setJmax(j_max); // Set jrk (as control input)
      planner_->setUmax(u_max);// 2D discretization with 1
      planner_->setDt(dt); // Set dt for each primitive
      planner_->setTmax(ndt * dt); // Set dt for each primitive
      planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
      planner_->setU(1, false);// 2D discretization with 1
      planner_->setTol(0.2, 1, 1); // Tolerance for goal region
      planner_->setLPAstar(false); // Use Astar

      replan_planner_.reset(new MPMapUtil(false));

      replan_planner_->setMapUtil(map_util_); // Set collision checking function
      replan_planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
      replan_planner_->setVmax(v_max); // Set max velocity
      replan_planner_->setAmax(a_max); // Set max acceleration (as control input)
      replan_planner_->setJmax(j_max); // Set jrk (as control input)
      replan_planner_->setUmax(u_max);// 2D discretization with 1
      replan_planner_->setDt(dt); // Set dt for each primitive
      replan_planner_->setTmax(ndt * dt); // Set dt for each primitive
      replan_planner_->setMaxNum(-1); // Set maximum allowed expansion, -1 means no limitation
      replan_planner_->setU(1, false);// 2D discretization with 1
      replan_planner_->setTol(0.2, 1, 1); // Tolerance for goal region
      replan_planner_->setLPAstar(true); // Use LPAstar

      //replan_planner_.reset();

      density_ = 0;
      obs_number_ = 0;

      add_points(base_num);

      plan(false);

      clear_points(addition_num_);
      plan(true);
    }
    base_num -= addition_num_;
    printf(ANSI_COLOR_GREEN "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! base_num %d\n" ANSI_COLOR_RESET, base_num);
  }

  myfile.close();
  //ros::spin();

  return 0;
}
