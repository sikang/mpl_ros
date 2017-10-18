#include "bag_reader.hpp"
#include <std_msgs/Bool.h>
#include <ros_utils/data_ros_utils.h>
#include <ros_utils/primitive_ros_utils.h>
#include <ros_utils/mapping_ros_utils.h>
#include <planner/mp_map_util.h>

using namespace MPL;

std::unique_ptr<MPMapUtil> planner_;

ros::Publisher sg_pub;
ros::Publisher prs_pub;
ros::Publisher traj_pub;
ros::Publisher close_cloud_pub;
ros::Publisher open_cloud_pub;
std_msgs::Header header;

Waypoint start, goal;
bool terminated = false;

void replanCallback(const std_msgs::Bool::ConstPtr& msg) {
  if(terminated)
    return;
  //Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header = header;
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start.pos(0), pt1.y = start.pos(1), pt1.z = start.pos(2);
  pt2.x = goal.pos(0), pt2.y = goal.pos(1), pt2.z = goal.pos(2);
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2); 
  sg_pub.publish(sg_cloud);

  ros::Time t0 = ros::Time::now();
  bool valid = planner_->plan(start, goal, msg->data);

  //Publish expanded nodes
  sensor_msgs::PointCloud close_ps = vec_to_cloud(planner_->getCloseSet());
  close_ps.header = header;
  close_cloud_pub.publish(close_ps);

  //Publish nodes in open set
  sensor_msgs::PointCloud open_ps = vec_to_cloud(planner_->getOpenSet());
  open_ps.header = header;
  open_cloud_pub.publish(open_ps);


  if(!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());
    terminated = true;
  }
  else{
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());

    //Publish trajectory
    Trajectory traj = planner_->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    traj_pub.publish(traj_msg);

    printf("================== Traj -- total J: %f, total time: %f\n", traj.J(1), traj.getTotalTime());

    planning_ros_msgs::Primitives prs_msg = toPrimitivesROSMsg(planner_->getPrimitives());
    prs_msg.header =  header;
    prs_pub.publish(prs_msg);

    std::vector<Waypoint> ws = planner_->getWs();
    if(ws.size() < 3)
      terminated = true;
    else {
      start = ws[1];
      start.t = 0;
    }
  }
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Subscriber replan_sub = nh.subscribe("replan", 1, replanCallback);
  ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  prs_pub = nh.advertise<planning_ros_msgs::Primitives>("primitives", 1, true);
  traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  close_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("close_cloud", 1, true);
  open_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("open_set", 1, true);

  header.frame_id = std::string("map");
  //Read map from bag file
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  planning_ros_msgs::VoxelMap map = read_bag<planning_ros_msgs::VoxelMap>(file_name, topic_name);

  //Initialize map util 
  std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
  setMap(map_util.get(), map);

  //Free unknown space and dilate obstacles
  map_util->freeUnKnown();
  map_util->dilate(0.2, 0.1);
  map_util->dilating();


  //Publish the dilated map for visualization
  getMap(map_util.get(), map);
  map.header = header;
  map_pub.publish(map);

  bool replan;
  nh.param("replan", replan, false);

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
 
  start.pos = Vec3f(start_x, start_y, start_z);
  start.vel = Vec3f(start_vx, start_vy, start_vz);
  start.acc = Vec3f(0, 0, 0);
  start.t = 0;
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false;

  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;


  //Initialize planner
  double dt, v_max, a_max;
  int max_num, ndt;
  bool use_3d;
  nh.param("dt", dt, 1.0);
  nh.param("ndt", ndt, -1);
  nh.param("v_max", v_max, 2.0);
  nh.param("a_max", a_max, 1.0);
  nh.param("max_num", max_num, -1);
  nh.param("use_3d", use_3d, false);

  planner_.reset(new MPMapUtil(true));
  planner_->setMapUtil(map_util); // Set collision checking function
  planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner_->setVmax(v_max); // Set max velocity
  planner_->setAmax(a_max); // Set max acceleration (as control input)
  planner_->setUmax(a_max);// 2D discretization with 1
  planner_->setDt(dt); // Set dt for each primitive
  planner_->setTmax(ndt * dt); // Set dt for each primitive
  planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
  planner_->setU(1, false);// 2D discretization with 1
  planner_->setMode(start); // use acc as control
  planner_->setTol(1, 1, 1); // Tolerance for goal region

  //Planning thread!
  
  std_msgs::Bool init;
  init.data = false;
  replanCallback(boost::make_shared<std_msgs::Bool>(init));

  ros::spin();

  return 0;
}
