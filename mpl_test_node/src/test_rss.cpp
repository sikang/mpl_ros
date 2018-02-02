#include "bag_reader.hpp"
#include <ros/ros.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <ros_utils/data_ros_utils.h>
#include <ros_utils/primitive_ros_utils.h>
#include <primitive/poly_solver.h>
#include <planner/mp_map_util.h>

using namespace MPL;

std::unique_ptr<MPMapUtil> planner_;

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
  ros::Publisher block_pub = nh.advertise<sensor_msgs::PointCloud>("block", 1, true);
  ros::Publisher end_pub = nh.advertise<sensor_msgs::PointCloud>("end", 1, true);

  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher openset_pub = nh.advertise<sensor_msgs::PointCloud>("open", 1, true);
  ros::Publisher sub_openset_pub = nh.advertise<sensor_msgs::PointCloud>("sub_open", 1, true);
  ros::Publisher closeset_pub = nh.advertise<sensor_msgs::PointCloud>("close", 1, true);
  ros::Publisher sub_closeset_pub = nh.advertise<sensor_msgs::PointCloud>("sub_close", 1, true);
  ros::Publisher nullset_pub = nh.advertise<sensor_msgs::PointCloud>("null", 1, true);
  ros::Publisher sub_nullset_pub = nh.advertise<sensor_msgs::PointCloud>("sub_null", 1, true);
  ros::Publisher prs_pub = nh.advertise<planning_ros_msgs::Primitives>("prs", 1, true);
  ros::Publisher changed_prs_pub = nh.advertise<planning_ros_msgs::Primitives>("changed_prs", 1, true);
  ros::Publisher sub_prs_pub = nh.advertise<planning_ros_msgs::Primitives>("sub_prs", 1, true);

  std_msgs::Header header;
  header.frame_id = std::string("map");
  //Read map from bag file
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  planning_ros_msgs::VoxelMap map = read_bag<planning_ros_msgs::VoxelMap>(file_name, topic_name, 0).back();


  //Initialize map util 
  std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
  setMap(map_util, map);

  //Free unknown space and dilate obstacles
  map_util->freeUnKnown();
  
  //map_util->dilate(0.2, 0.1);
  //map_util->dilating();


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

  planner_.reset(new MPMapUtil(true));
  planner_->setMapUtil(map_util); // Set collision checking function
  planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner_->setVmax(v_max); // Set max velocity
  planner_->setAmax(a_max); // Set max acceleration (as control input)
  planner_->setJmax(j_max); // Set max acceleration (as control input)
  planner_->setUmax(u_max);// 2D discretization with 1
  planner_->setDt(dt); // Set dt for each primitive
  planner_->setTmax(dt*ndt); // Set dt for each primitive
  planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
  planner_->setU(1, false);// 2D discretization with 1
  planner_->setTol(1, 1, 1); // Tolerance for goal region
  planner_->setLPAstar(true); // Use LPAstar


  //Planning thread!
  bool valid = planner_->plan(start, goal);

  /*
  //Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_->getCloseSet());
  ps.header = header;
  cloud_pub.publish(ps);
  */

  //Publish nodes linked nodes
  sensor_msgs::PointCloud linked_ps = vec_to_cloud(planner_->getLinkedNodes());
  linked_ps.header = header;
  cloud_pub.publish(linked_ps);

  //Publish nodes in close set
  sensor_msgs::PointCloud close_ps = vec_to_cloud(planner_->getCloseSet());
  close_ps.header = header;
  closeset_pub.publish(close_ps);

  //Publish nodes in open set
  sensor_msgs::PointCloud open_ps = vec_to_cloud(planner_->getOpenSet());
  open_ps.header = header;
  openset_pub.publish(open_ps);

  //Publish nodes in null set
  sensor_msgs::PointCloud null_ps = vec_to_cloud(planner_->getNullSet());
  null_ps.header = header;
  nullset_pub.publish(null_ps);


  planning_ros_msgs::Primitives prs_msg = toPrimitivesROSMsg(planner_->getAllPrimitives());
  prs_msg.header = header;
  prs_pub.publish(prs_msg);


  if(0) {
    vec_Vec3i new_obs;
    new_obs.push_back(Vec3i(136, 16, 0));

    std::vector<Primitive> prs = planner_->updateBlockedNodes(new_obs);
    planning_ros_msgs::Primitives changed_prs_msg = toPrimitivesROSMsg(prs);
    changed_prs_msg.header = header;
    changed_prs_pub.publish(changed_prs_msg);

    Vec3f pt = map_util->intToFloat(new_obs.back());

    sensor_msgs::PointCloud block_cloud;
    block_cloud.header = header;
    geometry_msgs::Point32 pt1;
    pt1.x = pt(0), pt1.y = pt(1), pt1.z = pt(2);
    block_cloud.points.push_back(pt1); 
    block_pub.publish(block_cloud);

    vec_Vec3f ends;
    for(auto it: prs) {
      Waypoint w = it.evaluate(it.t());
      ends.push_back(w.pos);
    }
    sensor_msgs::PointCloud end_cloud = vec_to_cloud(ends);
    end_cloud.header = header;
    end_pub.publish(end_cloud);
  }


  planner_->getSubStateSpace(1);

  planning_ros_msgs::Primitives sub_prs_msg = toPrimitivesROSMsg(planner_->getAllPrimitives());
  sub_prs_msg.header = header;
  sub_prs_pub.publish(sub_prs_msg);

  //Publish nodes in close set
  sensor_msgs::PointCloud sub_close_ps = vec_to_cloud(planner_->getCloseSet());
  sub_close_ps.header = header;
  sub_closeset_pub.publish(sub_close_ps);

  //Publish nodes in open set
  sensor_msgs::PointCloud sub_open_ps = vec_to_cloud(planner_->getOpenSet());
  sub_open_ps.header = header;
  sub_openset_pub.publish(sub_open_ps);

  //Publish nodes in null set
  sensor_msgs::PointCloud sub_null_ps = vec_to_cloud(planner_->getNullSet());
  sub_null_ps.header = header;
  sub_nullset_pub.publish(sub_null_ps);



  ros::spin();

  return 0;
}
