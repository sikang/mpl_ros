#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <ros/ros.h>

#include "bag_reader.hpp"
//#include <jps_planner/jps_planner/jps_planner.h>
//#include <jps_planner/distance_map_planner/distance_map_planner.h>
#include <mpl_planner/planner/map_planner.h>
#include <mpl_traj_solver/traj_solver.h>
#include <planning_ros_utils/primitive_ros_utils.h>

planning_ros_msgs::VoxelMap sliceMap(const planning_ros_msgs::VoxelMap &map,
                                     double h, double hh = 0) {
  // slice a 3D voxel map
  double res = map.resolution;
  int hhi = hh / res;
  int h_min = (h - map.origin.z) / res - hhi;
  h_min = h_min >= 0 ? h_min : 0;
  h_min = h_min < map.dim.z ? h_min : map.dim.z - 1;
  int h_max = (h - map.origin.z) / res + hhi + 1;
  h_max = h_max > 0 ? h_max : 1;
  h_max = h_max <= map.dim.z ? h_max : map.dim.z;

  // slice a 3D voxel map
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = map.origin.x;
  voxel_map.origin.y = map.origin.y;
  voxel_map.origin.z = h;
  voxel_map.dim.x = map.dim.x;
  voxel_map.dim.y = map.dim.y;
  voxel_map.dim.z = 1;
  voxel_map.resolution = map.resolution;

  voxel_map.data.resize(map.dim.x * map.dim.y, -1);

  for (int nx = 0; nx < map.dim.x; nx++) {
    for (int ny = 0; ny < map.dim.y; ny++) {
      for (int hi = h_min; hi < h_max; hi++) {
        int map_idx = nx + map.dim.x * ny + map.dim.x * map.dim.y * hi;
        int idx = nx + map.dim.x * ny;
        voxel_map.data[idx] = std::max(voxel_map.data[idx], map.data[map_idx]);
      }
    }
  }

  return voxel_map;
}

void setMap(std::shared_ptr<MPL::OccMapUtil> &map_util,
            const planning_ros_msgs::VoxelMap &msg) {
  Vec2f ori(msg.origin.x, msg.origin.y);
  Vec2i dim(msg.dim.x, msg.dim.y);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher region_pub =
      nh.advertise<sensor_msgs::PointCloud>("region", 1, true);
  ros::Publisher sg_pub = nh.advertise<sensor_msgs::PointCloud>("sg", 1, true);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher raw_traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("raw", 1, true);
  ros::Publisher perturb_traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("perturb", 1, true);
  ros::Publisher global_traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("global", 1, true);

  // Read map from bag file
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));

  double h;
  nh.param("height", h, 0.8);

  planning_ros_msgs::VoxelMap map =
      read_bag<planning_ros_msgs::VoxelMap>(file_name, topic_name, 0).back();
  map = sliceMap(map, h, 2.0);
  map.header.frame_id = "map";
  map_pub.publish(map);

  // Set start and goal
  double start_x, start_y;
  nh.param("start_x", start_x, 12.5);
  nh.param("start_y", start_y, 1.4);
  double start_vx, start_vy;
  nh.param("start_vx", start_vx, 0.0);
  nh.param("start_vy", start_vy, 0.0);
  double goal_x, goal_y;
  nh.param("goal_x", goal_x, 6.4);
  nh.param("goal_y", goal_y, 16.6);

  Waypoint2D start;
  start.pos = Vec2f(start_x, start_y);
  start.vel = Vec2f(start_vx, start_vy);
  start.acc = Vec2f(0, 0);
  start.jrk = Vec2f(0, 0);
  start.yaw = 0;
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false;
  start.use_jrk = false;
  start.use_yaw = false;

  Waypoint2D goal(start.control);
  goal.pos = Vec2f(goal_x, goal_y);
  goal.vel = Vec2f(0, 0);
  goal.acc = Vec2f(0, 0);
  goal.jrk = Vec2f(0, 0);

  std::shared_ptr<MPL::OccMapUtil> map_util =
      std::make_shared<MPL::OccMapUtil>();
  setMap(map_util, map);
  map_util->freeUnknown();

  // Initialize planner
  double dt, v_max, a_max;
  double u, u_yaw;
  int num, ndt;
  double potential_weight;
  nh.param("dt", dt, 1.0);
  nh.param("ndt", ndt, -1);
  nh.param("v_max", v_max, 2.0);
  nh.param("a_max", a_max, 1.0);
  nh.param("u", u, 1.0);
  nh.param("u_yaw", u_yaw, 0.5);
  nh.param("num", num, 1);
  nh.param("potential_weight", potential_weight, 1.0);

  vec_E<VecDf> U;
  const decimal_t du = u / num;
  for (decimal_t dx = -u; dx <= u; dx += du)
    for (decimal_t dy = -u; dy <= u; dy += du) U.push_back(Vec2f(dx, dy));

  std::unique_ptr<MPL::OccMapPlanner> planner_ptr;

  planner_ptr.reset(new MPL::OccMapPlanner(true));
  planner_ptr->setMapUtil(map_util);  // Set collision checking function
  planner_ptr->setVmax(v_max);        // Set max velocity
  planner_ptr->setAmax(a_max);        // Set max acceleration (as control input)
  planner_ptr->setEpsilon(1.0);       // Set dt for each primitive
  planner_ptr->setDt(dt);             // Set dt for each primitive
  planner_ptr->setU(U);               // 2D discretization with 1
  planner_ptr->setTol(0.2);           // Tolerance for goal region

  ros::Time t0 = ros::Time::now();
  bool valid = planner_ptr->plan(start, goal);
  if (!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             planner_ptr->getCloseSet().size());
  } else {
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             planner_ptr->getCloseSet().size());

    const auto traj = planner_ptr->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = "map";
    raw_traj_pub.publish(traj_msg);

    // Create a path from planned traj
    const auto ws = traj.getWaypoints();
    vec_Vec2f path;
    for (const auto &w : ws) path.push_back(w.pos);

    planner_ptr.reset(new MPL::OccMapPlanner(false));
    planner_ptr->setMapUtil(map_util);  // Set collision checking function
    planner_ptr->setVmax(v_max);        // Set max velocity
    planner_ptr->setAmax(a_max);   // Set max acceleration (as control input)
    planner_ptr->setEpsilon(1.0);  // Set dt for each primitive
    planner_ptr->setDt(dt);        // Set dt for each primitive
    planner_ptr->setU(U);          // 2D discretization with 1
    planner_ptr->setTol(0.5);      // Tolerance for goal region

    planner_ptr->setSearchRadius(Vec2f(0.5, 0.5));  // Set search region radius
    planner_ptr->setSearchRegion(path);  // Set search region around the path
    planner_ptr->setPotentialRadius(Vec2f(1.5, 1.5));  // Set potential distance
    planner_ptr->setPotentialWeight(10);               // Set potential weight
    planner_ptr->setGradientWeight(0);                 // Set gradient weight
    // planner_ptr->setPotentialMapRange(Vec2f(2.0, 2.0));
    planner_ptr->updatePotentialMap(start.pos);  // Update potential map

    planner_ptr->plan(start, goal);
    planning_ros_msgs::Trajectory perturb_traj_msg =
        toTrajectoryROSMsg(planner_ptr->getTraj(), 1);
    perturb_traj_msg.header.frame_id = "map";
    perturb_traj_pub.publish(perturb_traj_msg);

    auto region_msg = vec_to_cloud(planner_ptr->getSearchRegion(), -2);
    region_msg.header.frame_id = "map";
    region_pub.publish(region_msg);

    std::shared_ptr<MPL::OccMapUtil> global_map_util =
        std::make_shared<MPL::OccMapUtil>();
    setMap(global_map_util, map);
    global_map_util->freeUnknown();

    planner_ptr.reset(new MPL::OccMapPlanner(false));
    planner_ptr->setMapUtil(
        global_map_util);          // Set collision checking function
    planner_ptr->setVmax(v_max);   // Set max velocity
    planner_ptr->setAmax(a_max);   // Set max acceleration (as control input)
    planner_ptr->setEpsilon(1.0);  // Set dt for each primitive
    planner_ptr->setDt(dt);        // Set dt for each primitive
    planner_ptr->setU(U);          // 2D discretization with 1
    planner_ptr->setTol(0.5);      // Tolerance for goal region

    planner_ptr->setPotentialRadius(Vec2f(1.5, 1.5));  // Set potential distance
    planner_ptr->setPotentialWeight(10);               // Set potential weight
    planner_ptr->setGradientWeight(0);                 // Set gradient weight
    // planner_ptr->setPotentialMapRange(Vec2f(2.0, 2.0));
    planner_ptr->updatePotentialMap(start.pos);  // Update potential map

    planner_ptr->plan(start, goal);
    planning_ros_msgs::Trajectory global_traj_msg =
        toTrajectoryROSMsg(planner_ptr->getTraj());
    global_traj_msg.header.frame_id = "map";
    global_traj_pub.publish(global_traj_msg);
  }

  auto potential = planner_ptr->getPotentialCloud();
  for (auto &it : potential) it(2) -= 1;
  auto cloud_msg = vec_to_cloud(potential);
  cloud_msg.header.frame_id = "map";
  cloud_pub.publish(cloud_msg);

  vec_Vec2f sg;
  sg.push_back(start.pos);
  sg.push_back(goal.pos);
  auto sg_msg = vec_to_cloud(sg, 1);
  sg_msg.header.frame_id = "map";
  sg_pub.publish(sg_msg);

  ros::spin();

  return 0;
}

