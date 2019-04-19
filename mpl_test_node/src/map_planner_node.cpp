#include <mpl_planner/planner/map_planner.h>
#include <mpl_traj_solver/traj_solver.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

#include "bag_reader.hpp"

void setMap(std::shared_ptr<MPL::VoxelMapUtil> &map_util,
            const planning_ros_msgs::VoxelMap &msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

void getMap(std::shared_ptr<MPL::VoxelMapUtil> &map_util,
            planning_ros_msgs::VoxelMap &map) {
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher sg_pub =
      nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher prs_pub =
      nh.advertise<planning_ros_msgs::PrimitiveArray>("primitives", 1, true);
  ros::Publisher traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher refined_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>(
      "trajectory_refined", 1, true);

  // Standard header
  std_msgs::Header header;
  header.frame_id = std::string("map");

  // Read map from bag file
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  planning_ros_msgs::VoxelMap map =
      read_bag<planning_ros_msgs::VoxelMap>(file_name, topic_name, 0).back();

  // Initialize map util
  std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
  setMap(map_util, map);

  // Free unknown space and dilate obstacles
  map_util->freeUnknown();
  // Inflate obstacle using robot radius (>0)
  double robot_r = 0.0;
  if (robot_r > 0) {
    vec_Vec3i ns;
    int rn = std::ceil(robot_r / map_util->getRes());
    for (int nx = -rn; nx <= rn; nx++) {
      for (int ny = -rn; ny <= rn; ny++) {
        if (nx == 0 && ny == 0) continue;
        if (std::hypot(nx, ny) > rn) continue;
        ns.push_back(Vec3i(nx, ny, 0));
      }
    }
    map_util->dilate(ns);
  }

  // Publish the dilated map for visualization
  getMap(map_util, map);
  map.header = header;
  map_pub.publish(map);

  // Initialize planner
  double dt, v_max, a_max, yaw_max;
  double u, u_yaw;
  int num;
  bool use_3d, use_yaw;
  nh.param("dt", dt, 1.0);
  nh.param("v_max", v_max, 2.0);
  nh.param("a_max", a_max, 1.0);
  nh.param("yaw_max", yaw_max, -1.0);
  nh.param("u", u, 1.0);
  nh.param("u_yaw", u_yaw, 0.3);
  nh.param("num", num, 1);
  nh.param("use_3d", use_3d, false);
  nh.param("use_yaw", use_yaw, false);

  // Set control input
  vec_E<VecDf> U;  // Control input
  const decimal_t du = u / num;
  if (use_3d && !use_yaw) {
    // consider primitive in z-axis, without yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dz = -u; dz <= u; dz += du)
          U.push_back(Vec3f(dx, dy, dz));
  } else if (!use_3d && !use_yaw) {
    // consider 2D primitive, without yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du) U.push_back(Vec3f(dx, dy, 0));
  } else if (!use_3d && use_yaw) {
    // consider 2D primitive, with yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dyaw = -u_yaw; dyaw <= u_yaw; dyaw += u_yaw) {
          Vec4f vec;
          vec << dx, dy, 0, dyaw;
          U.push_back(vec);
        }
  } else if (use_3d && use_yaw) {
    // consider primitive in z-axis, with yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dz = -u; dz <= u; dz += du)
          for (decimal_t dyaw = -u_yaw; dyaw <= u_yaw; dyaw += u_yaw) {
            Vec4f vec;
            vec << dx, dy, dz, dyaw;
            U.push_back(vec);
          }
  }

  // Set start and goal
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

  Waypoint3D start;
  start.pos = Vec3f(start_x, start_y, start_z);
  start.vel = Vec3f(start_vx, start_vy, start_vz);
  start.acc = Vec3f(0, 0, 0);
  start.jrk = Vec3f(0, 0, 0);
  start.yaw = 0;
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false;
  start.use_jrk = false;
  start.use_yaw = use_yaw;  // if true, yaw is also propogated

  Waypoint3D goal(start.control);  // initialized with the same control as start
  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.jrk = Vec3f(0, 0, 0);

  std::unique_ptr<MPL::VoxelMapPlanner> planner_ptr;

  planner_ptr.reset(new MPL::VoxelMapPlanner(true));
  planner_ptr->setMapUtil(map_util);  // Set collision checking function
  planner_ptr->setVmax(v_max);        // Set max velocity
  planner_ptr->setAmax(a_max);        // Set max acceleration (as control input)
  planner_ptr->setYawmax(yaw_max);    // Set yaw threshold
  planner_ptr->setDt(dt);             // Set dt for each primitive
  planner_ptr->setU(U);               // Set control input
  planner_ptr->setTol(0.5);           // Tolerance for goal region
  // planner_ptr->setHeurIgnoreDynamics(true);

  // Planning thread!
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

    auto traj = planner_ptr->getTraj();
    // Publish trajectory as primitives
    planning_ros_msgs::PrimitiveArray prs_msg =
        toPrimitiveArrayROSMsg(traj.getPrimitives());
    prs_msg.header = header;
    prs_pub.publish(prs_msg);

    // Publish trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    traj_pub.publish(traj_msg);

    printf(
        "Raw traj -- J(VEL): %f, J(ACC): %f, J(JRK): %f, J(SNP): %f, J(YAW): "
        "%f, total time: %f\n",
        traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::JRK),
        traj.J(Control::SNP), traj.Jyaw(), traj.getTotalTime());

    // Get intermediate waypoints
    auto waypoints = traj.getWaypoints();
    for (size_t i = 1; i < waypoints.size() - 1; i++)
      waypoints[i].control = Control::VEL;
    // Get time allocation
    auto dts = traj.getSegmentTimes();

    // Generate higher order polynomials
    TrajSolver3D traj_solver(Control::JRK);
    traj_solver.setWaypoints(waypoints);
    traj_solver.setDts(dts);
    traj = traj_solver.solve();

    // Publish refined trajectory
    planning_ros_msgs::Trajectory refined_traj_msg = toTrajectoryROSMsg(traj);
    refined_traj_msg.header = header;
    refined_traj_pub.publish(refined_traj_msg);

    printf(
        "Refined traj -- J(VEL): %f, J(ACC): %f, J(JRK): %f, J(SNP): %f, "
        "J(YAW): %f, total time: %f\n",
        traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::JRK),
        traj.J(Control::SNP), traj.Jyaw(), traj.getTotalTime());
  }

  // Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getCloseSet());
  // sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getValidRegion());
  ps.header = header;
  cloud_pub.publish(ps);

  // Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header = header;
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
  pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
  sg_pub.publish(sg_cloud);

  ros::spin();

  return 0;
}
