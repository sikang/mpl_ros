#include "bag_reader.hpp"
//#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_planner/planner/map_planner.h>
#include <mpl_traj_solver/poly_solver.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

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
  // map_util->dilate(0.2, 0.1);
  // map_util->dilating();

  // Publish the dilated map for visualization
  getMap(map_util, map);
  map.header = header;
  map_pub.publish(map);

  // Initialize planner
  double dt, v_max, a_max, yaw_max;
  double u, u_yaw;
  int num, ndt;
  bool use_3d, use_yaw;
  nh.param("dt", dt, 1.0);
  nh.param("ndt", ndt, -1);
  nh.param("v_max", v_max, 2.0);
  nh.param("a_max", a_max, 1.0);
  nh.param("yaw_max", yaw_max, -1.0);
  nh.param("u", u, 1.0);
  nh.param("u_yaw", u_yaw, 0.3);
  nh.param("num", num, 1);
  nh.param("use_3d", use_3d, false);
  nh.param("use_yaw", use_yaw, false);

  vec_E<VecDf> U; // Control input
  const decimal_t du = u / num;
  if (use_3d && !use_yaw) {
    // consider primitive in z-axis, without yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dz = -u; dz <= u; dz += du)
          U.push_back(Vec3f(dx, dy, dz));
  } else if(!use_3d && !use_yaw) {
    // consider 2D primitive, without yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        U.push_back(Vec3f(dx, dy, 0));
  } else if(!use_3d && use_yaw) {
    // consider 2D primitive, with yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dyaw = -u_yaw; dyaw <= u_yaw; dyaw += u_yaw) {
          Vec4f vec;
          vec << dx, dy, 0, dyaw;
          U.push_back(vec);
        }
  } else if(use_3d && use_yaw) {
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
  start.use_yaw = use_yaw; // if true, yaw is also propogated

  Waypoint3D goal(start.control); // initialized with the same control as start
  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.jrk = Vec3f(0, 0, 0);

  std::unique_ptr<MPL::VoxelMapPlanner> planner_ptr;

  planner_ptr.reset(new MPL::VoxelMapPlanner(true));
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->setVmax(v_max);       // Set max velocity
  planner_ptr->setAmax(a_max);       // Set max acceleration (as control input)
  planner_ptr->setYawmax(yaw_max);       // Set yaw threshold
  planner_ptr->setDt(dt);            // Set dt for each primitive
  planner_ptr->setTmax(ndt * dt);    // Set the planning horizon: n*dt
  planner_ptr->setU(U); // Set control input
  planner_ptr->setTol(0.2); // Tolerance for goal region
  planner_ptr->setHeurIgnoreDynamics(true);

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

    const auto traj = planner_ptr->getTraj();
    // Publish trajectory as primitives
    planning_ros_msgs::PrimitiveArray prs_msg =
      toPrimitiveArrayROSMsg(traj.getPrimitives());
    prs_msg.header = header;
    prs_pub.publish(prs_msg);

    // Publish trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    traj_pub.publish(traj_msg);

    printf("==================  Raw traj -- total J in ACC: %f, total time: %f\n",
           traj.J(Control::ACC), traj.getTotalTime());

    // Get intermediate waypoints
    const auto waypoints = traj.getWaypoints();
    // Get time allocation
    const auto dts = traj.getSegmentTimes();

    // Generate higher order polynomials
    PolySolver3D poly_solver(2, 3);
    poly_solver.solve(waypoints, dts);

    auto traj_refined = Trajectory3D(poly_solver.getTrajectory()->toPrimitives());

    // Publish refined trajectory
    planning_ros_msgs::Trajectory refined_traj_msg =
      toTrajectoryROSMsg(traj_refined);
    refined_traj_msg.header = header;
    refined_traj_pub.publish(refined_traj_msg);

    printf("================ Refined traj -- total J in ACC: %f, total time: %f\n",
           traj_refined.J(Control::ACC), traj_refined.getTotalTime());
  }

  // Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getCloseSet());
  //sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getValidRegion());
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


  // Publish primitives
  /*
  planning_ros_msgs::PrimitiveArray prs_msg =
      toPrimitiveArrayROSMsg(planner_ptr->getPrimitivesToGoal());
  prs_msg.header = header;
  prs_pub.publish(prs_msg);
  */


  ros::spin();

  return 0;
}
