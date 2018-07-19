#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_external_planner/poly_map_planner/poly_map_planner.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

using namespace MPL;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher es_pub =
      nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoids", 1, true);
  ros::Publisher poly_pub =
      nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons", 1, true);
  ros::Publisher sg_pub =
      nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher ps_pub = nh.advertise<sensor_msgs::PointCloud>("ps", 1, true);

  vec_E<Ellipsoid2D> elli_obs;
  vec_E<Polyhedron2D> poly_obs;

  Polyhedron2D rec1;
  rec1.add(Hyperplane2D(Vec2f(5.5, 2.5), -Vec2f::UnitX()));
  rec1.add(Hyperplane2D(Vec2f(6.5, 2.5), Vec2f::UnitX()));
  rec1.add(Hyperplane2D(Vec2f(6, 1.0), -Vec2f::UnitY()));
  rec1.add(Hyperplane2D(Vec2f(6, 4.0), Vec2f::UnitY()));
  poly_obs.push_back(rec1);

  Vec2f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), 0.0);
  nh.param("range_x", dim(0), 20.0);
  nh.param("range_y", dim(1), 5.0);

  // Initialize planner
  double dt, v_max, a_max, w, epsilon, t_max;
  double u_max;
  int num;
  nh.param("dt", dt, 1.0);
  nh.param("epsilon", epsilon, 1.0);
  nh.param("v_max", v_max, -1.0);
  nh.param("a_max", a_max, -1.0);
  nh.param("u_max", u_max, 1.0);
  nh.param("t_max", t_max, -1.0);
  nh.param("w", w, 10.);
  nh.param("num", num, 1);

  std::unique_ptr<MPL::PolyMapPlanner2D> planner_;
  planner_.reset(new MPL::PolyMapPlanner2D(false));
  planner_->setMap(origin, dim, poly_obs);         // Set collision checking function
  planner_->setEpsilon(epsilon); // Set greedy param (default equal to 1)
  planner_->setVmax(v_max);      // Set max velocity
  planner_->setAmax(a_max);      // Set max acceleration
  planner_->setTmax(t_max);      // Set max time
  planner_->setDt(dt);           // Set dt for each primitive
  planner_->setW(w);             // Set w for each primitive
  planner_->setTol(0.5); // Tolerance for goal region

  // Set start and goal
  double start_x, start_y;
  nh.param("start_x", start_x, 0.5);
  nh.param("start_y", start_y, 1.0);
  double start_vx, start_vy;
  nh.param("start_vx", start_vx, 0.0);
  nh.param("start_vy", start_vy, 0.0);
  double goal_x, goal_y;
  nh.param("goal_x", goal_x, 6.4);
  nh.param("goal_y", goal_y, 16.6);

  bool use_acc, use_jrk;
  nh.param("use_acc", use_acc, true);
  nh.param("use_jrk", use_jrk, false);

  Waypoint2D start;
  start.pos = Vec2f(start_x, start_y);
  start.vel = Vec2f(start_vx, start_vy);
  start.acc = Vec2f::Zero();
  start.jrk = Vec2f::Zero();
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = use_acc;
  start.use_jrk = use_jrk;
  start.use_yaw = false;

  Waypoint2D goal(start.control);
  goal.pos = Vec2f(goal_x, goal_y);
  goal.vel = Vec2f::Zero();
  goal.acc = Vec2f::Zero();
  goal.jrk = Vec2f::Zero();

  // Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header.frame_id = "map";
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start_x, pt1.y = start_y, pt1.z = 0;
  pt2.x = goal_x, pt2.y = goal_y, pt2.z = 0;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
  sg_pub.publish(sg_cloud);

  // Set input control
  vec_E<VecDf> U;
  const decimal_t du = u_max / num;
  for (decimal_t dx = -u_max; dx <= u_max; dx += du)
    for (decimal_t dy = -u_max; dy <= u_max; dy += du)
      U.push_back(Vec3f(dx, dy, 0));
  planner_->setU(U); // Set discretization with 1 and efforts

  auto t0 = ros::Time::now();
  bool valid = planner_->plan(start, goal);

   if (!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());
  } else {
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());

    // Publish trajectory
    auto traj = planner_->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = "map";
    traj_pub.publish(traj_msg);

    printf("================== Traj -- total J(VEL): %f, J(ACC): %F, J(JRK): %f, "
           "total time: %f\n",
           traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::SNP), traj.getTotalTime());
  }

  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(elli_obs);
  es_msg.header.frame_id = "map";
  es_pub.publish(es_msg);

  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_obs);
  poly_msg.header.frame_id = "map";
  poly_pub.publish(poly_msg);

  ros::spin();

  return 0;
}
