#include "obstacle_config.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

ros::Publisher poly_pub;
ros::Publisher sg_pub;
ros::Publisher ps_pub;
ros::Publisher blocked_prs_pub;
ros::Publisher cleared_prs_pub;
std::vector<ros::Publisher> traj_pubs;

std::unique_ptr<ObstacleCourse<2>> obs_ptr;
std::unique_ptr<MPL::PolyMapPlanner2D> astar_ptr;
std::unique_ptr<MPL::PolyMapPlanner2D> lpastar_ptr;

Waypoint2D start, goal;

/// Simple test environment
struct Simple2DConfig0 : ObstacleCourse<2> {
  Simple2DConfig0() {
    Polyhedron2D rec1;
    rec1.add(Hyperplane2D(Vec2f(-0.5, 0), -Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(0.5, 0), Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(0, -0.5), -Vec2f::UnitY()));
    rec1.add(Hyperplane2D(Vec2f(0, 0.5), Vec2f::UnitY()));
    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        rec1, square(Vec2f(10, 0), Vec2f(0, 0.5), 1), 0));
  }

  void update() {

  }
};


void plan(std::unique_ptr<MPL::PolyMapPlanner2D>& planner_ptr, int id) {
  planner_ptr->setStaticObstacles(obs_ptr->static_obs); // Set static obstacles
  planner_ptr->setLinearObstacles(obs_ptr->linear_obs); // Set linear obstacles
  planner_ptr->setNonlinearObstacles(obs_ptr->nonlinear_obs); // Set nonlinear obstacles
  planner_ptr->updateNodes();

  if(id == 0)
    ROS_WARN("AStar:");
  else {
    ROS_WARN("LPAStar:");
    // Publish affected primitives
    auto blocked_prs_msg = toPrimitiveArrayROSMsg(planner_ptr->getBlockedPrimitives());
    blocked_prs_msg.header.frame_id = "map";
    blocked_prs_pub.publish(blocked_prs_msg);

    auto cleared_prs_msg = toPrimitiveArrayROSMsg(planner_ptr->getClearedPrimitives());
    cleared_prs_msg.header.frame_id = "map";
    cleared_prs_pub.publish(cleared_prs_msg);
  }

  ros::Time t0 = ros::Time::now();
  bool valid = planner_ptr->plan(start, goal);
  double plan_dt = (ros::Time::now() - t0).toSec();
  ROS_INFO(
    "Succeed! Takes %f sec, openset: [%zu], "
      "closeset (expanded): [%zu](%zu), total: [%zu]",
      plan_dt, planner_ptr->getOpenSet().size(),
      planner_ptr->getCloseSet().size(), planner_ptr->getExpandedNodes().size(),
      planner_ptr->getOpenSet().size() + planner_ptr->getCloseSet().size());

  // Publish trajectory
  auto traj = planner_ptr->getTraj();
  planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
  traj_msg.header.frame_id = "map";
  traj_pubs[id].publish(traj_msg);
}

void replanCallback(const std_msgs::Bool::ConstPtr &msg) {
  // Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header.frame_id = "map";
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start.pos(0), pt1.y = start.pos(1), pt1.z = 0;
  pt2.x = goal.pos(0), pt2.y = goal.pos(1), pt2.z = 0;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
  sg_pub.publish(sg_cloud);

  plan(astar_ptr, 0);
  plan(lpastar_ptr, 1);

  vec_E<Polyhedron2D> poly_obs = astar_ptr->getPolyhedrons(0);
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_obs);
  poly_msg.header.frame_id = "map";
  poly_pub.publish(poly_msg);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Subscriber replan_sub = nh.subscribe("replan", 1, replanCallback);
  ros::Publisher bound_pub =
      nh.advertise<decomp_ros_msgs::PolyhedronArray>("bound", 1, true);

  poly_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons", 1, true);

  sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);

  ros::Publisher traj0_pub = nh.advertise<planning_ros_msgs::Trajectory>("astar", 1, true);
  ros::Publisher traj1_pub = nh.advertise<planning_ros_msgs::Trajectory>("lpastar", 1, true);
  traj_pubs.push_back(traj0_pub);
  traj_pubs.push_back(traj1_pub);

  blocked_prs_pub =
    nh.advertise<planning_ros_msgs::PrimitiveArray>("blocked_primitives", 1, true);
  cleared_prs_pub =
    nh.advertise<planning_ros_msgs::PrimitiveArray>("cleared_primitives", 1, true);

  ps_pub = nh.advertise<sensor_msgs::PointCloud>("ps", 1, true);

  obs_ptr.reset(new Simple2DConfig0());

  Vec2f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), -2.5);
  nh.param("range_x", dim(0), 20.0);
  nh.param("range_y", dim(1), 5.0);

  // Initialize planner
  double dt, v_max, a_max;
  double u;
  int num;
  nh.param("dt", dt, 1.0);
  nh.param("v_max", v_max, -1.0);
  nh.param("a_max", a_max, -1.0);
  nh.param("u", u, 1.0);
  nh.param("num", num, 1);

  // Set input control
  vec_E<VecDf> U;
  const decimal_t du = u / num;
  for (decimal_t dx = -u; dx <= u; dx += du)
    for (decimal_t dy = -u; dy <= u; dy += du)
      U.push_back(Vec2f(dx, dy));


  astar_ptr.reset(new MPL::PolyMapPlanner2D(true));
  astar_ptr->setMap(origin, dim);         // Set collision checking function
  astar_ptr->setStaticObstacles(obs_ptr->static_obs); // Set static obstacles
  astar_ptr->setLinearObstacles(obs_ptr->linear_obs); // Set linear obstacles
  astar_ptr->setNonlinearObstacles(obs_ptr->nonlinear_obs); // Set nonlinear obstacles
  astar_ptr->setVmax(v_max);      // Set max velocity
  astar_ptr->setAmax(a_max);      // Set max acceleration
  astar_ptr->setDt(dt);           // Set dt for each primitive
  astar_ptr->setU(U); // Set discretization with 1 and efforts
  astar_ptr->setTol(0.5, 0.1); // Tolerance for goal region
  astar_ptr->setLPAstar(false);  // Use LPAstar

  lpastar_ptr.reset(new MPL::PolyMapPlanner2D(true));
  lpastar_ptr->setMap(origin, dim);         // Set collision checking function
  lpastar_ptr->setStaticObstacles(obs_ptr->static_obs); // Set static obstacles
  lpastar_ptr->setLinearObstacles(obs_ptr->linear_obs); // Set linear obstacles
  lpastar_ptr->setNonlinearObstacles(obs_ptr->nonlinear_obs); // Set nonlinear obstacles
  lpastar_ptr->setVmax(v_max);      // Set max velocity
  lpastar_ptr->setAmax(a_max);      // Set max acceleration
  lpastar_ptr->setDt(dt);           // Set dt for each primitive
  lpastar_ptr->setU(U); // Set discretization with 1 and efforts
  lpastar_ptr->setTol(0.5, 0.1); // Tolerance for goal region
  lpastar_ptr->setLPAstar(true);  // Use LPAstar

  // Set start and goal
  double start_x, start_y;
  nh.param("start_x", start_x, 0.5);
  nh.param("start_y", start_y, 0.0);
  double start_vx, start_vy;
  nh.param("start_vx", start_vx, 0.0);
  nh.param("start_vy", start_vy, 0.0);
  double goal_x, goal_y;
  nh.param("goal_x", goal_x, 10.0);
  nh.param("goal_y", goal_y, 0.0);

  bool use_acc, use_jrk;
  nh.param("use_acc", use_acc, false);
  nh.param("use_jrk", use_jrk, false);

  start.pos = Vec2f(start_x, start_y);
  start.vel = Vec2f(start_vx, start_vy);
  start.acc = Vec2f::Zero();
  start.jrk = Vec2f::Zero();
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = use_acc;
  start.use_jrk = use_jrk;
  start.use_yaw = false;

  goal.control = start.control;
  goal.pos = Vec2f(goal_x, goal_y);
  goal.vel = Vec2f::Zero();
  goal.acc = Vec2f::Zero();
  goal.jrk = Vec2f::Zero();

  // Publish bounding box
  vec_E<Polyhedron2D> bbox;
  bbox.push_back(astar_ptr->getBoundingBox());
  decomp_ros_msgs::PolyhedronArray bbox_msg = DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bound_pub.publish(bbox_msg);

  // Planning thread!
  std_msgs::Bool init;
  replanCallback(boost::make_shared<std_msgs::Bool>(init));

  ros::spin();

  return 0;
}
