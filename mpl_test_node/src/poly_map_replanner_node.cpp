#include "obstacle_config.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Float32.h>

/// Simple test environment
struct Simple2DConfig0 : ObstacleCourse<2> {
  Simple2DConfig0() {
    Polyhedron2D rec1;
    rec1.add(Hyperplane2D(Vec2f(-1, 0), -Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(1, 0), Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(0, -2), -Vec2f::UnitY()));
    rec1.add(Hyperplane2D(Vec2f(0, 2), Vec2f::UnitY()));

    circular_obs.push_back(PolyhedronCircularObstacle2D(
        rec1, Vec2f(10, 0), 1, 1));
   // nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
     //   rec1, square(Vec2f(10, 1), Vec2f(0, -0.5), 4, false), 0));

    update(0);
  }

  void update(decimal_t t) {
    linear_obs.clear();
    for(const auto& it: nonlinear_obs)
      linear_obs.push_back(it.get_linear_obstacle(t));
    for(const auto& it: circular_obs)
      linear_obs.push_back(it.get_linear_obstacle(t));
    for(auto& it: linear_obs)
      it.set_cov_v(0.5);
  }

  vec_E<Polyhedron2D> getPolyhedrons() {
    vec_E<Polyhedron2D> polys;
    for(const auto& it: linear_obs)
      polys.push_back(it.poly(1));
    return polys;
  }

  vec_E<PolyhedronCircularObstacle2D> circular_obs;
};


ros::Publisher poly_pub;
ros::Publisher sg_pub;
ros::Publisher ps_pub;
ros::Publisher blocked_prs_pub;
ros::Publisher cleared_prs_pub;
ros::Publisher prs_pub;
std::vector<ros::Publisher> traj_pubs;

std::unique_ptr<Simple2DConfig0> obs_ptr;
std::unique_ptr<MPL::PolyMapPlanner2D> astar_ptr;
std::unique_ptr<MPL::PolyMapPlanner2D> lpastar_ptr;

Waypoint2D start, goal;

std::vector<std_msgs::Float32> astar_runtime_msgs, lpastar_runtime_msgs;
std::vector<std_msgs::Float32> astar_value_msgs, lpastar_value_msgs;

bool subtree_;

void plan(std::unique_ptr<MPL::PolyMapPlanner2D>& planner_ptr, int id) {
  planner_ptr->setStaticObstacles(obs_ptr->static_obs); // Set static obstacles
  planner_ptr->setLinearObstacles(obs_ptr->linear_obs); // Set linear obstacles
  //planner_ptr->setNonlinearObstacles(obs_ptr->nonlinear_obs); // Set nonlinear obstacles
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

  if(id == 0) {
    std_msgs::Float32 runtime;
    runtime.data = plan_dt;
    astar_runtime_msgs.push_back(runtime);

    std_msgs::Float32 traj_cost;
    traj_cost.data = planner_ptr->getTrajCost();
    astar_value_msgs.push_back(traj_cost);
  }
  else {
    std_msgs::Float32 runtime;
    runtime.data = plan_dt;
    lpastar_runtime_msgs.push_back(runtime);

    std_msgs::Float32 traj_cost;
    traj_cost.data = planner_ptr->getTrajCost();
    lpastar_value_msgs.push_back(traj_cost);
  }

}

void replanCallback(const std_msgs::Float32::ConstPtr &msg) {
  static bool terminated = false;
  static decimal_t plan_time = 0;
  static decimal_t prev_time = -1000;
  obs_ptr->update(plan_time);

  if(plan_time - prev_time > 0.2 && !terminated) {
    plan(astar_ptr, 0);
    plan(lpastar_ptr, 1);
    prev_time = plan_time;
    // Publish location of start and goal
    sensor_msgs::PointCloud sg_cloud;
    sg_cloud.header.frame_id = "map";
    geometry_msgs::Point32 pt1, pt2;
    pt1.x = start.pos(0), pt1.y = start.pos(1), pt1.z = 0;
    pt2.x = goal.pos(0), pt2.y = goal.pos(1), pt2.z = 0;
    sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
    sg_pub.publish(sg_cloud);

    if(subtree_) {
      auto ws = lpastar_ptr->getTraj().getWaypoints();
      if (ws.size() <= 2)
        terminated = true;
      else {
        start = ws[1];
        start.enable_t = true;
        start.t = 0;
        if (lpastar_ptr->initialized())
          lpastar_ptr->getSubStateSpace(1);
        // Publish primitives
        planning_ros_msgs::PrimitiveArray prs_msg =
          toPrimitiveArrayROSMsg(lpastar_ptr->getAllPrimitives());
        prs_msg.header.frame_id = "map";
        prs_pub.publish(prs_msg);
      }
    }

  }

  vec_E<Polyhedron2D> poly_obs = obs_ptr->getPolyhedrons();
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_obs);
  poly_msg.header.frame_id = "map";
  poly_pub.publish(poly_msg);

  plan_time += msg->data;

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

  prs_pub =
    nh.advertise<planning_ros_msgs::PrimitiveArray>("graph", 1, true);
  blocked_prs_pub =
    nh.advertise<planning_ros_msgs::PrimitiveArray>("blocked_primitives", 1, true);
  cleared_prs_pub =
    nh.advertise<planning_ros_msgs::PrimitiveArray>("cleared_primitives", 1, true);

  ps_pub = nh.advertise<sensor_msgs::PointCloud>("ps", 1, true);

  obs_ptr.reset(new Simple2DConfig0());

  nh.param("subtree", subtree_, false);
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
  //astar_ptr->setNonlinearObstacles(obs_ptr->nonlinear_obs); // Set nonlinear obstacles
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
  //lpastar_ptr->setNonlinearObstacles(obs_ptr->nonlinear_obs); // Set nonlinear obstacles
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
  start.enable_t = true;

  goal.control = start.control;
  goal.pos = Vec2f(goal_x, goal_y);
  goal.vel = Vec2f::Zero();
  goal.acc = Vec2f::Zero();
  goal.jrk = Vec2f::Zero();
  goal.enable_t = true;

  // Publish bounding box
  vec_E<Polyhedron2D> bbox;
  bbox.push_back(astar_ptr->getBoundingBox());
  decomp_ros_msgs::PolyhedronArray bbox_msg = DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bound_pub.publish(bbox_msg);


  bool show_animation = true;
  nh.param("show_animation", show_animation, true);
  if(!show_animation) {
    // Manual thread!
    std_msgs::Float32 init;
    init.data = 1.0;
    replanCallback(boost::make_shared<std_msgs::Float32>(init));

    ros::spin();
  }
  else {
    // Show animation
    double hz = 10;
    ros::Rate loop_rate(hz);

    double t = 0;
    while (ros::ok()) {
      //if(t < obs_ptr->nonlinear_obs.front().traj().getTotalTime()) {
      if(t < 5) {
        std_msgs::Float32 plan;
        plan.data = 0.02;
        replanCallback(boost::make_shared<std_msgs::Float32>(plan));
        t += plan.data;
      }
      else
        break;

      ros::spinOnce();

      loop_rate.sleep();
    }

    std::string file_name;
    nh.param("file", file_name, std::string("sim.bag"));

    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Write);

    auto t0 = ros::Time::now();
    for(size_t i = 0; i < astar_runtime_msgs.size(); i++)
      bag.write("astar_run_time", t0 + ros::Duration(i * 1.0), astar_runtime_msgs[i]);
    for(size_t i = 0; i < lpastar_runtime_msgs.size(); i++)
      bag.write("lpastar_run_time", t0 + ros::Duration(i * 1.0), lpastar_runtime_msgs[i]);
    for(size_t i = 0; i < astar_value_msgs.size(); i++)
      bag.write("astar_traj_cost", t0 + ros::Duration(i * 1.0), astar_value_msgs[i]);
    for(size_t i = 0; i < lpastar_value_msgs.size(); i++)
      bag.write("lpastar_traj_cost", t0 + ros::Duration(i * 1.0) , lpastar_value_msgs[i]);


    bag.close();

  }


  return 0;
}
