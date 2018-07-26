/**
 * @brief Multi-robot test node in a tunnel case
 */
#include "bag_writter.hpp"
#include "robot.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

using namespace MPL;

vec_E<Polyhedron2D> set_obs(vec_E<Robot<2>>& robots, decimal_t time,
                            const vec_E<PolyhedronObstacle2D>& external_static_obs) {
  vec_E<Polyhedron2D> poly_obs;
  for(size_t i = 0; i < robots.size(); i++) {
    vec_E<PolyhedronLinearObstacle2D> linear_obs;
    vec_E<PolyhedronNonlinearObstacle2D> nonlinear_obs;
    for(size_t j = 0; j < robots.size(); j++) {
      if(i != j)
        nonlinear_obs.push_back(robots[j].get_nonlinear_obstacle(time, 4));
        //linear_obs.push_back(robots[j].get_linear_obstacle(time));
    }
    robots[i].set_static_obs(external_static_obs);
    robots[i].set_linear_obs(linear_obs);
    robots[i].set_nonlinear_obs(nonlinear_obs);
    poly_obs.push_back(robots[i].get_nonlinear_obstacle(time).poly(0));
  }
  for(const auto& it: external_static_obs)
    poly_obs.push_back(it.poly(0));
  return poly_obs;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  std::string file_name;
  std::string states_name, starts_name, polys_name, paths_name, prs_name;
  nh.param("file", file_name, std::string("sim.bag"));
  nh.param("states_name", states_name, std::string("/states"));
  nh.param("starts_name", starts_name, std::string("/starts"));
  nh.param("polys_name", polys_name, std::string("/polyhedrons"));
  nh.param("paths_name", paths_name, std::string("/paths"));
  nh.param("prs_name", prs_name, std::string("/prs"));

  ros::Publisher poly_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>(polys_name, 1, true);
  ros::Publisher bound_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("bound", 1, true);
  ros::Publisher state_pub =
      nh.advertise<sensor_msgs::PointCloud>(states_name, 1, true);
  ros::Publisher start_pub =
      nh.advertise<sensor_msgs::PointCloud>(starts_name, 1, true);
  ros::Publisher prs_pub =
      nh.advertise<planning_ros_msgs::PrimitiveArray>(prs_name, 1, true);
  ros::Publisher path_pub =
      nh.advertise<planning_ros_msgs::PathArray>(paths_name, 1, true);

  Polyhedron2D rec;
  rec.add(Hyperplane2D(Vec2f(-0.5, 0), -Vec2f::UnitX()));
  rec.add(Hyperplane2D(Vec2f(0.5, 0), Vec2f::UnitX()));
  rec.add(Hyperplane2D(Vec2f(0, -0.5), -Vec2f::UnitY()));
  rec.add(Hyperplane2D(Vec2f(0, 0.5), Vec2f::UnitY()));

  Vec2f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), -5.0);
  nh.param("range_x", dim(0), 10.0);
  nh.param("range_y", dim(1), 10.0);

  // Initialize planner
  double dt, v_max, a_max, t_max;
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

  // Create a team of 5 robots
  Robot2D robot1(rec, "robot1", true);
  robot1.set_v_max(v_max);
  robot1.set_a_max(a_max);
  robot1.set_u(U);
  robot1.set_dt(dt);
  robot1.set_map(origin, dim);
  robot1.set_start(Vec2f(0, -5));
  robot1.set_goal(Vec2f(10, -5));
  robot1.plan(0); // need to set a small difference in the starting time

  Robot2D robot2(rec, "robot2", true);
  robot2.set_v_max(v_max);
  robot2.set_a_max(a_max);
  robot2.set_u(U);
  robot2.set_dt(dt);
  robot2.set_map(origin, dim);
  robot2.set_start(Vec2f(0, -2.5));
  robot2.set_goal(Vec2f(10, -2.5));
  robot2.plan(0.01); // need to set a small difference in the starting time

  Robot2D robot3(rec, "robot3", true);
  robot3.set_v_max(v_max);
  robot3.set_a_max(a_max);
  robot3.set_u(U);
  robot3.set_dt(dt);
  robot3.set_map(origin, dim);
  robot3.set_start(Vec2f(0, 0));
  robot3.set_goal(Vec2f(10, 0));
  robot3.plan(0.02); // need to set a small difference in the starting time

  Robot2D robot4(rec, "robot4", true);
  robot4.set_v_max(v_max);
  robot4.set_a_max(a_max);
  robot4.set_u(U);
  robot4.set_dt(dt);
  robot4.set_map(origin, dim);
  robot4.set_start(Vec2f(0, 2.5));
  robot4.set_goal(Vec2f(10, 2.5));
  robot4.plan(0.03);

  Robot2D robot5(rec, "robot5", true);
  robot5.set_v_max(v_max);
  robot5.set_a_max(a_max);
  robot5.set_u(U);
  robot5.set_dt(dt);
  robot5.set_map(origin, dim);
  robot5.set_start(Vec2f(0, 5));
  robot5.set_goal(Vec2f(10, 5));
  robot5.plan(0.04);

  Robot2D robot6(rec, "robot6", true);
  robot6.set_v_max(v_max);
  robot6.set_a_max(a_max);
  robot6.set_u(U);
  robot6.set_dt(dt);
  robot6.set_map(origin, dim);
  robot6.set_start(Vec2f(10, 0));
  robot6.set_goal(Vec2f(0, 0));
  robot6.plan(0.05);

  Robot2D robot7(rec, "robot7", true);
  robot7.set_v_max(v_max);
  robot7.set_a_max(a_max);
  robot7.set_u(U);
  robot7.set_dt(dt);
  robot7.set_map(origin, dim);
  robot7.set_start(Vec2f(10, 2.5));
  robot7.set_goal(Vec2f(0, 2.5));
  robot7.plan(0.06);

  Robot2D robot8(rec, "robot8", true);
  robot8.set_v_max(v_max);
  robot8.set_a_max(a_max);
  robot8.set_u(U);
  robot8.set_dt(dt);
  robot8.set_map(origin, dim);
  robot8.set_start(Vec2f(10, 5));
  robot8.set_goal(Vec2f(0, 5));
  robot8.plan(0.07);

  vec_E<Robot2D> robots;
  robots.push_back(robot1);
  robots.push_back(robot2);
  robots.push_back(robot3);
  robots.push_back(robot4);
  robots.push_back(robot5);
  //robots.push_back(robot6);
  //robots.push_back(robot7);
  //robots.push_back(robot8);

  // Build the obstacle course
  vec_E<PolyhedronObstacle2D> static_obs;

  Polyhedron2D rec2;
  rec2.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
  rec2.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
  rec2.add(Hyperplane2D(Vec2f(5, 0.2), -Vec2f::UnitY()));
  rec2.add(Hyperplane2D(Vec2f(5, 5.5), Vec2f::UnitY()));
  static_obs.push_back(PolyhedronObstacle2D(rec2, Vec2f::Zero()));

  Polyhedron2D rec3;
  rec3.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
  rec3.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
  rec3.add(Hyperplane2D(Vec2f(5, -5.5), -Vec2f::UnitY()));
  rec3.add(Hyperplane2D(Vec2f(5, -0.2), Vec2f::UnitY()));
  static_obs.push_back(PolyhedronObstacle2D(rec3, Vec2f::Zero()));

  vec_E<Polyhedron2D> bbox;
  bbox.push_back(robots.front().get_bbox());
  decomp_ros_msgs::PolyhedronArray bbox_msg = DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bbox_msg.header.stamp = ros::Time::now();
  bound_pub.publish(bbox_msg);

  // Start the replan loop
  ros::Rate loop_rate(100);
  decimal_t update_t = 0.01;
  decimal_t time = 0;
  ros::Time t0 = ros::Time::now();

  std::vector<sensor_msgs::PointCloud> state_msgs;
  std::vector<sensor_msgs::PointCloud> start_msgs;
  std::vector<decomp_ros_msgs::PolyhedronArray> poly_msgs;
  std::vector<planning_ros_msgs::PathArray> path_msgs;
  std::vector<planning_ros_msgs::PrimitiveArray> prs_msgs;
  while (ros::ok()) {
    time += update_t;

    // set obstacle simultaneously
    auto poly_obs = set_obs(robots, time, static_obs); // set obstacles at time

    // plan
    bool safe = true;
    for(auto& it: robots) {
      if(!it.plan(time)) {
        safe = false;
        break;
      }
    }

    if(!safe) {
      ROS_INFO("Robot fails to plan, ABORT!");
      break;
    }

    // Reduce the size of obstacles manually.
    for(auto& it: poly_obs) {
      for(auto& itt: it.vs_)
        itt.p_ -= itt.n_ * 0.25;
    }

    // Visualizing current status
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_obs);
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = t0 + ros::Duration(time);
    poly_pub.publish(poly_msg);
    poly_msgs.push_back(poly_msg);

    vec_E<vec_Vec3f> path_array;
    vec_Vec2f states;
    vec_Vec2f starts;
    for(auto& it: robots) {
      starts.push_back(it.get_start().pos);
      states.push_back(it.get_state(time).pos);
      path_array.push_back(vec2_to_vec3(it.get_history()));
    }

    auto path_msg = path_array_to_ros(path_array);
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = t0 + ros::Duration(time);
    path_pub.publish(path_msg);
    path_msgs.push_back(path_msg);

    auto state_msg = vec_to_cloud(vec2_to_vec3(states));
    state_msg.header.frame_id = "map";
    state_msg.header.stamp = t0 + ros::Duration(time);
    state_pub.publish(state_msg);
    state_msgs.push_back(state_msg);

    auto start_msg = vec_to_cloud(vec2_to_vec3(starts));
    start_msg.header.frame_id = "map";
    start_msg.header.stamp = t0 + ros::Duration(time);
    start_pub.publish(start_msg);
    start_msgs.push_back(start_msg);

    vec_E<Primitive2D> prs_array;
    for(auto& it: robots) {
      auto prs = it.get_primitives();
      prs_array.insert(prs_array.end(), prs.begin(), prs.end());
    }

    auto prs_msg = toPrimitiveArrayROSMsg(prs_array);
    prs_msg.header.frame_id = "map";
    prs_msg.header.stamp = t0 + ros::Duration(time);
    prs_pub.publish(prs_msg);
    prs_msgs.push_back(prs_msg);

    bool reached = true;
    for(const auto& it: robots)
      if(!it.reached(time))
        reached = false;

    if(reached) {
      ROS_INFO("All robots reached!");
      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  ROS_WARN("Total time: %f", (ros::Time::now() - t0).toSec());

  // Write to bag (optional)
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Write);

  for(const auto& it: start_msgs)
    bag.write(starts_name, it.header.stamp, it);
  for(const auto& it: state_msgs)
    bag.write(states_name, it.header.stamp, it);
  for(const auto& it: poly_msgs)
    bag.write(polys_name, it.header.stamp, it);
  for(const auto& it: path_msgs)
    bag.write(paths_name, it.header.stamp, it);
  for(const auto& it: prs_msgs)
    bag.write(prs_name, it.header.stamp, it);

  bag.close();

  return 0;
}
