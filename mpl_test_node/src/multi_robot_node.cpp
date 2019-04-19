/**
 * @brief Multi-robot test node in a tunnel case
 */
#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

#include "bag_writter.hpp"
#include "robot_team.hpp"

int main(int argc, char** argv) {
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

  Vec2f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), -5.0);
  nh.param("range_x", dim(0), 10.0);
  nh.param("range_y", dim(1), 10.0);

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
    for (decimal_t dy = -u; dy <= u; dy += du) U.push_back(Vec2f(dx, dy));

  bool use_config1, use_config2;
  nh.param("use_config1", use_config1, true);
  nh.param("use_config2", use_config2, false);

  // Set robot geomtric shape
  Polyhedron2D rec;
  rec.add(Hyperplane2D(Vec2f(-0.5, 0), -Vec2f::UnitX()));
  rec.add(Hyperplane2D(Vec2f(0.5, 0), Vec2f::UnitX()));
  rec.add(Hyperplane2D(Vec2f(0, -0.5), -Vec2f::UnitY()));
  rec.add(Hyperplane2D(Vec2f(0, 0.5), Vec2f::UnitY()));

  std::unique_ptr<HomogeneousRobotTeam<2>> robot_team;
  if (use_config1)
    robot_team.reset(new Team1(0.01));
  else if (use_config2)
    robot_team.reset(new Team2(0.01));
  else
    robot_team.reset(new HomogeneousRobotTeam<2>());
  robot_team->set_v_max(v_max);
  robot_team->set_a_max(a_max);
  robot_team->set_u(U);
  robot_team->set_dt(dt);
  robot_team->set_map(origin, dim);
  robot_team->set_geometry(rec);

  robot_team->init();

  vec_E<Polyhedron2D> bbox;
  bbox.push_back(robot_team->get_robots().front()->get_bbox());
  decomp_ros_msgs::PolyhedronArray bbox_msg =
      DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bbox_msg.header.stamp = ros::Time::now();
  bound_pub.publish(bbox_msg);

  // Start the replan loop
  ros::Rate loop_rate(100);
  decimal_t update_t = 0.01;
  decimal_t time = 0;
  decimal_t prev_time = -1000;
  ros::Time t0 = ros::Time::now();

  std::vector<sensor_msgs::PointCloud> state_msgs;
  std::vector<sensor_msgs::PointCloud> start_msgs;
  std::vector<decomp_ros_msgs::PolyhedronArray> poly_msgs;
  std::vector<planning_ros_msgs::PathArray> path_msgs;
  std::vector<planning_ros_msgs::PrimitiveArray> prs_msgs;
  while (ros::ok()) {
    time += update_t;

    // plan
    // if(!robot_team->update_centralized(time)) {
    if (!robot_team->update_decentralized(time)) {
      ROS_INFO("Robot fails to plan, ABORT!");
      break;
    }

    // set obstacle simultaneously
    auto poly_obs = robot_team->get_obs();  // set obstacles at time

    // Visualizing current status at 10 Hz
    if (time - prev_time >= 0.1) {
      // Reduce the size of obstacles manually.
      for (auto& it : poly_obs) {
        for (auto& itt : it.vs_) itt.p_ -= itt.n_ * 0.25;
      }

      decomp_ros_msgs::PolyhedronArray poly_msg =
          DecompROS::polyhedron_array_to_ros(poly_obs);
      poly_msg.header.frame_id = "map";
      poly_msg.header.stamp = t0 + ros::Duration(time);
      poly_pub.publish(poly_msg);
      poly_msgs.push_back(poly_msg);

      vec_E<vec_Vec2f> path_array;
      vec_Vec2f states;
      vec_Vec2f starts;
      for (auto& it : robot_team->get_robots()) {
        starts.push_back(it->get_start().pos);
        states.push_back(it->get_state(time).pos);
        path_array.push_back(it->get_history());
      }

      auto path_msg = path_array_to_ros(path_array);
      path_msg.header.frame_id = "map";
      path_msg.header.stamp = t0 + ros::Duration(time);
      path_pub.publish(path_msg);
      path_msgs.push_back(path_msg);

      auto state_msg = vec_to_cloud(states);
      state_msg.header.frame_id = "map";
      state_msg.header.stamp = t0 + ros::Duration(time);
      state_pub.publish(state_msg);
      state_msgs.push_back(state_msg);

      auto start_msg = vec_to_cloud(starts);
      start_msg.header.frame_id = "map";
      start_msg.header.stamp = t0 + ros::Duration(time);
      start_pub.publish(start_msg);
      start_msgs.push_back(start_msg);

      vec_E<Primitive2D> prs_array;
      for (auto& it : robot_team->get_robots()) {
        auto prs = it->get_primitives();
        prs_array.insert(prs_array.end(), prs.begin(), prs.end());
      }

      auto prs_msg = toPrimitiveArrayROSMsg(prs_array);
      prs_msg.header.frame_id = "map";
      prs_msg.header.stamp = t0 + ros::Duration(time);
      prs_pub.publish(prs_msg);
      prs_msgs.push_back(prs_msg);

      prev_time = time;
    }

    if (robot_team->finished(time)) {
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

  for (const auto& it : start_msgs) bag.write(starts_name, it.header.stamp, it);
  for (const auto& it : state_msgs) bag.write(states_name, it.header.stamp, it);
  for (const auto& it : poly_msgs) bag.write(polys_name, it.header.stamp, it);
  for (const auto& it : path_msgs) bag.write(paths_name, it.header.stamp, it);
  for (const auto& it : prs_msgs) bag.write(prs_name, it.header.stamp, it);

  bag.close();

  return 0;
}
