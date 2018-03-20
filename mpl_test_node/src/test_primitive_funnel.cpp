#include "bag_reader.hpp"
#include <ros/ros.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <motion_primitive_library/planner/mp_funnel_util.h>
#include <decomp_ros_utils/data_ros_utils.h>


using namespace MPL;
std::unique_ptr<MPFunnelUtil> planner_;

//**** Sort poits in an order
vec_Vec3f sort_pts(const vec_Vec3f &pts) {
  //**** sort in body frame
  Vec3f avg = Vec3f::Zero();
  for (auto p : pts)
    avg += p;
  avg /= pts.size();

  vec_E<std::pair<decimal_t, Vec3f>> ps_valued;
  ps_valued.resize(pts.size());
  for (unsigned int i = 0; i < pts.size(); i++) {
    decimal_t theta = atan2(pts[i](1) - avg(1), pts[i](0) - avg(0));
    ps_valued[i] = std::make_pair(theta, pts[i]);
  }

  std::sort(ps_valued.begin(), ps_valued.end(), [](const std::pair<decimal_t, Vec3f>& p1, const std::pair<decimal_t, Vec3f>& p2) { return p1.first < p2.first; });
  vec_Vec3f b;
  for (const auto& it : ps_valued)
    b.push_back(it.second);
  return b;
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::Ellipsoids>("ellipsoids", 1, true);
  ros::Publisher sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher ps_pub = nh.advertise<sensor_msgs::PointCloud>("ps", 1, true);

  ros::Time t0 = ros::Time::now();
  //Read map from bag file
  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("voxel_map"));
  nh.param("topic", topic_name, std::string("voxel_map"));
  sensor_msgs::PointCloud map = read_bag<sensor_msgs::PointCloud>(file_name, topic_name, 0).back();
  cloud_pub.publish(map);

  Vec3f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), 0.0);
  nh.param("origin_z", origin(2), 0.0);
  nh.param("range_x", dim(0), 20.0);
  nh.param("range_y", dim(1), 20.0);
  nh.param("range_z", dim(2), 1.0);

  ROS_INFO("Takse %f sec to set up map!", (ros::Time::now() - t0).toSec());
  t0 = ros::Time::now();

  //Initialize planner
  double dt, v_max, a_max, w, epsilon, t_max;
  double u_max;
  int max_num, num;
  nh.param("dt", dt, 1.0);
  nh.param("epsilon", epsilon, 1.0);
  nh.param("v_max", v_max, -1.0);
  nh.param("a_max", a_max, -1.0);
  nh.param("u_max", u_max, 1.0);
  nh.param("t_max", t_max, -1.0);
  nh.param("w", w, 10.);
  nh.param("num", num, 1);
  nh.param("max_num", max_num, -1);

  double kp, kv, v;
  nh.param("kp", kp, 5.0);
  nh.param("kv", kv, 3.0);
  nh.param("v", v, 3.0);

  planner_.reset(new MPFunnelUtil(true));
  planner_->setMap(cloud_to_vec(map), origin, dim, kp, kv, v); // Set collision checking function
  planner_->setEpsilon(epsilon); // Set greedy param (default equal to 1)
  planner_->setVmax(v_max); // Set max velocity
  planner_->setAmax(a_max); // Set max acceleration 
  planner_->setUmax(u_max); // Set max control 
  planner_->setTmax(t_max); // Set max time
  planner_->setDt(dt); // Set dt for each primitive
  planner_->setW(w); // Set w for each primitive
  planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
  planner_->setTol(1.0, 1.0); // Tolerance for goal region


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
 
  Waypoint3 start;
  start.pos = Vec3f(start_x, start_y, start_z);
  start.vel = Vec3f(start_vx, start_vy, start_vz);
  start.acc = Vec3f(0, 0, 0);
  start.jrk = Vec3f(start_x, start_vx, 0);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false;
  start.use_jrk = false;

  Waypoint3 goal;
  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.jrk = Vec3f(0, 0, 0);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;


  //Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header.frame_id = "map";
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
  pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2); 
  sg_pub.publish(sg_cloud);

  //Set input control
  vec_Vec3f U;
  const decimal_t du = u_max / num;
   for(decimal_t dx = -u_max; dx <= u_max; dx += du ) 
      for(decimal_t dy = -u_max; dy <= u_max; dy += du )
        U.push_back(Vec3f(dx, dy, 0));
  planner_->setU(U);// Set discretization with 1 and efforts

  t0 = ros::Time::now();
  bool valid = planner_->plan(start, goal);

  //Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_->getCloseSet());
  ps.header.frame_id = "map";
  ps_pub.publish(ps);

  if(!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());
  }
  else {
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());

    //Publish trajectory
    auto traj = planner_->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = "map";
    traj_pub.publish(traj_msg);

    printf("================== Traj -- total J(1): %f, J(2): %F, J(3): %f, total time: %f\n", 
        traj.J(1), traj.J(2), traj.J(3), traj.getTotalTime());

    vec_Vec3f pts_x, pts_y;
    Waypoint3 s1 = start, s2 = start;
    Waypoint3 s3 = start, s4 = start;
    Waypoint3 s5 = start, s6 = start;

    Vec2f wf(start.pos(0), start.vel(0));


    vec_E<vec_Vec3f> bounds;
    vec_Ellipsoid Es;

    for(auto seg: traj.segs) {
      PrimitiveFunnel<3> f(seg, 5, 3);
      auto b1 = f.compute(s1, Vec2f(3, 0));
      auto b2 = f.compute(s2, Vec2f(-3, 0));
      s1  = b1.back();
      s2  = b2.back();
      for(auto it: b1)
        pts_x.push_back(it.pos);
      for(auto it: b2)
        pts_x.push_back(it.pos);


      auto b3 = f.compute(s3, Vec2f(3, 3), 5);
      auto b4 = f.compute(s4, Vec2f(-3, -3), 5);
      auto b5 = f.compute(s5, Vec2f(3, -3), 5);
      auto b6 = f.compute(s6, Vec2f(-3, 3), 5);
      s3  = b3.back();
      s4  = b4.back();
      s5  = b5.back();
      s6  = b6.back();
      for(int i = 0; i < b3.size(); i++) {
        pts_y.push_back(b3[i].pos);
        pts_y.push_back(b4[i].pos);
        pts_y.push_back(b5[i].pos);
        pts_y.push_back(b6[i].pos);
        vec_Vec3f b;
        b.push_back(b3[i].pos);
        b.push_back(b4[i].pos);
        b.push_back(b5[i].pos);
        b.push_back(b6[i].pos);
        b = sort_pts(b);
        b.push_back(b.front());

        bounds.push_back(b);
      }

      auto es = f.compute(wf, 3, 5);
      Es.insert(Es.end(), es.begin(), es.end());
      wf = f.get_wf();
    }

    decomp_ros_msgs::Ellipsoids es_msg = DecompROS::ellipsoids_to_ros(Es);
    es_msg.header.frame_id = "map";
    es_pub.publish(es_msg);
  }


  ros::spin();

  return 0;
}
