#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_external_planner/poly_map_planner/poly_map_planner.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

using namespace MPL;

template <int Dim>
class Robot {
 public:
  Robot() {}

  ~Robot() {}

  Robot(const Polyhedron<Dim>& poly) : poly_(poly) {}

  void set_v_max(decimal_t v) { v_max_ = v; }

  void set_a_max(decimal_t a) { a_max_ = a; }

  void set_u(const vec_E<VecDf>& U) { U_ = U; }

  void set_start(const Vecf<Dim>& s) {
    Waypoint<Dim> start(control_);
    start.pos = s;
    start.vel = Vecf<Dim>::Zero();
    start.acc = Vecf<Dim>::Zero();
    start.jrk = Vecf<Dim>::Zero();

    start_ = start;
  }

  void set_goal(const Vecf<Dim>& g) {
    Waypoint<Dim> goal(control_);
    goal.pos = g;
    goal.vel = Vecf<Dim>::Zero();
    goal.acc = Vecf<Dim>::Zero();
    goal.jrk = Vecf<Dim>::Zero();

    goal_ = goal;
  }

  void set_map(const Vecf<Dim>& origin, const Vecf<Dim>& dim) {
    origin_ = origin;
    dim_ = dim;
  }

  void set_obs(const vec_E<PolyhedronObstacle<Dim>>& obs) {
    obs_ = obs;
  }

  bool plan(decimal_t t) {
    if(t > 0)
      start_ = traj_.evaluate(t - traj_t_);
    if(t - traj_t_ < dt_)
      return true;

    planner_ptr.reset(new MPL::PolyMapPlanner<Dim>(false));
    planner_ptr->setMap(origin_, dim_, obs_);// Set collision checking function
    planner_ptr->setVmax(v_max_);      // Set max velocity
    planner_ptr->setAmax(a_max_);      // Set max acceleration
    planner_ptr->setDt(dt_);           // Set dt for each primitive
    planner_ptr->setTol(0.5); // Tolerance for goal region
    planner_ptr->setU(U_); // Set discretization with 1 and efforts

    if(!planner_ptr->plan(start_, goal_))
      return false;

    traj_ = planner_ptr->getTraj();
    traj_t_ = t;
    return true;
  }

  Waypoint<Dim> get_state(decimal_t t) const {
    return traj_.evaluate(t - traj_t_);
  }

  Trajectory<Dim> get_trajectory() const {
    return traj_;
  }

  PolyhedronObstacle<Dim> get_geometry(decimal_t t) const {
    auto state = get_state(t);
    auto polygon = poly_;
    for(auto& it: polygon.vs_) {
      it.p_ += state.pos;
    }

    return PolyhedronObstacle<Dim>(polygon, state.vel);
  }

  Polyhedron<Dim> get_bbox() const {
    return planner_ptr->getBoundingBox();
  }

 private:
  Polyhedron<Dim> poly_;
  std::unique_ptr<PolyMapPlanner<Dim>> planner_ptr;
  decimal_t v_max_{1};
  decimal_t a_max_{1};
  decimal_t dt_{1};
  vec_E<VecDf> U_;
  Waypoint<Dim> start_;
  Waypoint<Dim> goal_;
  Control::Control control_{Control::ACC};
  Vecf<Dim> origin_;
  Vecf<Dim> dim_;
  Trajectory<Dim> traj_;
  vec_E<PolyhedronObstacle<Dim>> obs_;
  decimal_t traj_t_{-10000};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher poly_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons", 1, true);
  ros::Publisher bound_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("bound", 1, true);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud>("states", 1, true);
  ros::Publisher traj1_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("trajectory1", 1, true);
  ros::Publisher traj2_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("trajectory2", 1, true);


  Polyhedron2D rec;
  rec.add(Hyperplane2D(Vec2f(-0.5, 0), -Vec2f::UnitX()));
  rec.add(Hyperplane2D(Vec2f(0.5, 0), Vec2f::UnitX()));
  rec.add(Hyperplane2D(Vec2f(0, -0.5), -Vec2f::UnitY()));
  rec.add(Hyperplane2D(Vec2f(0, 0.5), Vec2f::UnitY()));

  Vec2f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), -5.0);
  nh.param("range_x", dim(0), 20.0);
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

  Robot<2> robot1(rec);
  robot1.set_v_max(v_max);
  robot1.set_a_max(a_max);
  robot1.set_u(U);
  robot1.set_map(origin, dim);
  robot1.set_start(Vec2f(1, 0.1));
  robot1.set_goal(Vec2f(9.5, 0));
  robot1.plan(0);

  Robot<2> robot2(rec);
  robot2.set_v_max(v_max);
  robot2.set_a_max(a_max);
  robot2.set_u(U);
  robot2.set_map(origin, dim);
  robot2.set_start(Vec2f(9, 0));
  robot2.set_goal(Vec2f(0.5, 0));
  robot2.plan(0);


  /*
  vec_E<Robot<2>> robots;
  robots.push_back(robot1);
  robots.push_back(robot2);
  */

  vec_E<Polyhedron2D> bbox;
  bbox.push_back(robot1.get_bbox());
  decomp_ros_msgs::PolyhedronArray bbox_msg = DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bound_pub.publish(bbox_msg);

  ros::Rate loop_rate(5);
  decimal_t update_t = 0.1;
  decimal_t time = 0;

  while (ros::ok()) {
    time += update_t;
    vec_E<PolyhedronObstacle2D> obs1;
    decimal_t d = (robot2.get_state(time).pos - robot1.get_state(time).pos).norm();
    obs1.push_back(robot2.get_geometry(time));
    if(d < 2)
    robot1.set_obs(obs1);

    vec_E<PolyhedronObstacle2D> obs2;
    obs2.push_back(robot1.get_geometry(time));
    if(d < 2)
    robot2.set_obs(obs2);

    robot1.plan(time);
    robot2.plan(time);

    // Publish trajectory
    auto traj1 = robot1.get_trajectory();
    planning_ros_msgs::Trajectory traj1_msg = toTrajectoryROSMsg(traj1);
    traj1_msg.header.frame_id = "map";
    traj1_pub.publish(traj1_msg);

    auto traj2 = robot2.get_trajectory();
    planning_ros_msgs::Trajectory traj2_msg = toTrajectoryROSMsg(traj2);
    traj2_msg.header.frame_id = "map";
    traj2_pub.publish(traj2_msg);

    vec_E<Polyhedron2D> poly_obs;
    for(const auto& it: obs1)
      poly_obs.push_back(it.poly());
    for(const auto& it: obs2)
      poly_obs.push_back(it.poly());
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_obs);
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);

    vec_Vec2f states;
    states.push_back(robot1.get_state(time).pos);
    states.push_back(robot2.get_state(time).pos);

    auto cloud_msg = vec_to_cloud(vec2_to_vec3(states));
    cloud_msg.header.frame_id = "map";
    cloud_pub.publish(cloud_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
