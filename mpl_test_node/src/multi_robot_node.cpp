#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_external_planner/poly_map_planner/poly_map_planner.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

using namespace MPL;

template <int Dim>
class Robot {
 public:
  Robot(const Polyhedron<Dim>& poly, const Vecf<Dim>& pos) {
    auto polygon = poly;
    for(auto& it: polygon.vs_) {
      it.p_ += pos;
    }

    geometry_ = PolyhedronObstacle<Dim>(polygon);
    set_start(pos);
  }

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

  void set_obs(const Vecf<Dim>& origin, const Vecf<Dim>& dim,
               const vec_E<PolyhedronObstacle<Dim>>& obs) {
    planner_ptr.reset(new MPL::PolyMapPlanner<Dim>(false));
    planner_ptr->setMap(origin, dim, obs);         // Set collision checking function
    planner_ptr->setVmax(v_max_);      // Set max velocity
    planner_ptr->setAmax(a_max_);      // Set max acceleration
    planner_ptr->setDt(dt_);           // Set dt for each primitive
    planner_ptr->setTol(0.5); // Tolerance for goal region
    planner_ptr->setU(U_); // Set discretization with 1 and efforts
  }

  bool plan() {
    if(!planner_ptr->plan(start_, goal_))
      return false;

    auto traj = planner_ptr->getTraj();
    start_ = traj.evaluate(dt_);
    geometry_ = PolyhedronObstacle<Dim>(geometry_.poly(dt_), start_.vel);
    return true;
  }


  Trajectory<Dim> get_trajectory() const {
    return planner_ptr->getTraj();
  }

  PolyhedronObstacle<Dim> get_geometry() const {
    return geometry_;
  }

 private:
  PolyhedronObstacle<Dim> geometry_;
  std::unique_ptr<PolyMapPlanner<Dim>> planner_ptr;
  decimal_t v_max_{1};
  decimal_t a_max_{1};
  decimal_t dt_{1};
  vec_E<VecDf> U_;
  Waypoint<Dim> start_;
  Waypoint<Dim> goal_;
  Control::Control control_{Control::ACC};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher poly_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons", 1, true);
  ros::Publisher bound_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("bound", 1, true);
  ros::Publisher sg_pub =
      nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
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

  Robot<2> robot1(rec, Vec2f(1, 0));
  robot1.set_v_max(v_max);
  robot1.set_a_max(a_max);
  robot1.set_u(U);
  robot1.set_goal(Vec2f(10, 0));

  Robot<2> robot2(rec, Vec2f(9, 0));
  robot2.set_v_max(v_max);
  robot2.set_a_max(a_max);
  robot2.set_u(U);
  robot2.set_goal(Vec2f(0, 0));


  /*
  vec_E<Polyhedron2D> bbox;
  bbox.push_back(planner_ptr->getBoundingBox());
  decomp_ros_msgs::PolyhedronArray bbox_msg = DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bound_pub.publish(bbox_msg);
  */

  ros::Rate loop_rate(1);

  while (ros::ok()) {
    vec_E<PolyhedronObstacle2D> obs1;
    obs1.push_back(robot2.get_geometry());
    robot1.set_obs(origin, dim, obs1);
    robot1.plan();

    vec_E<PolyhedronObstacle2D> obs2;
    obs2.push_back(robot1.get_geometry());
    robot2.set_obs(origin, dim, obs2);
    robot2.plan();


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

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
