/**
 * @file robot.hpp
 * @brief this file defines Robot class
 */

#include <mpl_external_planner/poly_map_planner/poly_map_planner.h>
#include <ros/ros.h>

using namespace MPL;

/// Robot class
template <int Dim>
class Robot {
 public:
  /// Empty constructor
  Robot() {}

  /// Destructor
  ~Robot() {}

  /// Construct by the given shape
  Robot(const Polyhedron<Dim>& shape, std::string robot_name = "",
        bool verbose = false)
      : shape_(shape), robot_name_(robot_name), verbose_(verbose) {}

  /// Set max velocity for planner
  void set_v_max(decimal_t v) { v_max_ = v; }

  /// Set max acceleration for planner
  void set_a_max(decimal_t a) { a_max_ = a; }

  /// Set primitive dt
  void set_dt(decimal_t dt) { dt_ = dt; }

  /// Set control input for planner
  void set_u(const vec_E<VecDf>& U) { U_ = U; }

  /// Set the starting state
  void set_start(const Vecf<Dim>& s) {
    Waypoint<Dim> start(control_);
    start.pos = s;
    start.vel = Vecf<Dim>::Zero();
    start.acc = Vecf<Dim>::Zero();
    start.jrk = Vecf<Dim>::Zero();

    start_ = start;
  }

  /// Set the goal state
  void set_goal(const Vecf<Dim>& g) {
    Waypoint<Dim> goal(control_);
    goal.pos = g;
    goal.vel = Vecf<Dim>::Zero();
    goal.acc = Vecf<Dim>::Zero();
    goal.jrk = Vecf<Dim>::Zero();

    goal_ = goal;
  }

  /// Set bounding box for the map
  void set_map(const Vecf<Dim>& origin, const Vecf<Dim>& dim) {
    origin_ = origin;
    dim_ = dim;
  }

  /// Set static obstacles (optional)
  void set_static_obs(const vec_E<PolyhedronObstacle<Dim>>& obs) {
    static_obs_ = obs;
  }

  /// Set linear moving obstacles (optional)
  void set_linear_obs(const vec_E<PolyhedronLinearObstacle<Dim>>& obs) {
    linear_obs_ = obs;
  }

  /// Set nonlinear moving obstacles (optional)
  void set_nonlinear_obs(const vec_E<PolyhedronNonlinearObstacle<Dim>>& obs) {
    nonlinear_obs_ = obs;
  }

  /// Set traj init time manually
  void set_traj_t(decimal_t t) { traj_t_ = t; }

  /**
   * @brief planning thread
   * @param t robot current local time
   *
   * actual plan only happens at \f$1/dt\f$ Hz. Current back-end won't terminate
   * if no valid trajectory found.
   *
   */
  bool plan(decimal_t t) {
    if (traj_t_ < 0) {
      history_.push_back(start_.pos);
    } else {
      auto state = traj_.evaluate(t - traj_t_);
      start_ = traj_.evaluate(dt_);
      if (!history_.empty() && (state.pos - history_.back()).norm() > 0.2)
        history_.push_back(state.pos);
    }

    if (t - traj_t_ < dt_ - 1e-8 || (start_.pos - goal_.pos).norm() < 1)
      return true;

    if (verbose_)
      printf("[%s]: start planning at t: %f, traj_t: %f\n", robot_name_.c_str(),
             t, traj_t_);

    ros::Time t0 = ros::Time::now();
    planner_ptr.reset(new MPL::PolyMapPlanner<Dim>(false));
    planner_ptr->setMap(origin_, dim_);            // Set map dimenstion
    planner_ptr->setStaticObstacles(static_obs_);  // Set static obstacles
    planner_ptr->setLinearObstacles(
        linear_obs_);  // Set linear moving obstacles
    planner_ptr->setNonlinearObstacles(
        nonlinear_obs_);           // Set nonlinear moving obstacles
    planner_ptr->setVmax(v_max_);  // Set max velocity
    planner_ptr->setAmax(a_max_);  // Set max acceleration
    planner_ptr->setDt(dt_);       // Set dt for each primitive
    planner_ptr->setTol(0.5);      // Tolerance for goal region
    planner_ptr->setU(U_);         // Set control input

    if (!planner_ptr->plan(start_, goal_)) return false;

    if (verbose_)
      printf("[%s]: takes %f to plan\n", robot_name_.c_str(),
             (ros::Time::now() - t0).toSec());

    traj_ = planner_ptr->getTraj();
    traj_t_ = t;
    // printf("traj_t: %.2f\n", traj_t_);
    return true;
  }

  /// Get start
  Waypoint<Dim> get_start() const { return start_; }

  /// Get robot state at time \f$t\f$
  Waypoint<Dim> get_state(decimal_t t) const {
    return traj_.evaluate(t - traj_t_);
  }

  /// Get robot most recent trajectory
  Trajectory<Dim> get_trajectory() const { return traj_; }

  /// Get trajectory for visualizing
  vec_E<Primitive<Dim>> get_primitives() const { return traj_.getPrimitives(); }

  /// Create linear obstacle model for robot using position and velocity at time
  /// \f$t\f$
  PolyhedronLinearObstacle<Dim> get_linear_obstacle(decimal_t t) const {
    auto state = get_state(t);
    return PolyhedronLinearObstacle<Dim>(shape_, state.pos, state.vel);
  }

  /// Create nonlinear obstacle model for robot using trajectory at time \f$t\f$
  PolyhedronNonlinearObstacle<Dim> get_nonlinear_obstacle(
      decimal_t t, decimal_t max_t = 0) const {
    bool disappear = false;
    auto prs = traj_.getPrimitives();
    if (max_t > 0 && traj_.getTotalTime() > max_t) {
      int n = std::round(max_t / dt_);
      prs.resize(n);
      disappear = true;
    }
    Trajectory<Dim> traj_truncated(prs);
    PolyhedronNonlinearObstacle<Dim> obs(shape_, traj_truncated, t - traj_t_);
    obs.disappear_back_ = disappear;
    return obs;
  }

  /// Get the map bounding box
  Polyhedron<Dim> get_bbox() const { return planner_ptr->getBoundingBox(); }

  /// Get the history of robot's position
  vec_Vecf<Dim> get_history() const { return history_; }

  /// Check if the goal has been reached
  bool reached(decimal_t time) const {
    auto goal = traj_.evaluate(traj_.getTotalTime());
    return get_state(time).pos == goal.pos;
  }

 protected:
  /// Robot shape
  Polyhedron<Dim> shape_;
  /// Planner
  std::shared_ptr<PolyMapPlanner<Dim>> planner_ptr;
  /// Max velocity
  decimal_t v_max_{1};
  /// Max acceleration
  decimal_t a_max_{1};
  /// Primitive dt
  decimal_t dt_{1};
  /// Control input
  vec_E<VecDf> U_;
  /// Initial start
  Waypoint<Dim> start_;
  /// Goal
  Waypoint<Dim> goal_;
  /// Control flag, use acceleration by default
  Control::Control control_{Control::ACC};
  /// Map origin
  Vecf<Dim> origin_;
  /// Map dimension
  Vecf<Dim> dim_;
  /// Current trajectory
  Trajectory<Dim> traj_;
  /// Static obstacles
  vec_E<PolyhedronObstacle<Dim>> static_obs_;
  /// Linear obstacles
  vec_E<PolyhedronLinearObstacle<Dim>> linear_obs_;
  /// Nonlinear obstacles
  vec_E<PolyhedronNonlinearObstacle<Dim>> nonlinear_obs_;
  /// Robot past position
  vec_Vecf<Dim> history_;
  /// Trajectory initial time in global frame
  decimal_t traj_t_{-10000};
  /// Robot name
  std::string robot_name_;
  /// Verbose
  bool verbose_{false};
};

/// Robot 2D
typedef Robot<2> Robot2D;

/// Robot 3D
typedef Robot<3> Robot3D;
