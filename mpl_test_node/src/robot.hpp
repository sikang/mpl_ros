#include <mpl_external_planner/poly_map_planner/poly_map_planner.h>

using namespace MPL;

template <int Dim>
class Robot {
 public:
  Robot() {}

  ~Robot() {}

  Robot(const Polyhedron<Dim>& poly) : poly_(poly) {}

  void set_v_max(decimal_t v) { v_max_ = v; }

  void set_a_max(decimal_t a) { a_max_ = a; }

  void set_dt(decimal_t dt) { dt_ = dt; }

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

  void set_static_obs(const vec_E<PolyhedronObstacle<Dim>>& obs) {
    static_obs_ = obs;
  }

  void set_linear_obs(const vec_E<PolyhedronLinearObstacle<Dim>>& obs) {
    linear_obs_ = obs;
  }

  void set_nonlinear_obs(const vec_E<PolyhedronNonlinearObstacle<Dim>>& obs) {
    nonlinear_obs_ = obs;
  }

  bool plan(decimal_t t) {
    if(traj_t_ < 0) {
      history_.push_back(start_.pos);
    }
    else {
      start_ = traj_.evaluate(t - traj_t_);
      if(!history_.empty() && (start_.pos - history_.back()).norm() > 0.1)
        history_.push_back(start_.pos);
    }

    //printf("t: %.2f, traj_t: %.2f\n", t, traj_t_);
    if(t - traj_t_ < dt_)
      return true;

    //printf("plan: %.2f\n", t);
    planner_ptr.reset(new MPL::PolyMapPlanner<Dim>(false));
    planner_ptr->setMap(origin_, dim_);// Set collision checking function
    planner_ptr->setStaticObstacles(static_obs_);
    planner_ptr->setLinearObstacles(linear_obs_);
    planner_ptr->setNonlinearObstacles(nonlinear_obs_);
    planner_ptr->setVmax(v_max_);      // Set max velocity
    planner_ptr->setAmax(a_max_);      // Set max acceleration
    planner_ptr->setDt(dt_);           // Set dt for each primitive
    planner_ptr->setTol(0.5); // Tolerance for goal region
    planner_ptr->setU(U_); // Set discretization with 1 and efforts

    if(!planner_ptr->plan(start_, goal_))
      return false;

    traj_ = planner_ptr->getTraj();
    traj_t_ = t;
    //printf("traj_t: %.2f\n", traj_t_);
    return true;
  }

  Waypoint<Dim> get_state(decimal_t t) const {
    return traj_.evaluate(t - traj_t_);
  }

  Trajectory<Dim> get_trajectory() const {
    return traj_;
  }

  vec_E<Primitive<Dim>> get_primitives() const {
    return traj_.getPrimitives();
  }

  PolyhedronLinearObstacle<Dim> get_linear_geometry(decimal_t t) const {
    auto state = get_state(t);
    return PolyhedronLinearObstacle<Dim>(poly_, state.pos, state.vel);
  }

  PolyhedronNonlinearObstacle<Dim> get_nonlinear_geometry(decimal_t t) const {
    return PolyhedronNonlinearObstacle<Dim>(poly_, traj_, t - traj_t_);
  }

  Polyhedron<Dim> get_bbox() const {
    return planner_ptr->getBoundingBox();
  }

  vec_Vecf<Dim> get_history() const {
    return history_;
  }

 private:
  Polyhedron<Dim> poly_;
  std::shared_ptr<PolyMapPlanner<Dim>> planner_ptr;
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
  vec_E<PolyhedronObstacle<Dim>> static_obs_;
  vec_E<PolyhedronLinearObstacle<Dim>> linear_obs_;
  vec_E<PolyhedronNonlinearObstacle<Dim>> nonlinear_obs_;
  vec_Vecf<Dim> history_;
  decimal_t traj_t_{-10000};
};


