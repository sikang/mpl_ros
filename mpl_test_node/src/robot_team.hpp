#include "robot.hpp"

template <int Dim>
class HomogeneousRobotTeam {
 public:
  HomogeneousRobotTeam(decimal_t ddt = 0) : ddt_(ddt) {}

  /// Set max velocity for planner
  void set_v_max(decimal_t v) { v_max_ = v; }

  /// Set max acceleration for planner
  void set_a_max(decimal_t a) { a_max_ = a; }

  /// Set primitive dt
  void set_dt(decimal_t dt) { dt_ = dt; }

  /// Set control input for planner
  void set_u(const vec_E<VecDf>& U) { U_ = U; }

  /// Set bounding box for the map
  void set_map(const Vecf<Dim>& origin, const Vecf<Dim>& dim) {
    origin_ = origin;
    dim_ = dim;
  }

  /// Set planner versbose for individual robot
  void set_verbose(bool verbose) { verbose_ = verbose; }

  /// Set geometry of robot
  void set_geometry(const Polyhedron<Dim>& rec) { rec_ = rec; }

  /// Set obstacles for each robot
  vec_E<Polyhedron<Dim>> set_obs(decimal_t time) {
    vec_E<Polyhedron<Dim>> poly_obs;
    for (size_t i = 0; i < robots_.size(); i++) {
      vec_E<PolyhedronLinearObstacle<Dim>> linear_obs;
      vec_E<PolyhedronNonlinearObstacle<Dim>> nonlinear_obs;
      for (size_t j = 0; j < robots_.size(); j++) {
        if (i != j)
          nonlinear_obs.push_back(
              robots_[j]->get_nonlinear_obstacle(time, traj_time_));
        // linear_obs.push_back(robots_[j]->get_linear_obstacle(time));
      }
      robots_[i]->set_static_obs(static_obs_);
      robots_[i]->set_linear_obs(linear_obs);
      robots_[i]->set_nonlinear_obs(nonlinear_obs);
      poly_obs.push_back(robots_[i]->get_nonlinear_obstacle(time).poly(0));
    }
    for (const auto& it : static_obs_) poly_obs.push_back(it.poly(0));
    return poly_obs;
  }

  /// Get robots
  std::vector<std::shared_ptr<Robot<Dim>>> get_robots() { return robots_; }

  /// Get current obstacles
  vec_E<Polyhedron<Dim>> get_obs() { return curr_obs_; }

  /// Plan decentralized
  bool update_decentralized(decimal_t time) {
    curr_obs_ = set_obs(time);

    for (auto& it : robots_)
      if (!it->plan(time)) return false;
    return true;
  }

  /// Plan decentralized
  bool update_centralized(decimal_t time) {
    curr_obs_.clear();
    for (const auto& it : robots_)
      curr_obs_.push_back(it->get_nonlinear_obstacle(time).poly(0));
    for (const auto& it : static_obs_) curr_obs_.push_back(it.poly(0));

    static bool planned = false;
    if (planned) return true;

    for (size_t i = 0; i < robots_.size(); i++) {
      vec_E<PolyhedronLinearObstacle<Dim>> linear_obs;
      vec_E<PolyhedronNonlinearObstacle<Dim>> nonlinear_obs;
      for (size_t j = 0; j < i; j++) {
        nonlinear_obs.push_back(robots_[j]->get_nonlinear_obstacle(time, 0));
        // linear_obs.push_back(robots_[j]->get_linear_obstacle(time));
      }
      robots_[i]->set_static_obs(static_obs_);
      robots_[i]->set_linear_obs(linear_obs);
      robots_[i]->set_nonlinear_obs(nonlinear_obs);
      robots_[i]->set_traj_t(-1);

      if (!robots_[i]->plan(0)) return false;
    }
    planned = true;
    return true;
  }

  /// Check if all robot reaches the goal
  bool finished(decimal_t time) {
    bool reached = true;
    for (const auto& it : robots_)
      if (!it->reached(time)) return false;
    return true;
  }

  virtual void init(){};

 protected:
  /// Planning gap
  decimal_t ddt_{0};
  /// Max velocity
  decimal_t v_max_{1};
  /// Max acceleration
  decimal_t a_max_{1};
  /// Primitive dt
  decimal_t dt_{1};
  /// Control input
  vec_E<VecDf> U_;
  /// Map origin
  Vecf<Dim> origin_;
  /// Map dimension
  Vecf<Dim> dim_;
  /// Verbose
  bool verbose_{true};
  /// Trajectory duration for each robot's nonlinear obstacle
  decimal_t traj_time_{4.0};
  /// Robot shape
  Polyhedron<Dim> rec_;
  /// Obstacle course
  vec_E<Polyhedron<Dim>> curr_obs_;
  /// Robot array
  std::vector<std::shared_ptr<Robot<Dim>>> robots_;
  /// Static Obstacle array
  vec_E<PolyhedronObstacle2D> static_obs_;
};

/*********************** Team 1: 10 robots **************************/
/********************************************************************/
/******************            |    |            ********************/
/******************  o         |    |         o  ********************/
/******************            |    |            ********************/
/******************            |    |            ********************/
/******************  o         |    |         o  ********************/
/******************            |    |            ********************/
/******************            |____|            ********************/
/******************  o          ____          o  ********************/
/******************            |    |            ********************/
/******************            |    |            ********************/
/******************  o         |    |         o  ********************/
/******************            |    |            ********************/
/******************            |    |            ********************/
/******************  o         |    |         o  ********************/
/******************            |    |            ********************/
/********************************************************************/
class Team1 : public HomogeneousRobotTeam<2> {
 public:
  Team1(decimal_t ddt) : HomogeneousRobotTeam<2>(ddt) {}

  void init() {
    traj_time_ = v_max_ / U_.front().lpNorm<Eigen::Infinity>() / dt_;
    printf("traj time: %f\n", traj_time_);

    std::shared_ptr<Robot2D> robot0 =
        std::make_shared<Robot2D>(rec_, "robot0", verbose_);
    robot0->set_start(Vec2f(0, -5));
    robot0->set_goal(Vec2f(10, -5));

    std::shared_ptr<Robot2D> robot1 =
        std::make_shared<Robot2D>(rec_, "robot1", verbose_);
    robot1->set_start(Vec2f(0, -2.5));
    robot1->set_goal(Vec2f(10, -2.5));

    std::shared_ptr<Robot2D> robot2 =
        std::make_shared<Robot2D>(rec_, "robot2", verbose_);
    robot2->set_start(Vec2f(0, 0));
    robot2->set_goal(Vec2f(10, 0));

    std::shared_ptr<Robot2D> robot3 =
        std::make_shared<Robot2D>(rec_, "robot3", verbose_);
    robot3->set_start(Vec2f(0, 2.5));
    robot3->set_goal(Vec2f(10, 2.5));

    std::shared_ptr<Robot2D> robot4 =
        std::make_shared<Robot2D>(rec_, "robot4", verbose_);
    robot4->set_start(Vec2f(0, 5));
    robot4->set_goal(Vec2f(10, 5));

    std::shared_ptr<Robot2D> robot5 =
        std::make_shared<Robot2D>(rec_, "robot5", verbose_);
    robot5->set_start(Vec2f(10, 0));
    robot5->set_goal(Vec2f(0, 0));

    std::shared_ptr<Robot2D> robot6 =
        std::make_shared<Robot2D>(rec_, "robot6", verbose_);
    robot6->set_start(Vec2f(10, 2.5));
    robot6->set_goal(Vec2f(0, 2.5));

    std::shared_ptr<Robot2D> robot7 =
        std::make_shared<Robot2D>(rec_, "robot7", verbose_);
    robot7->set_start(Vec2f(10, 5));
    robot7->set_goal(Vec2f(0, 5));

    std::shared_ptr<Robot2D> robot8 =
        std::make_shared<Robot2D>(rec_, "robot8", verbose_);
    robot8->set_start(Vec2f(10, -2.5));
    robot8->set_goal(Vec2f(0, -2.5));

    std::shared_ptr<Robot2D> robot9 =
        std::make_shared<Robot2D>(rec_, "robot9", verbose_);
    robot9->set_start(Vec2f(10, -5));
    robot9->set_goal(Vec2f(0, -5));

    robots_.push_back(robot0);
    robots_.push_back(robot1);
    robots_.push_back(robot2);
    robots_.push_back(robot3);
    robots_.push_back(robot4);
    robots_.push_back(robot5);
    robots_.push_back(robot6);
    robots_.push_back(robot7);
    robots_.push_back(robot8);
    robots_.push_back(robot9);

    decimal_t ddt = 0;
    for (auto robot : robots_) {
      robot->set_v_max(v_max_);
      robot->set_a_max(a_max_);
      robot->set_u(U_);
      robot->set_dt(dt_);
      robot->set_map(origin_, dim_);
      robot->plan(ddt);  // need to set a small difference in the starting time
      ddt += ddt_;
    }

    // Build the obstacle course
    Polyhedron2D rec2;
    rec2.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
    rec2.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
    rec2.add(Hyperplane2D(Vec2f(5, 0.2), -Vec2f::UnitY()));
    rec2.add(Hyperplane2D(Vec2f(5, 5.5), Vec2f::UnitY()));
    static_obs_.push_back(PolyhedronObstacle2D(rec2, Vec2f::Zero()));

    Polyhedron2D rec3;
    rec3.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
    rec3.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
    rec3.add(Hyperplane2D(Vec2f(5, -5.5), -Vec2f::UnitY()));
    rec3.add(Hyperplane2D(Vec2f(5, -0.2), Vec2f::UnitY()));
    static_obs_.push_back(PolyhedronObstacle2D(rec3, Vec2f::Zero()));

    printf("Team1 Initialized!\n\n");
  }
};

/************************ Team 2: 16 robots **************************/
/********************************************************************/
/******************                             *********************/
/******************  o     o     o     o     o  *********************/
/******************                             *********************/
/******************  o                       o  *********************/
/******************              _              *********************/
/******************  o          |_|          o  *********************/
/******************                             *********************/
/******************  o                       o  *********************/
/******************                             *********************/
/******************  o     o     o     o     o  *********************/
/******************                             *********************/
/********************************************************************/
class Team2 : public HomogeneousRobotTeam<2> {
 public:
  Team2(decimal_t ddt) : HomogeneousRobotTeam<2>(ddt) {}

  void init() {
    // traj_time_ = v_max_ / U_.front().lpNorm<Eigen::Infinity>() / dt_ + dt_;
    traj_time_ = 0;
    printf("traj time: %f\n", traj_time_);

    std::shared_ptr<Robot2D> robot0 =
        std::make_shared<Robot2D>(rec_, "robot0", verbose_);
    robot0->set_start(Vec2f(0, -5));
    robot0->set_goal(Vec2f(10, 5));

    std::shared_ptr<Robot2D> robot1 =
        std::make_shared<Robot2D>(rec_, "robot1", verbose_);
    robot1->set_start(Vec2f(0, -2.5));
    robot1->set_goal(Vec2f(10, 2.5));

    std::shared_ptr<Robot2D> robot2 =
        std::make_shared<Robot2D>(rec_, "robot2", verbose_);
    robot2->set_start(Vec2f(0, 0));
    robot2->set_goal(Vec2f(10, 0));

    std::shared_ptr<Robot2D> robot3 =
        std::make_shared<Robot2D>(rec_, "robot3", verbose_);
    robot3->set_start(Vec2f(0, 2.5));
    robot3->set_goal(Vec2f(10, -2.5));

    std::shared_ptr<Robot2D> robot4 =
        std::make_shared<Robot2D>(rec_, "robot4", verbose_);
    robot4->set_start(Vec2f(0, 5));
    robot4->set_goal(Vec2f(10, -5));

    std::shared_ptr<Robot2D> robot5 =
        std::make_shared<Robot2D>(rec_, "robot5", verbose_);
    robot5->set_start(Vec2f(2.5, 5));
    robot5->set_goal(Vec2f(7.5, -5));

    std::shared_ptr<Robot2D> robot6 =
        std::make_shared<Robot2D>(rec_, "robot6", verbose_);
    robot6->set_start(Vec2f(5, 5));
    robot6->set_goal(Vec2f(5, -5));

    std::shared_ptr<Robot2D> robot7 =
        std::make_shared<Robot2D>(rec_, "robot7", verbose_);
    robot7->set_start(Vec2f(7.5, 5));
    robot7->set_goal(Vec2f(2.5, -5));

    std::shared_ptr<Robot2D> robot8 =
        std::make_shared<Robot2D>(rec_, "robot8", verbose_);
    robot8->set_start(Vec2f(10, 5));
    robot8->set_goal(Vec2f(0, -5));

    std::shared_ptr<Robot2D> robot9 =
        std::make_shared<Robot2D>(rec_, "robot9", verbose_);
    robot9->set_start(Vec2f(10, 2.5));
    robot9->set_goal(Vec2f(0, -2.5));

    std::shared_ptr<Robot2D> robot10 =
        std::make_shared<Robot2D>(rec_, "robot10", verbose_);
    robot10->set_start(Vec2f(10, 0));
    robot10->set_goal(Vec2f(0, 0));

    std::shared_ptr<Robot2D> robot11 =
        std::make_shared<Robot2D>(rec_, "robot11", verbose_);
    robot11->set_start(Vec2f(10, -2.5));
    robot11->set_goal(Vec2f(0, 2.5));

    std::shared_ptr<Robot2D> robot12 =
        std::make_shared<Robot2D>(rec_, "robot12", verbose_);
    robot12->set_start(Vec2f(10, -5));
    robot12->set_goal(Vec2f(0, 5));

    std::shared_ptr<Robot2D> robot13 =
        std::make_shared<Robot2D>(rec_, "robot13", verbose_);
    robot13->set_start(Vec2f(7.5, -5));
    robot13->set_goal(Vec2f(2.5, 5));

    std::shared_ptr<Robot2D> robot14 =
        std::make_shared<Robot2D>(rec_, "robot14", verbose_);
    robot14->set_start(Vec2f(5, -5));
    robot14->set_goal(Vec2f(5, 5));

    std::shared_ptr<Robot2D> robot15 =
        std::make_shared<Robot2D>(rec_, "robot15", verbose_);
    robot15->set_start(Vec2f(2.5, -5));
    robot15->set_goal(Vec2f(7.5, 5));

    robots_.push_back(robot0);
    robots_.push_back(robot1);
    robots_.push_back(robot2);
    robots_.push_back(robot3);
    robots_.push_back(robot4);
    robots_.push_back(robot5);
    robots_.push_back(robot6);
    robots_.push_back(robot7);
    robots_.push_back(robot8);
    robots_.push_back(robot9);
    robots_.push_back(robot10);
    robots_.push_back(robot11);
    robots_.push_back(robot12);
    robots_.push_back(robot13);
    robots_.push_back(robot14);
    robots_.push_back(robot15);

    decimal_t ddt = 0;
    for (auto robot : robots_) {
      robot->set_v_max(v_max_);
      robot->set_a_max(a_max_);
      robot->set_u(U_);
      robot->set_dt(dt_);
      robot->set_map(origin_, dim_);
      robot->plan(ddt);  // need to set a small difference in the starting time
      ddt += ddt_;
    }

    Polyhedron2D rec1;
    rec1.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(5, -1), -Vec2f::UnitY()));
    rec1.add(Hyperplane2D(Vec2f(5, 1), Vec2f::UnitY()));
    static_obs_.push_back(PolyhedronObstacle2D(rec1, Vec2f::Zero()));

    printf("Team2 Initialized!\n\n");
  }
};
