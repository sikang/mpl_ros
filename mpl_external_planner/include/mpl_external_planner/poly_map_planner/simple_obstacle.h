#include <decomp_geometry/polyhedron.h>
#include <mpl_basis/trajectory.h>

/// Base polygonal obstacle class
template <int Dim>
class PolyhedronObstacle {
 public:
  /// Empty constructor
  PolyhedronObstacle() {}
  /**
   * @brief Basic constructor
   * @param poly shape of obstacle
   * @param p initial representative point
   */
  PolyhedronObstacle(const Polyhedron<Dim>& poly, const Vecf<Dim>& p)
      : poly_(poly), p_(p) {}
  /// Get representative point
  Vecf<Dim> p() const { return p_; }
  /// Get geometry of obstalce at time \f$t\f$
  virtual Polyhedron<Dim> poly(decimal_t t) const {
    auto poly = poly_;
    for (auto& it : poly.vs_) it.p_ += p_;
    return poly;
  }
  /// Get raw geometry
  Polyhedron<Dim> geometry() const { return poly_; }
  /// Check if the given point is inside
  bool inside(const Vecf<Dim>& pt) const { return poly_.inside(pt - p_); }
  /// Update the representative point
  virtual void update(decimal_t t) {}

  vec_E<Polyhedron<Dim>> predict(decimal_t dt, size_t N,
                                 decimal_t t = 0) const {
    vec_E<Polyhedron<Dim>> polys;
    for (size_t i = 0; i < N; i++) {
      polys.push_back(poly(i * dt + t));
    }
    return polys;
  }

 protected:
  /// Obstacle initial contour
  Polyhedron<Dim> poly_;
  /// Obstacle initial representative position
  Vecf<Dim> p_;
};

typedef PolyhedronObstacle<2> PolyhedronObstacle2D;

typedef PolyhedronObstacle<3> PolyhedronObstacle3D;

/// Linear obstacle class
template <int Dim>
class PolyhedronLinearObstacle : public PolyhedronObstacle<Dim> {
 public:
  /// Empty constructor
  PolyhedronLinearObstacle() {}
  /**
   * @brief Basic constructor
   * @param poly shape of obstacle
   * @param p initial representative point
   */
  PolyhedronLinearObstacle(const Polyhedron<Dim>& poly, const Vecf<Dim>& p)
      : PolyhedronObstacle<Dim>(poly, p) {}
  /**
   * @brief Basic constructor
   * @param poly shape of obstacle
   * @param p initial representative point
   * @param v obstacle's velocity
   */
  PolyhedronLinearObstacle(const Polyhedron<Dim>& poly, const Vecf<Dim>& p,
                           const Vecf<Dim>& v)
      : PolyhedronObstacle<Dim>(poly, p), v_(v) {}
  /// Get velocity
  Vecf<Dim> v() const { return v_; }
  /// Get geometry of obstalce at time \f$t\f$
  Polyhedron<Dim> poly(decimal_t t) const {
    auto poly = this->poly_;
    for (auto& it : poly.vs_) it.p_ += v_ * t + this->p_ + cov_v_ * it.n_ * t;
    return poly;
  }
  /// Check if the given point is inside at time \f$t\f$
  bool inside(const Vecf<Dim>& pt, decimal_t t) const {
    return poly(t).inside(pt);
    // return this->poly_.inside(pt - this->p_ - t * v_);
  }
  /// Get covariance velocity
  decimal_t cov_v() const { return cov_v_; }
  void set_cov_v(decimal_t v) { cov_v_ = v; }

  /// Update the representative point, it resets the initial representative
  /// point
  void update(decimal_t t) { this->p_ += v_ * t; }

 protected:
  /// Obstacle velocity
  Vecf<Dim> v_{Vecf<Dim>::Zero()};
  /// Covariance increasing speed
  decimal_t cov_v_{0};
};

typedef PolyhedronLinearObstacle<2> PolyhedronLinearObstacle2D;

typedef PolyhedronLinearObstacle<3> PolyhedronLinearObstacle3D;

/// Nonlinear obstacle class
template <int Dim>
class PolyhedronNonlinearObstacle : public PolyhedronObstacle<Dim> {
 public:
  /// Empty constructor
  PolyhedronNonlinearObstacle() {}
  /**
   * @brief Basic constructor
   * @param poly shape of obstacle
   * @param traj trajecotory for representative point
   * @param t start time
   */
  PolyhedronNonlinearObstacle(const Polyhedron<Dim>& poly,
                              const Trajectory<Dim>& traj, decimal_t t)
      : PolyhedronObstacle<Dim>(poly, traj.evaluate(t).pos),
        traj_(traj),
        start_t_(t) {}
  /// Check if the given point is inside at time \f$t\f$
  bool inside(const Vecf<Dim>& pt, decimal_t t) const {
    t += start_t_;
    const auto w = traj_.evaluate(t);
    if (t <= traj_.getTotalTime() && t >= 0)
      return this->poly_.inside(pt - w.pos);
    else if (t < 0 && !disappear_front_)
      return this->poly_.inside(pt - w.pos);
    else if (t > traj_.getTotalTime() && !disappear_back_)
      return this->poly_.inside(pt - w.pos);
    else
      return false;
  }
  /// Get geometry of obstalce at time \f$t\f$
  Polyhedron<Dim> poly(decimal_t t) const {
    const auto w = traj_.evaluate(t + start_t_);
    auto poly = this->poly_;
    for (auto& it : poly.vs_) it.p_ += w.pos;
    return poly;
  }
  /// Get the start time
  decimal_t start_t() const { return start_t_; }

  /// Get the trajectory
  Trajectory<Dim> traj() const { return traj_; }

  /// Downgrade to linear obstacle at time \f$t\f$
  PolyhedronLinearObstacle<Dim> get_linear_obstacle(decimal_t t) const {
    auto waypoint = traj_.evaluate(t + start_t_);
    if (t + start_t_ >= 0)
      return PolyhedronLinearObstacle<Dim>(this->poly_, waypoint.pos,
                                           waypoint.vel);
    else
      return PolyhedronLinearObstacle<Dim>(this->poly_, waypoint.pos,
                                           Vecf<Dim>::Zero());
  }

  bool disappear_front_{false};
  bool disappear_back_{false};

 protected:
  Trajectory<Dim> traj_;
  decimal_t start_t_{0};
};

typedef PolyhedronNonlinearObstacle<2> PolyhedronNonlinearObstacle2D;

typedef PolyhedronNonlinearObstacle<3> PolyhedronNonlinearObstacle3D;

/// Nonlinear obstacle class
template <int Dim>
class PolyhedronCircularObstacle : public PolyhedronObstacle<Dim> {
 public:
  /// Empty constructor
  PolyhedronCircularObstacle() {}
  /**
   * @brief Basic constructor
   * @param poly shape of obstacle
   * @param traj trajecotory for representative point
   * @param t start time
   */
  PolyhedronCircularObstacle(const Polyhedron<Dim>& poly,
                             const Vecf<Dim>& center, decimal_t r, decimal_t w,
                             decimal_t init_angle = 0)
      : center_(center), radius_(r), w_(w), init_angle_(init_angle) {
    this->poly_ = poly;
  }

  Polyhedron<Dim> poly(decimal_t t) const {
    Vecf<Dim> offset = Vecf<Dim>::Zero();
    offset(0) = radius_ * cos(init_angle_ + t * w_);
    offset(1) = radius_ * sin(init_angle_ + t * w_);

    auto poly = this->poly_;
    for (auto& it : poly.vs_) it.p_ += center_ + offset;
    return poly;
  }

  /// Downgrade to linear obstacle at time \f$t\f$
  PolyhedronLinearObstacle<Dim> get_linear_obstacle(decimal_t t) const {
    Vecf<Dim> offset = Vecf<Dim>::Zero();
    offset(0) = radius_ * cos(init_angle_ + t * w_);
    offset(1) = radius_ * sin(init_angle_ + t * w_);

    auto pos = center_ + offset;

    Vecf<Dim> vel = Vecf<Dim>::Zero();
    vel(0) = -w_ * radius_ * sin(init_angle_ + t * w_);
    vel(1) = w_ * radius_ * cos(init_angle_ + t * w_);
    return PolyhedronLinearObstacle<Dim>(this->poly_, pos, vel);
  }

  /// Get future trajectory for duration T
  vec_Vecf<Dim> get_future_traj(decimal_t t0, decimal_t T = 2) const {
    vec_Vecf<Dim> traj;
    for (decimal_t t = t0; t <= t0 + T; t += 0.1) {
      Vecf<Dim> offset = Vecf<Dim>::Zero();
      offset(0) = radius_ * cos(init_angle_ + t * w_);
      offset(1) = radius_ * sin(init_angle_ + t * w_);
      traj.push_back(center_ + offset);
    }
    return traj;
  }

 protected:
  /// Center of circular trajectory
  Vecf<Dim> center_;
  /// Radius of circular trajectory
  decimal_t radius_;
  /// Angular velocity of circular trajectory
  decimal_t w_;
  /// Initial angle
  decimal_t init_angle_;
};

typedef PolyhedronCircularObstacle<2> PolyhedronCircularObstacle2D;

typedef PolyhedronCircularObstacle<3> PolyhedronCircularObstacle3D;
