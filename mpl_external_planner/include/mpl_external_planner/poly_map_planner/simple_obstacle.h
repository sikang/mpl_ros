#include <decomp_geometry/polyhedron.h>
#include <mpl_basis/trajectory.h>

///Base polygonal obstacle class
template <int Dim>
class PolyhedronObstacle {
 public:
  PolyhedronObstacle() {}
  PolyhedronObstacle(const Polyhedron<Dim>& poly, const Vecf<Dim>& p) :
      poly_(poly), p_(p) {}
  bool inside (const Vecf<Dim>& pt) const { return poly_.inside(pt - p_); }
  Vecf<Dim> p() const { return p_; }
  virtual Polyhedron<Dim> poly(decimal_t t) const {
    auto poly = poly_;
    for(auto& it: poly.vs_)
      it.p_ += p_;
    return poly;
  }
  Polyhedron<Dim> geometry() const {
    return poly_;
  }
 protected:
  /// Obstacle initial contour
  Polyhedron<Dim> poly_;
  ///Obstacle initial position
  Vecf<Dim> p_;
};

typedef PolyhedronObstacle<2> PolyhedronObstacle2D;

typedef PolyhedronObstacle<3> PolyhedronObstacle3D;



///Linear obstacle class
template <int Dim>
class PolyhedronLinearObstacle : public PolyhedronObstacle<Dim> {
 public:
  PolyhedronLinearObstacle() {}
  PolyhedronLinearObstacle(const Polyhedron<Dim> &poly, const Vecf<Dim>& p)
      : PolyhedronObstacle<Dim>(poly, p) {}
  PolyhedronLinearObstacle(const Polyhedron<Dim> &poly,
                           const Vecf<Dim>& p, const Vecf<Dim> &v)
      : PolyhedronObstacle<Dim>(poly, p), v_(v) {}
  Vecf<Dim> v() const { return v_; }
  bool inside (const Vecf<Dim>& pt, decimal_t t) const {
    return this->poly_.inside(pt - this->p_ - t * v_);
  }
  Polyhedron<Dim> poly(decimal_t t) const {
    auto poly = this->poly_;
    for(auto& it: poly.vs_)
      it.p_ += v_ * t + this->p_;
    return poly;
  }
 protected:
 ///Obstacle velocity
	Vecf<Dim> v_{Vecf<Dim>::Zero()};
};

typedef PolyhedronLinearObstacle<2> PolyhedronLinearObstacle2D;

typedef PolyhedronLinearObstacle<3> PolyhedronLinearObstacle3D;

template <int Dim>
class PolyhedronNonlinearObstacle : public PolyhedronObstacle<Dim> {
 public:
  PolyhedronNonlinearObstacle() {}
  PolyhedronNonlinearObstacle(const Polyhedron<Dim> &poly,
                              const Trajectory<Dim> &traj, decimal_t t)
      : PolyhedronObstacle<Dim>(poly, traj.evaluate(t).pos), traj_(traj),
        start_t_(t) {}
  bool inside(const Vecf<Dim> &pt, decimal_t t) const {
    t += start_t_;
    const auto w = traj_.evaluate(t);
    if(t <= traj_.getTotalTime() && t >= 0)
      return this->poly_.inside(pt - w.pos);
    else if(t < 0 && !disappear_front_)
      return this->poly_.inside(pt - w.pos);
    else if(t > traj_.getTotalTime() && !disappear_back_)
      return this->poly_.inside(pt - w.pos);
    else
      return false;
  }
  Polyhedron<Dim> poly(decimal_t t) const {
    const auto w = traj_.evaluate(t+start_t_);
    auto poly = this->poly_;
    for(auto& it: poly.vs_)
      it.p_ += w.pos;
    return poly;
  }
  decimal_t start_t() const { return start_t_; }

  Trajectory<Dim> traj() const { return traj_; }

  bool disappear_front_{false};
  bool disappear_back_{false};
 protected:
  Trajectory<Dim> traj_;
  decimal_t start_t_{0};
};

typedef PolyhedronNonlinearObstacle<2> PolyhedronNonlinearObstacle2D;

typedef PolyhedronNonlinearObstacle<3> PolyhedronNonlinearObstacle3D;


