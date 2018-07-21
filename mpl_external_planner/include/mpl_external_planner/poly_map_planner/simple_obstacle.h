#include <decomp_geometry/polyhedron.h>

///Based obstacle class
template <int Dim>
class PolyhedronObstacle {
 public:
  PolyhedronObstacle() {}
  PolyhedronObstacle(const Polyhedron<Dim>& poly) : poly_(poly) {}
  PolyhedronObstacle(const Polyhedron<Dim>& poly, const Vecf<Dim>& v) : poly_(poly), v_(v) {}
  bool inside (const Vecf<Dim>& pt) const { return poly_.inside(pt); }
  vec_E<Hyperplane<Dim>> hyperplanes() const { return poly_.hyperplanes(); }
  Vecf<Dim> v() const { return v_; }
  Polyhedron<Dim> poly() const { return poly_; }
  Polyhedron<Dim> poly(decimal_t t) const {
    auto poly = poly_;
    for(auto& it: poly.vs_)
      it.p_ += v_ * t;
    return poly;
  }
 protected:
  Polyhedron<Dim> poly_;
  ///Obstacle velocity
	Vecf<Dim> v_{Vecf<Dim>::Zero()};
};

typedef PolyhedronObstacle<2> PolyhedronObstacle2D;

typedef PolyhedronObstacle<3> PolyhedronObstacle3D;

