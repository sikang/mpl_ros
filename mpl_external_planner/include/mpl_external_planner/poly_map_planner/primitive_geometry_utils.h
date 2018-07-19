#include <mpl_basis/primitive.h>
#include <mpl_external_planner/poly_map_planner/simple_obstacle.h>

template <int Dim>
bool collide(const Primitive<Dim>& pr, const PolyhedronObstacle<Dim>& poly) {
  vec_E<Vec6f> cs(Dim);
	for(int i = 0; i < Dim; i++)
		cs[i] = pr.pr(i).coeff();

  for(const auto& v: poly.hyperplanes()){
    const auto n = v.n_;
    decimal_t a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
    for(int i = 0; i < Dim; i++) {
      a += n(i)*cs[i](0);
      b += n(i)*cs[i](1);
      c += n(i)*cs[i](2);
      d += n(i)*cs[i](3);
      e += n(i)*cs[i](4);
      f += n(i)*cs[i](5);
    }
    a /= 120;
    b /= 24;
    c /= 6;
    d /= 2;
    e /= 1;
    f -= n.dot(v.p_);

    std::vector<decimal_t> ts = solve(a, b, c, d, e, f);
    //printf("a, b, c, d, e: %f, %f, %f, %f, %f\n", a, b, c, d, e);
    for(const auto& it: ts) {
      if(it >= 0 && it <= pr.t()) {
        auto w = pr.evaluate(it);
        if(poly.inside(w.pos))
          return true;
      }
    }
  }

  return false;
}


template <int Dim>
bool collide(const Primitive<Dim>& pr, const PolyhedronObstacle<Dim>& poly, decimal_t t) {
  vec_E<Vec6f> cs(Dim);
	for(int i = 0; i < Dim; i++)
		cs[i] = pr.pr(i).coeff();

  for(const auto& v: poly.hyperplanes()){
    const auto n = v.n_;
    decimal_t a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
    for(int i = 0; i < Dim; i++) {
      a += n(i)*cs[i](0);
      b += n(i)*cs[i](1);
      c += n(i)*cs[i](2);
      d += n(i)*cs[i](3);
      e += n(i)*cs[i](4) - n(i)*poly.v_(i);
      f += n(i)*cs[i](5);
    }
    a /= 120;
    b /= 24;
    c /= 6;
    d /= 2;
    e /= 1;
    f -= n.dot(v.p_+poly.v_*t);

    std::vector<decimal_t> ts = solve(a, b, c, d, e, f);
    //printf("a, b, c, d, e: %f, %f, %f, %f, %f\n", a, b, c, d, e);
    for(const auto& it: ts) {
      if(it >= 0 && it <= pr.t()) {
        auto w = pr.evaluate(it);
        if(poly.inside(w.pos))
          return true;
      }
    }
  }

  return false;
}
