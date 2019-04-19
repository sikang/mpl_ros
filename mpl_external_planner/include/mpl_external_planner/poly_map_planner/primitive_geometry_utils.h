#include <mpl_basis/primitive.h>
#include <mpl_external_planner/poly_map_planner/simple_obstacle.h>

template <int Dim>
bool collide(const Primitive<Dim>& pr, const PolyhedronObstacle<Dim>& obs) {
  vec_E<Vec6f> cs(Dim);
  for (int i = 0; i < Dim; i++) cs[i] = pr.pr(i).coeff();

  const auto p = obs.p();
  for (const auto& v : obs.geometry().hyperplanes()) {
    const auto n = v.n_;
    decimal_t a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
    for (int i = 0; i < Dim; i++) {
      a += n(i) * cs[i](0);
      b += n(i) * cs[i](1);
      c += n(i) * cs[i](2);
      d += n(i) * cs[i](3);
      e += n(i) * cs[i](4);
      f += n(i) * cs[i](5);
    }
    a /= 120.0;
    b /= 24.0;
    c /= 6.0;
    d /= 2.0;
    e /= 1.0;
    f -= n.dot(v.p_ + p);

    std::vector<decimal_t> ts = solve(a, b, c, d, e, f);
    // printf("a, b, c, d, e: %f, %f, %f, %f, %f\n", a, b, c, d, e);
    for (const auto& it : ts) {
      if (it >= 0 && it <= pr.t()) {
        auto w = pr.evaluate(it);
        if (obs.inside(w.pos)) return true;
      }
    }
  }

  return false;
}

template <int Dim>
bool collide(const Primitive<Dim>& pr, const PolyhedronLinearObstacle<Dim>& obs,
             decimal_t t) {
  vec_E<Vec6f> cs(Dim);
  for (int i = 0; i < Dim; i++) cs[i] = pr.pr(i).coeff();

  const auto p = obs.p();
  const auto v = obs.v();
  for (const auto& hp : obs.geometry().hyperplanes()) {
    const auto n = hp.n_;
    const auto cov_v = v + obs.cov_v() * n;
    decimal_t a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
    for (int i = 0; i < Dim; i++) {
      a += n(i) * cs[i](0);
      b += n(i) * cs[i](1);
      c += n(i) * cs[i](2);
      d += n(i) * cs[i](3);
      e += n(i) * cs[i](4);
      f += n(i) * cs[i](5);
    }
    a /= 120.0;
    b /= 24.0;
    c /= 6.0;
    d /= 2.0;
    e -= n.dot(cov_v);
    f -= n.dot(hp.p_ + p + cov_v * t);

    // std::cout << "obs p: " << (obs.v_ * t + v.p_).transpose() << std::endl;
    // std::cout << "obs v: " << obs.v_.transpose() << std::endl;

    std::vector<decimal_t> ts = solve(a, b, c, d, e, f);
    // printf("a, b, c, d, e, f: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", a, b, c,
    // d, e, f);
    for (const auto& it : ts) {
      // std::cout << "t: " << it << std::endl;
      if (it >= 0 && it <= pr.t()) {
        auto w = pr.evaluate(it);
        if (obs.inside(w.pos, it + t)) return true;
      }
    }
  }

  return false;
}

template <int Dim>
bool collide(const Primitive<Dim>& pr,
             const PolyhedronNonlinearObstacle<Dim>& obs, decimal_t t) {
  vec_E<Vec6f> cs(Dim);
  for (int i = 0; i < Dim; i++) cs[i] = pr.pr(i).coeff();

  const auto traj = obs.traj();
  const decimal_t traj_t = t + obs.start_t();

  int start_id = -1;
  decimal_t T = 0.0;  // current seg start time
  const auto segs = traj.getPrimitives();
  for (size_t i = 0; i < segs.size(); i++) {
    if (traj_t >= T && traj_t < T + segs[i].t()) {
      start_id = i;
      break;
    }
    T += segs[i].t();
  }

  // printf("start_id: %d, traj_t: %f, t: %f start_t: %f\n", start_id, traj_t,
  // t, obs.start_t());

  /// if the time over the total trajectory time, use last state of traj as
  /// static obstacle
  if (start_id < 0) {
    if (traj_t <= traj.getTotalTime() && traj_t >= 0)
      return collide(pr, PolyhedronObstacle<Dim>(obs.geometry(),
                                                 traj.evaluate(traj_t).pos));
    else if (traj_t < 0 && !obs.disappear_front_)
      return collide(pr, PolyhedronObstacle<Dim>(obs.geometry(),
                                                 traj.evaluate(traj_t).pos));
    else if (traj_t > traj.getTotalTime() && !obs.disappear_back_)
      return collide(pr, PolyhedronObstacle<Dim>(obs.geometry(),
                                                 traj.evaluate(traj_t).pos));
    else
      return false;
  }

  for (size_t id = start_id; id < segs.size(); id++) {
    decimal_t t_residual = T - traj_t < 0 ? 0 : T - traj_t;
    decimal_t start_t = t_residual <= 0 ? traj_t : T;
    if (t_residual > pr.t()) break;
    const Waypoint<Dim> w = traj.evaluate(start_t);
    for (const auto& hp : obs.geometry().hyperplanes()) {
      const auto n = hp.n_;
      // std::cout << "n: " << n.transpose() << "p: " << hp.p_.transpose() <<
      // std::endl;
      decimal_t a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
      for (int i = 0; i < Dim; i++) {
        a += n(i) * cs[i](0);
        b += n(i) * cs[i](1);
        c += n(i) * cs[i](2) - n(i) * w.jrk(i);
        d += n(i) * cs[i](3) - n(i) * w.acc(i);
        e += n(i) * cs[i](4) - n(i) * w.vel(i);
        f += n(i) * cs[i](5) - n(i) * (hp.p_(i) + w.pos(i));
      }
      a /= 120;
      b /= 24;
      c /= 6;
      d /= 2;

      std::vector<decimal_t> ts = solve(a, b, c, d, e, f);
      // printf("a, b, c, d, e, f: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", a, b,
      // c, d, e, f);
      for (const auto& it : ts) {
        if (it >= t_residual && it <= pr.t() &&
            T + segs[id].t() >= it + start_t && T <= it + start_t) {
          const auto curr = pr.evaluate(it);
          if (obs.inside(curr.pos, it + t)) {
            /*
           std::cout << "collide pos: " << curr.pos.transpose() << std::endl;
           std::cout << "w p: " << w.pos.transpose() << std::endl;
           std::cout << "w v: " << w.vel.transpose() << std::endl;
           printf("it: %f, t_residual: %f, T: %f, start_t: %f\n",
                  it, t_residual, T, start_t);
                  */
            return true;
          }
        }
      }
    }

    T += segs[id].t();
  }

  return false;
}
