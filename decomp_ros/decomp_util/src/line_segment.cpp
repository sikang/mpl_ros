#include <decomp_util/ellipse_decomp.h>

LineSegment::LineSegment(const Vec3f &p1, const Vec3f &p2):
  p1_(p1), p2_(p2) {}


void LineSegment::set_virtual_dim(decimal_t x, decimal_t y, decimal_t z){
  virtual_x_ = x;
  virtual_y_ = y;
  virtual_z_ = z;
}

void LineSegment::set_obstacles(const vec_Vec3f& obs) {
  Polyhedron vs;
  add_virtual_wall(vs);
  obs_ = ps_in_polytope(vs, obs);
}

Ellipsoid LineSegment::find_ellipsoid(const Vec3f& p1, const Vec3f& p2){
  const decimal_t f = (p1 - p2).norm() / 2;
  Mat3f C = f * Mat3f::Identity();
  Vec3f axes(C(0, 0), C(1, 1), C(2, 2));

  const Quatf qi = vec_to_quaternion(p2 - p1);
  C = qi * C * qi.conjugate();

  Ellipsoid E = std::make_pair(C, (p1 + p2) / 2);
  Vec3f pw = Vec3f::Zero();
  Quatf qf = qi;

  vec_Vec3f Os = ps_in_ellipsoid(E, obs_);
  Vec3f prev_pw;
  bool removed = false;
  //**** decide short axes
  while (inside_ellipsoid(E, Os)) {
    int id = -1;
    closest_pt(E, Os, pw, id);
    Vec3f p = qi.inverse() * (pw - E.second); // to ellipse frame
    const decimal_t roll = atan2(p(2), p(1));
    qf = qi * Quatf(cos(roll / 2), sin(roll / 2), 0, 0);
    p = qf.inverse() * (pw - E.second);
    axes(1) = fabs(p(1)) / sqrt(1 - pow(p(0) / axes(0), 2));
    E.first = Mat3f::Identity();
    E.first(0, 0) = axes(0);
    E.first(1, 1) = axes(1);
    E.first(2, 2) = axes(1);
    E.first = qf * E.first * qf.conjugate();

    Os.erase(Os.begin() + id); // remove pw
    if(removed)
      Os.push_back(prev_pw);
    removed = true;
    prev_pw = pw;

  }

  //**** reset ellipsoid with old axes(2)
  E.first = f * Mat3f::Identity();
  E.first(0, 0) = axes(0);
  E.first(1, 1) = axes(1);
  E.first(2, 2) = axes(2);
  E.first = qf * E.first * qf.conjugate();
  Os = ps_in_ellipsoid(E, Os);

  removed = false;
  while (inside_ellipsoid(E, Os)) {
    int id = -1;
    closest_pt(E, Os, pw, id);

    Vec3f p = qf.inverse() * (pw - E.second);
    axes(2) =
      fabs(p(2)) / sqrt(1 - pow(p(0) / axes(0), 2) - pow(p(1) / axes(1), 2));
    E.first = Mat3f::Identity();
    E.first(0, 0) = axes(0);
    E.first(1, 1) = axes(1);
    E.first(2, 2) = axes(2);
    E.first = qf * E.first * qf.conjugate();

    Os.erase(Os.begin() + id); // remove pw
    if(removed)
      Os.push_back(prev_pw);
    removed = true;
    prev_pw = pw;
  }

  return E;
}

Polyhedron LineSegment::find_polyhedron(const Ellipsoid& E){
  //**** find half-space
  Polyhedron Vs;
  vec_Vec3f O_remain = obs_;
  while (!O_remain.empty()) {
    Face v = closest_obstacle(E, O_remain);
    adjust(v);
    Vs.push_back(v);
    Vec3f a = v.n;
    decimal_t b = v.p.dot(a);
    vec_Vec3f O_tmp;
    for (const auto &it : O_remain) {
      if (a.dot(it) - b < 0)
        O_tmp.push_back(it);
    }
    O_remain = O_tmp;
    /*
    std::cout << "a: " << a.transpose() << std::endl;
    std::cout << "b: " << b << std::endl;
    */
  }

  return Vs;
}

void LineSegment::adjust(Face& v){
  decimal_t d1 = fabs(v.n.dot(p1_ - v.p));
  decimal_t d2 = fabs(v.n.dot(p2_ - v.p));
  decimal_t d = std::min(d1, d2);
  if(d >= robot_radius_ )
    return;

  Vec3f p, pp;
  if(d == d1){
    p = p1_;
    pp = p2_;
  }
  else{
    p = p2_;
    pp = p1_;
  }

  const Vec3f pv = v.p;
  const Vec3f n1 = (pv - p).normalized();
  const Vec3f n3 = (v.n.cross(n1)).normalized();
  const Vec3f n2 = (n3.cross(n1)).normalized();

  Aff3f TFbw;
  TFbw.matrix() << n1(0), n2(0), n3(0), p(0),
    n1(1), n2(1), n3(1), p(1),
    n1(2), n2(2), n3(2), p(2),
    0, 0, 0, 1;

  const decimal_t r = (pv - p).norm();
  const decimal_t r1 = robot_radius_;
  if(1 && r < r1){
    printf(ANSI_COLOR_GREEN "r: %f, r1: %f\n" ANSI_COLOR_RESET, r, r1);
    return;
  }
  const decimal_t r2 = sqrt(r*r - r1*r1);
  const decimal_t theta = atan2(r2, r1);
  const Vec3f new_p1_b(r1 * cos(theta), r1 * sin(theta), 0);
  const Vec3f new_p2_b(r1 * cos(-theta), r1 * sin(-theta), 0);
  const Vec3f new_p1 = TFbw * new_p1_b;
  const Vec3f new_p2 = TFbw * new_p2_b;

  const Vec3f new_n1 = (new_p1-p).normalized();
  const Vec3f new_n2 = (new_p2-p).normalized();

  d1 = -new_n1.dot(pp - v.p);
  d2 = -new_n2.dot(pp - v.p);

  if(d1 > robot_radius_)
    v.n = new_n1;
  else if(d2 > robot_radius_)
    v.n = new_n2;
}

void LineSegment::add_virtual_wall(Polyhedron &Vs) {
  //**** virtual walls parallel to path p1->p2
  Vec3f dir = p2_ - p1_;
  dir /= dir.norm();
  Vec3f dir_h(dir(1), -dir(0), 0);
  if (dir_h == Vec3f::Zero())
    dir_h << -1, 0, 0;
  Vec3f pp1 = p1_ + dir_h * virtual_y_;
  Vec3f pp2 = p1_ - dir_h * virtual_y_;
  Vs.push_back(Face(pp1, dir_h));
  Vs.push_back(Face(pp2, -dir_h));

  Vec3f dir_v = dir.cross(dir_h);
  Vec3f pp3 = p1_ + dir_v * virtual_z_;
  Vec3f pp4 = p1_ - dir_v * virtual_z_;
  Vs.push_back(Face(pp3, dir_v));
  Vs.push_back(Face(pp4, -dir_v));

  Vec3f pp5 = p2_ + dir * virtual_x_;
  Vec3f pp6 = p1_ - dir * virtual_x_;
  Vs.push_back(Face(pp5, dir));
  Vs.push_back(Face(pp6, -dir));
}

void LineSegment::dilate(decimal_t radius) {
  robot_radius_ = radius;
  ellipsoid_ = find_ellipsoid(p1_, p2_);
  polyhedron_ = find_polyhedron(ellipsoid_);
  /*
     decimal_t d1 = cal_closest_dist(p1_, polyhedron_);
     decimal_t d2 = cal_closest_dist(p2_, polyhedron_);
     printf("v1: d1: %f, d2: %f\n", d1, d2);
     */

  add_virtual_wall(polyhedron_);
}


decimal_t LineSegment::ellipsoid_volume() {
  return ellipsoid_.first.determinant();
}

decimal_t LineSegment::polyhedron_volume() {
  vec_E<vec_Vec3f> bs = cal_extreme_points(polyhedron_);
  return cal_volume(bs, (p1_ + p2_)/2);
}


void LineSegment::shrink(const Vec3f& p1, const Vec3f& p2, decimal_t thr) {
  for (auto &it : polyhedron_) {
    decimal_t b = it.p.dot(it.n);
    decimal_t d1 = it.n.dot(p1) - b;
    decimal_t d2 = it.n.dot(p2) - b;
    decimal_t d = -std::max(d1, d2) - 0.1;
    d = d < thr ? d : thr;
    if (d > 0.0)
      it.p -= d * it.n;
  }
}
