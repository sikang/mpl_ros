#include <primitive/primitive.h>

//******* Primitive 1D Base Class ***********
Primitive1D::Primitive1D() {}

Primitive1D::Primitive1D(const Vec6f& coeff) : c(coeff) {}

Primitive1D::Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1,
                         decimal_t p2, decimal_t v2, decimal_t a2, decimal_t t) {
  Mat6f A;
  A << 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 1, 0, 0,
    t*t*t*t*t/120, t*t*t*t/24, t*t*t/6,t*t/2, t, 1,
    t*t*t*t/24, t*t*t/6, t*t/2, t, 1, 0,
    t*t*t/6, t*t/2, t, 1, 0, 0;
  Vec6f b;
  b << p1, v1, a1, p2, v2, a2;
  c = A.inverse() * b;
}


Primitive1D::Primitive1D(decimal_t p1, decimal_t v1,
                         decimal_t p2, decimal_t v2, decimal_t t) {
  Mat4f A;
  A << 0, 0, 0, 1,
    0, 0, 1, 0,
    t*t*t/6,t*t/2, t, 1,
    t*t/2, t, 1, 0;
  Vec4f b;
  b << p1, v1, p2, v2;
  Vec4f cc = A.inverse() * b;
  c << 0, 0, cc(0), cc(1), cc(2), cc(3);
}

Vec3f Primitive1D::evaluate(decimal_t t) const {
  return Vec3f(c(0)/120*t*t*t*t*t+c(1)/24*t*t*t*t+c(2)/6*t*t*t+c(3)/2*t*t+c(4)*t+c(5),
               c(0)/24*t*t*t*t+c(1)/6*t*t*t+c(2)/2*t*t+c(3)*t+c(4),
               c(0)/6*t*t*t+c(1)/2*t*t+c(2)*t+c(3));
}

decimal_t Primitive1D::J(decimal_t t, int i) const {
  // i = 1, return integration of square of acc
  if(i == 1)
    return c(0)*c(0)/252*t*t*t*t*t*t*t+c(0)*c(1)/36*t*t*t*t*t*t+(c(1)*c(1)/20+c(0)*c(2)/15)*t*t*t*t*t+
      (c(0)*c(3)/12+c(1)*c(2)/4)*t*t*t*t+(c(2)*c(2)/3+c(1)*c(3)/3)*t*t*t+
      c(2)*c(3)*t*t+c(3)*c(3)*t;
  // i = 2, return integration of square of jerk
  else if(i == 2)
    return c(2)*c(2)*t+c(1)*c(2)*t*t+(c(1)*c(1)+c(0)*c(2))/3.*t*t*t+
      c(0)*c(1)/4.*t*t*t*t+c(0)*c(0)/20.*t*t*t*t*t;
  // i = 3, return integration of square of snap
  else if(i == 3)
    return c(0)*c(0)/3.*t*t*t+c(0)*c(1)*t*t+c(1)*c(1)*t;
  else
    return 0;
}

Vec6f Primitive1D::coeff() const {
  return c;
}

//********** Primitive 1D Vel Class ***********
Primitive1D::Primitive1D(decimal_t p1, decimal_t v1, decimal_t u) {
  c << 0, 0, 0, u, v1, p1;
}



//********** Primitive 1D Acc Class ***********
Primitive1D::Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1, decimal_t u) {
  c << 0, 0, u, a1, v1, p1;
}

std::vector<decimal_t> Primitive1D::extrema_vel(decimal_t t) const {
  std::vector<decimal_t> ts = solve(0, c(0)/6, c(1)/2, c(2), c(3));
  std::vector<decimal_t> ts_max;
  for(const auto &it: ts) {
    if(it > 0 && it < t)
      ts_max.push_back(it);
  }
  return ts_max;
}

std::vector<decimal_t> Primitive1D::extrema_acc(decimal_t t) const {
  std::vector<decimal_t> ts = solve(0, 0, c(0)/2, c(1), c(2));
  std::vector<decimal_t> ts_max;
  for(const auto &it: ts) {
    if(it > 0 && it < t)
      ts_max.push_back(it);
  }
  return ts_max;
}



//********** Primitive Main Class *************
Primitive::Primitive() {}

Primitive::Primitive(const vec_E<Vec6f>& cs, decimal_t t) :
    t_(t)
{
  // Constructor from coeffs
  for(int i = 0; i < 3; i++)
    trajs_[i] = Primitive1D(cs[i]);
}


Primitive::Primitive(const Waypoint& p, const Vec3f& u, decimal_t t) :
    t_(t)
{
  if(p.use_acc) {
    for(int i = 0; i < 3; i++)
      trajs_[i] = Primitive1D(p.pos(i), p.vel(i), p.acc(i), u(i));
  }
  else if(p.use_vel) {
    for(int i = 0; i < 3; i++)
      trajs_[i] = Primitive1D(p.pos(i), p.vel(i), u(i));
  }
  else
    printf("Null Primitive!\n");
}

Primitive::Primitive(const Waypoint& p1, const Waypoint& p2, decimal_t t) :
    t_(t)
{
  // Constructor from Two Waypoints
  // Fully contrained
  if(p1.use_pos && p1.use_vel && p1.use_acc &&
     p2.use_pos && p2.use_vel && p2.use_acc) {
    for(int i = 0; i < 3; i++)
      trajs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i),
                              p2.pos(i), p2.vel(i), p2.acc(i), t_);
  }
  // Use vel only
  else if(p1.use_pos && p1.use_vel && !p1.use_acc &&
          p2.use_pos && p2.use_vel && !p2.use_acc) {
    for(int i = 0; i < 3; i++)
      trajs_[i] = Primitive1D(p1.pos(i), p1.vel(i),
                              p2.pos(i), p2.vel(i), t_);
  }
  // Null
  else {
    printf("Null Primitive!\n");
    p1.print();
    p2.print();
  }
}



decimal_t Primitive::max_vel(int k) const {
  std::vector<decimal_t> ts = trajs_[k].extrema_vel(t_);
  Vec3f p1 = trajs_[k].evaluate(0);
  Vec3f p2 = trajs_[k].evaluate(t_);
  decimal_t max_v = std::max(fabs(p1(1)), fabs(p2(1)));

  for(const auto &it: ts) {
    if(it > 0 && it < t_){
      Vec3f p3 = trajs_[k].evaluate(it);
      if(fabs(p3(1)) > max_v)
        max_v = fabs(p3(1));
    }
  }
  return max_v;
}

decimal_t Primitive::max_acc(int k) const {
  std::vector<decimal_t> ts = trajs_[k].extrema_acc(t_);
  Vec3f p1 = trajs_[k].evaluate(0);
  Vec3f p2 = trajs_[k].evaluate(t_);
  decimal_t max_a = std::max(fabs(p1(2)), fabs(p2(2)));

  for(const auto &it: ts) {
    if(it > 0 && it < t_){
      Vec3f p3 = trajs_[k].evaluate(it);
      if(fabs(p3(2)) > max_a)
        max_a = fabs(p3(2));
    }
  }

  return max_a;
}

bool Primitive::valid_vel(decimal_t mv) const {
  // ignore negative threshold
  if(mv < 0)
    return true;
  // check if max vel is violating the constraint
  for(int i = 0; i < 3; i++) {
    if(max_vel(i) > mv)
      return false;
  }
  return true;
}

bool Primitive::valid_acc(decimal_t ma) const {
  // ignore negative threshold
  if(ma < 0)
    return true;
  // check if max acc is violating the constraint
  for(int i = 0; i < 3; i++) {
    if(max_acc(i) > ma)
      return false;
  }
  return true;
}



Waypoint Primitive::evaluate(decimal_t t) const {
  Waypoint p;
  for(int j = 0; j < 3; j++) {
    Vec3f d = trajs_[j].evaluate(t);
    p.pos(j) = d(0);
    p.vel(j) = d(1);
    p.acc(j) = d(2);
  }
  return p;
}


std::vector<Waypoint> Primitive::sample(int N) const {
  std::vector<Waypoint> ps;
  decimal_t dt = t_ / N;
  for(decimal_t t = 0; t <= t_; t+= dt)
      ps.push_back(evaluate(t));
  return ps;
}

Primitive1D Primitive::traj(int k) const { return trajs_[k]; }

decimal_t Primitive::t() const { return t_; }

decimal_t Primitive::J(int i) const {
  return trajs_[0].J(t_, i)+trajs_[1].J(t_, i)+trajs_[2].J(t_, i);
}


vec_E<Vec6f> Primitive::coeffs() const {
  vec_E<Vec6f> cs;
  cs.push_back(trajs_[0].coeff());
  cs.push_back(trajs_[1].coeff());
  cs.push_back(trajs_[2].coeff());

  return cs;
}
