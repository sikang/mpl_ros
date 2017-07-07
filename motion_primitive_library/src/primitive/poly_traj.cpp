#include <primitive/poly_traj.h>

PolyTraj::PolyTraj() {}

Waypoint PolyTraj::evaluate(decimal_t time) const {
  if (time < 0)
    time = 0;
  int cur_idx = -1;
  for (unsigned int i = 1; i < waypoint_times_.size(); i++) {
    if (time <= waypoint_times_[i]) {
      cur_idx = i - 1;
      break;
    }
  }
  if (cur_idx == -1)
    time = waypoint_times_.back();

  const decimal_t t_traj = time - waypoint_times_[cur_idx];
  const MatD3f &p = coefficients_[cur_idx];
  Waypoint waypoint;
  for (int derr = 0; derr < 3; derr++ ){
    Vec3f v3 = Vec3f::Zero();
    for (unsigned int i = derr; i < p.rows(); i++) {
      unsigned int c = 1;
      for (int j = 0; j < derr; j++)
        c *= (i - j);
      v3 += p.row(i).transpose() * (c * std::pow(t_traj, i - derr));
    }
    if(derr == 0)
      waypoint.pos = v3;
    else if(derr == 1)
      waypoint.vel = v3;
    else if(derr == 2)
      waypoint.acc = v3;
  }

  return waypoint;
}

decimal_t PolyTraj::getTotalTime() const { return waypoint_times_.back(); }

void PolyTraj::clear() {
  waypoint_times_.clear();
  coefficients_.clear();
}

void PolyTraj::addCoeff(const MatD3f &p) {
  coefficients_.push_back(p);
}

void PolyTraj::addTime(const std::vector<decimal_t> &dts) {
  waypoint_times_.clear();
  waypoint_times_.push_back(0);
  for (auto t : dts)
    waypoint_times_.push_back(waypoint_times_.back() + t);
  dts_ = dts;
}

std::vector<Primitive> PolyTraj::toPrimitives() {
  std::vector<Primitive> trajs;
  trajs.resize(coefficients_.size());
  for (unsigned int i = 0; i < coefficients_.size(); i++) {
    vec_E<Vec6f> coeffs;
    const MatD3f &p = coefficients_[i];
    for (unsigned int j = 0; j < p.cols(); j++) {
      Vec6f coeff;
      for (unsigned int k = 0; k < p.rows(); k++)
        coeff(k) = p(k, j) * factorial(k);
      coeffs.push_back(coeff.reverse());
    }

    trajs[i] = Primitive(coeffs, dts_[i]);
  }
  return trajs;
}

inline int PolyTraj::factorial(int n) {
  if(n <= 1)
    return 1;
  return n*factorial(n-1);
}

