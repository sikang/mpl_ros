#ifndef POLY_TRAJ_H
#define POLY_TRAJ_H

#include <primitive/primitive.h>
#include <deque>

class PolyTraj {
public:
  PolyTraj();
  void clear();
  void addCoeff(const MatD3f &p);
  void addTime(const std::vector<decimal_t> &dts);
  std::vector<Primitive> toPrimitives();

  Waypoint evaluate(decimal_t t) const;
  decimal_t getTotalTime() const;
  decimal_t getCost() { return 0; }

private:
  inline int factorial(int n);
  std::vector<decimal_t> waypoint_times_;
  std::vector<decimal_t> dts_;
  std::deque<MatD3f, Eigen::aligned_allocator<MatD3f>>
      coefficients_;
};
#endif
