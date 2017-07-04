/**
 * @file poly_traj.h
 * @brief Trajectory object for PolySolver
 */
#ifndef POLY_TRAJ_H
#define POLY_TRAJ_H

#include <primitive/primitive.h>
#include <deque>

/**
 * @brief Trajectory class for solving n-th polynomial with PolySolver
 */
class PolyTraj {
public:
  ///Simple constructor
  PolyTraj();
  ///Clear
  void clear();
  ///Set coefficients
  void addCoeff(const MatD3f &p);
  ///Set time allocation
  void addTime(const std::vector<decimal_t> &dts);
  ///Convert to Primitive class
  std::vector<Primitive> toPrimitives();

  ///Evaluate the waypoint at t
  Waypoint evaluate(decimal_t t) const;
  ///Get the total time for the trajectory
  decimal_t getTotalTime() const;

private:
  inline int factorial(int n);
  std::vector<decimal_t> waypoint_times_;
  std::vector<decimal_t> dts_;
  std::deque<MatD3f, Eigen::aligned_allocator<MatD3f>>
      coefficients_;
};
#endif
