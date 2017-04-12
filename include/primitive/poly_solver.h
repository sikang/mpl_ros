#ifndef POLY_SOLVER_H
#define POLY_SOLVER_H

#include <stdio.h>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <primitive/poly_traj.h>

class PolySolver {
public:
  PolySolver(unsigned int smooth_derivative_order, // The max derivative we want
                                                   // continuous
             unsigned int minimize_derivative);    // The derivative to minimize

  bool solve(const std::vector<Waypoint>& waypoints, 
             const std::vector<decimal_t> &dts);

  std::shared_ptr<PolyTraj> getTrajectory();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  unsigned int N_;
  unsigned int R_;
  bool debug_;
  std::shared_ptr<PolyTraj> ptraj_;
};
#endif
