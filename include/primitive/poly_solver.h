/**
 * @file poly_solver.h
 * @brief Trajectory generator back-end
 */
#ifndef POLY_SOLVER_H
#define POLY_SOLVER_H

#include <stdio.h>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <primitive/poly_traj.h>

/**
 * @brief Trajectory generator back-end class
 *
 * Given intermediate waypoints and associated time allocation, generate the n-th order polynomials
 */
class PolySolver {
  public:
    /**
     * @brief Simple constructor
     * @param smooth_derivative_order The max derivative we want continuous
     * @param minimize_derivative The derivative to minimize
     */
    PolySolver(unsigned int smooth_derivative_order, 
        unsigned int minimize_derivative);
    /**
     * @brief Solve the trajector as defined in constructor
     * @param waypoints Intermediate waypoints that the trajectory pass through
     * @param dts Time allocation for each segment
     *
     * Note that the element in dts is the time for that segment
     */
    bool solve(const std::vector<Waypoint>& waypoints, 
        const std::vector<decimal_t> &dts);

    ///Get the solved trajectory
    std::shared_ptr<PolyTraj> getTrajectory();

  private:
    unsigned int N_;
    unsigned int R_;
    bool debug_;
    std::shared_ptr<PolyTraj> ptraj_;
};
#endif
