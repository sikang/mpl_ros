/**
 * @file trajectory.h
 * @brief Trajectory classes
 */


#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <primitive/primitive.h>

/**
 * @biref Used for scaling, ignored for most case
 */
struct VirtualPoint{
  decimal_t p;
  decimal_t v;
  decimal_t t;
};


bool VirtualPointComp (VirtualPoint i, VirtualPoint j);

class LambdaSeg {
  public:
    LambdaSeg() {}
    LambdaSeg(const VirtualPoint& v1, const VirtualPoint& v2);
    VirtualPoint evaluate(decimal_t t) const;
    decimal_t getT(decimal_t t) const;
    Vec4f a; // a3, a2, a1, a0
    decimal_t ti;
    decimal_t tf;

    decimal_t dT;
};

class Lambda {
  public:
    Lambda() {}
    Lambda(const std::vector<VirtualPoint>& vs);

    bool exist() const;
    std::vector<VirtualPoint> sample(int N);
    vec_Vec3f sampleT(int N);
    VirtualPoint evaluate(decimal_t t) const;

    decimal_t getT(decimal_t tau) const;
    decimal_t getTau(decimal_t t) const;
    decimal_t getTotalTime() const;
    std::vector<LambdaSeg> segs;
};


/**
 * @brief Trajectory class
 *
 * A trajectory is composed from several primitives, so-called linear piece-wise polynomials
 */
class Trajectory {
  public:
    /**
     * @brief Empty constructor
     */
    Trajectory() {}
    /**
     * @brief Construct from multiple primitives
     */
    Trajectory(const std::vector<Primitive>& trajs);
    /**
     * @brief Return the total duration of the trajectory
     */
    decimal_t getTotalTime() const;
    /**
     * @brief Retrieve scaling factor
     */
    Lambda lambda() const;
    /**
     * @brief Evaluate state at t, return false if fails to evaluate
     *
     * The failure cases involve: 1) t is out of scope; 2) lambda is ill-posed such that \f$t = \lambda(\tau)^{-1}\f$ has no solution
     */ 
    bool evaluate(decimal_t t, Waypoint &p) const;
    /**
     * @brief Scale according to ratio at start and end (velocity only)
     */
    bool scale(decimal_t ri, decimal_t rf);
    /**
     * @brief Scale down the whole trajectory according to mv
     */
    bool scale_down(decimal_t mv, decimal_t ri, decimal_t rf);
    /**
     * @brief Sample N states using uniformed time
     */
    std::vector<Waypoint> sample(int N) const;
    /**
     * @brief Return total efforts of primitive for the given duration: \f$J(i) = \int_0^t |p^{(i+1)}(t)|^2dt\f$
     *
     * Return J is the summation of efforts in all three dimensions
     * @param i effort is defined as (i+1)-th derivative of polynomial
     */
    decimal_t J(int i) const;
    ///Get time for each segment
    std::vector<decimal_t> getSegsT() const;

    ///Segments of primitives
    std::vector<Primitive> segs;
    ///Time in virtual domain
    std::vector<decimal_t> taus;
    ///Time in actual domain
    std::vector<decimal_t> Ts;
    ///Total time of the trajectory
    decimal_t total_t_;
    ///Scaling object
    Lambda lambda_;
};

#endif
