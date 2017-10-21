/**
 * @file primitive.h
 * @brief Primitive classes
 */

#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include <memory>
#include <motion_primitive_library/data_type.h>
#include "math.h"

/**
 * @brief Node in graph
 *
 * State includes position, velocity and acceleration in \f$R^3\f$
 */
struct Waypoint {
  Vec3f pos; 
  Vec3f vel;
  Vec3f acc;
  Vec3f jrk;

  bool use_pos = false;///<If true, attribute pos will be used in primitive generation
  bool use_vel = false;///<If true, attribute vel will be used in primitive generation 
  bool use_acc = false;///<If true, attribute acc will be used in primitive generation 
  bool use_jrk = false;///<If true, attribute jrk will be used in primitive generation 

  decimal_t t = 0;
  std::shared_ptr<Waypoint> parent;
  ///Print all the useful attributes
  void print() const {
    std::cout << "t: " << t << std::endl;
   if(use_pos)
      std::cout << "pos: " << pos.transpose() << std::endl;
    if(use_vel)
      std::cout << "vel: " << vel.transpose() << std::endl;
    if(use_acc)
      std::cout << "acc: " << acc.transpose() << std::endl;
    if(use_jrk)
      std::cout << "jrk: " << jrk.transpose() << std::endl;
    if(!use_pos && !use_vel && !use_acc && !use_jrk)
      std::cout << "Nothing is used!" << std::endl;
  }

  ///Check if two waypoints are equivalent
  bool operator==(const Waypoint& n){
    return this->pos == n.pos &&
      this->vel == n.vel &&
      this->acc == n.acc &&
      this->jrk == n.jrk &&
      this->t == n.t;
  }
};

/**
 * @brief Primitive1D class
 *
 * Assume the 1D primitive is the n-th order polynomial with n = 5 as 
 * \f$p(t) = \frac{c(0)}{120}t^5+\frac{c(1)}{24}t^4+\frac{c(2)}{6}t^3+\frac{c(3)}{2}t^2+c(4)t+c(5) = 0\f$
 */
class Primitive1D {
  public:
    /**
     *@brief Empty constructor
     */
    Primitive1D();
    /**
     * @brief Construct from known coefficients
     * @param coeff[0] is the coefficient of the highest order
     */
    Primitive1D(const Vec6f& coeff);
    /**
     * @brief Construct 1D primitive from an initial state (p) and an input control (u)
     */
    Primitive1D(decimal_t p, decimal_t u);
    /**
     * @brief Construct 1D primitive from an initial state (p, v) and an input control (u)
     */
    Primitive1D(Vec2f state, decimal_t u);
    /**
     * @brief Construct 1D primitive from an initial state (p, v, a) and an input control (u)
     */
    Primitive1D(Vec3f state, decimal_t u);
    /**
     * @brief Construct 1D primitive from an initial state (p, v, a, j) and an input control (u)
     */
    Primitive1D(Vec4f state, decimal_t u);
    /**
    * @brief Construct 1D primitive from an initial state (p1) to a goal state (p2), given duration t
     */
    Primitive1D(decimal_t p1, decimal_t p2,  decimal_t t);
    /**
     * @brief Construct 1D primitive from an initial state (p1, v1) to a goal state (p2, v2), given duration t 
     */
    Primitive1D(decimal_t p1, decimal_t v1, decimal_t p2, decimal_t v2, decimal_t t);
    /**
     * @brief Construct 1D primitive from an initial state (p1, v1, a1) to a goal state (p2, v2, a2), given duration t
     */
    Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1, decimal_t p2, decimal_t v2, decimal_t a2, decimal_t t);
    /**
     * @brief Return total efforts of 1D primitive for the given duration: \f$J(t, i) = \int_0^t |p^{(i+1)}(t)|^2dt\f$
     * @param t assume the duration is from 0 to t
     * @param i effort is defined as (i+1)-th derivative of polynomial
     */
    decimal_t J(decimal_t t, int i) const;
    /**
     * @brief Return coffecients
     */
    Vec6f coeff() const;
    /** 
     * @brief Return (p, v, a) at t, deault v, a are zeros
     */
    Vec4f evaluate(decimal_t t) const;
    /**
     * @brief Return extrema of velocity, velocities at both ends (0, t) are considered
     */
    std::vector<decimal_t> extrema_vel(decimal_t t) const;
    /**
     * @brief Return extrema of acceleration, accelerations at both ends (0, t) are considered
     */
    std::vector<decimal_t> extrema_acc(decimal_t t) const;
    /**
     * @brief Return extrema of jerk, jerk at both ends (0, t) are considered
     */
    std::vector<decimal_t> extrema_jrk(decimal_t t) const;
 
  private:
    /**@brief Coefficients*/
    Vec6f c;
};


/** 
 * @brief Primitive class
 *
 * Contains three 1D primitives corresponding to x, y, z indivisually.
 */
class Primitive {
 public:
  /**
   * @brief Empty constructor
   */
  Primitive();
  /**
   * @brief Construct from an initial state p and an input control u for a given duration t
   */
  Primitive(const Waypoint& p, const Vec3f& u, decimal_t t);
  /**
   * @brief Construct from an initial state p1 and a goal state p2 for a given duration t
   */
  Primitive(const Waypoint& p1, const Waypoint& p2, decimal_t t);
  /**
   * @brief Construct from given coefficients and duration
   */
  Primitive(const vec_E<Vec6f>& cs, decimal_t t);
  /**
   * @brief Return state at t
   *
   * Note: no flag in the returned waypoint is set
   */
  Waypoint evaluate(decimal_t t) const;
  /** 
   * @brief Return pre-defined duration
   */
  decimal_t t() const;
  /**
   * @brief Retrieve the 1D primitive
   * @param k indicates the retrieved dimension: 0-x, 1-y, 2-z
   */
  Primitive1D traj(int k) const;
  /**
   * @brief Return max velocity along k-th dimension
   */
  decimal_t max_vel(int k) const;
  /**
   * @brief Return max accleration along k-th dimension
   */
  decimal_t max_acc(int k) const;
  /**
   * @brief Return max jerk along k-th dimension
   */
  decimal_t max_jrk(int k) const;
  /**
   * @brief Check if the max velocity is below the threshold
   * @param mv is the max threshold for velocity
   */
  bool valid_vel(decimal_t mv) const;
  /**
   * @brief Check if the max acceleration is below the threshold
   * @param ma is the max threshold for acceleration
   */
  bool valid_acc(decimal_t ma) const;
  /**
   * @brief Check if the max jerk is below the threshold
   * @param mj is the max threshold for jerk
   */
  bool valid_jrk(decimal_t mj) const;
 
  /**
   * @brief Return total efforts of primitive for the given duration: \f$J(i) = \int_0^t |p^{(i+1)}(t)|^2dt\f$
   *
   * Return J is the summation of efforts in all three dimensions
   * @param i effort is defined as (i+1)-th derivative of polynomial
   */
  decimal_t J(int i) const;
  /**
   * @brief Sample N states using uniformed time
   */
  std::vector<Waypoint> sample(int N) const;
  /**
   * @brief Retrieve coefficients
   */
  vec_E<Vec6f> coeffs() const;
 protected:
  ///Duration
  decimal_t t_;
  ///By default, primitive class contains three 1D primitive
  Primitive1D trajs_[3];
};


#endif
