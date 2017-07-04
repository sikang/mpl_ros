/**
 * @file env_base.h
 * @brief environment base class
 */

#ifndef ENV_BASE_H
#define ENV_BASE_H

#include <memory>
#include <vector>
#include <primitive/primitive.h>

namespace MPL {
  
/**
 * @brief Key for node
 *
 * We use string as the Key for indexing, by default the Key refers to 'pos-vel-acc-...'
 */
typedef std::string Key; 

/**
 * @brief Base environment class
 */
class env_base
{
  public:
    ///Simple constructor
    env_base(){}
    ~env_base(){}


    ///Check if hit the goal region
    bool is_goal(const Waypoint& state) const
    {
      bool goaled = (state.pos - goal_node_.pos).norm() < tol_dis;
      if(goaled && goal_node_.use_vel)
        goaled = (state.vel - goal_node_.vel).norm() < tol_vel;
      return goaled;
    }

    ///Heuristic function 
    double get_heur(const Waypoint& state) const
    {
      //return 0;
      //return w_*(state.pos - goal_node_.pos).lpNorm<Eigen::Infinity>() / v_max_;
      const Vec3f p0 = state.pos;
      const Vec3f p1 = goal_node_.pos;
      const Vec3f v0 = state.vel;
      const Vec3f v1 = goal_node_.vel;
      decimal_t c1 = -36*(p1-p0).dot(p1-p0);
      decimal_t c2 = 24*(v0+v1).dot(p1-p0);
      decimal_t c3 = -4*(v0.dot(v0)+v0.dot(v1)+v1.dot(v1));
      decimal_t c4 = 0;
      decimal_t c5 = w_;

      std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
      decimal_t t_bar =(state.pos - goal_node_.pos).lpNorm<Eigen::Infinity>() / v_max_;
      ts.push_back(t_bar);
      decimal_t cost = 1000000;
      for(auto t: ts) {
        if(t < t_bar)
          continue;
        decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
        if(c < cost)
          cost = c;
        //printf("t: %f, cost: %f\n",t, cost);
      }
      //printf("-----------\n");
      if(ts.empty())
        printf("wrong! no root fond!\n");

      return cost;
    }

    ///Genegrate Key from state
    Key state_to_idx(const Waypoint& state) const
    {
      Vec3i pi = (state.pos/ds_).cast<int>();
      Vec3i vi = (state.vel/dv_).cast<int>();
      return std::to_string(pi(0)) + "-" + std::to_string(pi(1)) + "-" + std::to_string(pi(2)) +
        std::to_string(vi(0)) + "-" + std::to_string(vi(1)) + "-" + std::to_string(vi(2));
    }

    ///Recover trajectory
    void forward_action( const Waypoint& curr, int action_id, 
        std::vector<Waypoint>& next_micro ) const
    {
      next_micro.clear();
      if(action_id < 0) {
        next_micro.push_back(goal_node_);
      }
      else {
        Primitive p(curr, U_[action_id], dt_);
        next_micro.push_back(p.evaluate(dt_));
      }
    }

    /**
     * @brief Set discretization in control space
     * @param n indicates how many samples on each semi-axes, eq: \f$ du = \frac{u_{max}}{n}\f$
     * @param use_3d if true, we also expand in z-axis
     */
    void set_discretization(int n, bool use_3d) {
      decimal_t du = u_max_ / n;
      dv_ = std::min(0.2 * du * dt_, 0.4); //need to be small but not too much
      ds_ = 0.5 * dv_ * dt_;

      U_.clear();
      if(use_3d) {
        for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
          for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
            for(decimal_t dz = -0.5; dz <= 0.5; dz += 0.5 ) //here we reduce the z control
              U_.push_back(Vec3f(dx, dy, dz));
      }
      else{
        for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
          for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
            U_.push_back(Vec3f(dx, dy, 0));
      }
    }



    ///Set max vel in each axis
    void set_v_max(decimal_t v) {
      v_max_ = v;
    }

    ///Set max acc in each axis, also the control input as acc
    void set_a_max(decimal_t a) {
      a_max_ = a;
      u_max_ = a_max_;
    }

    ///Set dt for primitive
    void set_dt(decimal_t dt) {
      dt_ = dt;
    }

    ///Set distance tolerance for goal region
    void set_tol_dis(decimal_t dis) {
      tol_dis = dis;
    }

    ///Set velocity tolerance for goal region
    void set_tol_vel(decimal_t vel) {
      tol_vel = vel;
    }

    ///Set weight for cost in time, usually no need to change
    void set_w(decimal_t w) {
      w_ = w;
    }

    ///Set goal state
    virtual void set_goal(const Waypoint& state) {}

    ///Print out params
    void info() {
      printf(ANSI_COLOR_YELLOW "\n");
      printf("++++++++++ PLANNER +++++++++++\n");
      printf("+       dt: %.2f               +\n", dt_);
      printf("+       dv: %.2f               +\n", dv_);
      printf("+        w: %.2f               +\n", w_);
      printf("+       wi: %d                 +\n", wi_);
      printf("+    v_max: %.2f               +\n", v_max_);
      printf("+    a_max: %.2f               +\n", a_max_);
      printf("+    U num: %zu                +\n", U_.size());
      printf("+  tol_dis: %.2f               +\n", tol_dis);
      printf("+  tol_vel: %.2f               +\n", tol_vel);
      printf("++++++++++ PLANNER +++++++++++\n");
      printf(ANSI_COLOR_RESET "\n");
    }

    ///Check if a point is in free space
    virtual bool is_free(const Vec3f& pt) const { 
      printf("Used Null is_free() for pt\n");
      return false; 
    }

    ///Check if a primitive is in free space
    virtual bool is_free(const Primitive& pr) const { 
      printf("Used Null is_free() for pr\n");
      return false; 
    }

    ///Retrieve dt
    decimal_t get_dt() {
      return dt_;
    }

    ///Get successor
    virtual void get_succ( const Waypoint& curr, 
        std::vector<Waypoint>& succ,
        std::vector<Key>& succ_idx,
        std::vector<double>& succ_cost,
        std::vector<int>& action_idx ) const
    {
      printf("Used Null get_succ()\n");
      succ.push_back(curr);
      succ_idx.push_back( state_to_idx(curr) );
      succ_cost.push_back(0);
      action_idx.push_back(0);
    }

    ///Returns expanded nodes
    vec_Vec3f ps() {
      return ps_;
    }

    ///Returns expanded edges
    std::vector<Primitive> primitives() {
      return primitives_;
    }

    ///Flag shows that if the goal is outside map
    bool goal_outside() {
      return goal_outside_;
    }


    bool goal_outside_;
    double w_ = 10; 
    ///order of derivatives
    int wi_ = 1; 

    double tol_dis = 1.0;
    double tol_vel = 1.0;
    double u_max_ = 1;
    double v_max_ = 2;
    double a_max_ = 1;
    double dt_ = 1.0;
    double ds_, dv_;

    ///Array of constant control input
    vec_Vec3f U_;
    Waypoint goal_node_;

    mutable vec_Vec3f ps_;
    mutable std::vector<Primitive> primitives_;
};
}


#endif
