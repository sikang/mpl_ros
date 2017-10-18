/**
 * @file env_base.h
 * @brief environment base class
 */

#ifndef ENV_BASE_H
#define ENV_BASE_H

#include <memory>
#include <vector>
#include <primitive/primitive.h>
#include <primitive/trajectory.h>

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
      bool goaled = (state.pos - goal_node_.pos).lpNorm<Eigen::Infinity>() < tol_dis;
      if(goaled && goal_node_.use_vel) 
        goaled = (state.vel - goal_node_.vel).lpNorm<Eigen::Infinity>() < tol_vel;
      if(goaled && goal_node_.use_acc) 
        goaled = (state.acc - goal_node_.acc).lpNorm<Eigen::Infinity>() < tol_acc;
     return goaled;
    }

    ///Heuristic function 
    double get_heur(const Waypoint& state) const
    {
      Waypoint goal_node = goal_node_;
      decimal_t t = state.t + alpha_ * dt_;
      if(!prior_traj_.segs.empty() && t < prior_traj_.getTotalTime()) {
        prior_traj_.evaluate(t, goal_node);
        goal_node.use_pos = goal_node_.use_pos;
        goal_node.use_vel = goal_node_.use_vel;
        goal_node.use_acc = goal_node_.use_acc;
        goal_node.use_jrk = goal_node_.use_jrk;
        return cal_heur(state, goal_node) + w_ * (prior_traj_.getTotalTime() - t);
      }

      return cal_heur(state, goal_node);
    }

    double cal_heur(const Waypoint& state, const Waypoint& goal) const
    {
      //return 0;
      //return w_*(state.pos - goal.pos).norm();
      //If in acceleration control space
      if(state.use_pos && state.use_vel && state.use_acc && !state.use_jrk &&
         goal.use_pos && goal.use_vel && goal.use_acc && !goal.use_jrk) {
     // if(state.use_pos && state.use_vel && state.use_acc &&
       //  goal.use_pos && goal.use_vel && goal.use_acc) {
 
        const Vec3f dp = goal.pos - state.pos;
        const Vec3f v0 = state.vel;
        const Vec3f v1 = goal.vel;
        const Vec3f a0 = state.acc;
        const Vec3f a1 = goal.acc;
        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -9*a0.dot(a0)+6*a0.dot(a1)-9*a1.dot(a1);
        decimal_t d = -144*a0.dot(v0)-96*a0.dot(v1)+96*a1.dot(v0)+144*a1.dot(v1);
        decimal_t e = 360*(a0-a1).dot(dp)-576*v0.dot(v0)-1008*v0.dot(v1)-576*v1.dot(v1);
        decimal_t f = 2880*dp.dot(v0+v1);
        decimal_t g = -3600*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar =(state.pos - goal.pos).lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);
        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          //printf("t: %f ", t);
          if(t < t_bar)
           continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost) 
            min_cost = cost;
        }
        //printf("-----------\n");
        //if(ts.empty())
          //printf("wrong! no root found!\n");
        //printf("cost: %f, t: %f. t_bar: %f\n", min_cost, t_star, t_bar);

        return min_cost;
      }

      else if(state.use_pos && state.use_vel && state.use_acc && !state.use_jrk &&
         goal.use_pos && goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vec3f dp = goal.pos - state.pos;
        const Vec3f v0 = state.vel;
        const Vec3f v1 = goal.vel;
        const Vec3f a0 = state.acc;

        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -8*a0.dot(a0);
        decimal_t d = -112*a0.dot(v0)-48*a0.dot(v1);
        decimal_t e = 240*a0.dot(dp)-384*v0.dot(v0)-432*v0.dot(v1)-144*v1.dot(v1);
        decimal_t f = dp.dot(1600*v0+960*v1);
        decimal_t g = -1600*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar =(state.pos - goal.pos).lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);
        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          //printf("t: %f ", t);
          if(t < t_bar)
            continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost)
            min_cost = cost;
          //printf("t: %f, cost: %f\n",t, cost);
        }
        return min_cost;
      }

      else if(state.use_pos && state.use_vel && state.use_acc && !state.use_jrk &&
         goal.use_pos && !goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vec3f dp = goal.pos - state.pos;
        const Vec3f v0 = state.vel;
        const Vec3f a0 = state.acc;

        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -5*a0.dot(a0);
        decimal_t d = -40*a0.dot(v0);
        decimal_t e = 60*a0.dot(dp)-60*v0.dot(v0);
        decimal_t f = 160*dp.dot(v0);
        decimal_t g = -100*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar =(state.pos - goal.pos).lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost) 
            min_cost = cost;
        }
        return min_cost;
      }


      else if(state.use_pos && state.use_vel && !state.use_acc && !state.use_jrk &&
              goal.use_pos && goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vec3f dp = goal.pos - state.pos;
        const Vec3f v0 = state.vel;
        const Vec3f v1 = goal.vel;
        decimal_t c1 = -36*dp.dot(dp);
        decimal_t c2 = 24*(v0+v1).dot(dp);
        decimal_t c3 = -4*(v0.dot(v0)+v0.dot(v1)+v1.dot(v1));
        decimal_t c4 = 0;
        decimal_t c5 = w_;

        std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
        decimal_t t_bar =(state.pos - goal.pos).lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t cost = std::numeric_limits<decimal_t>::max();
       for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
          if(c < cost)
            cost = c;
          //printf("t: %f, cost: %f\n",t, cost);
        }
        //printf("-----------\n");
        //if(ts.empty())
          //printf("wrong! no root fond!\n");

        return cost;
      }

      else if(state.use_pos && state.use_vel && !state.use_acc && !state.use_jrk &&
              goal.use_pos && !goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vec3f dp = goal.pos - state.pos;
        const Vec3f v0 = state.vel;
        decimal_t c1 = -9*dp.dot(dp);
        decimal_t c2 = 12*v0.dot(dp);
        decimal_t c3 = -3*v0.dot(v0);
        decimal_t c4 = 0;
        decimal_t c5 = w_;

        std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
        decimal_t t_bar =(state.pos - goal.pos).lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
          if(c < cost)
            cost = c;
          //printf("t: %f, cost: %f\n",t, cost);
        }
        //printf("-----------\n");
        //if(ts.empty())
          //printf("wrong! no root fond!\n");

        return cost;
      }
 
      else if(state.use_pos && !state.use_vel && !state.use_acc && !state.use_jrk &&
              goal.use_pos && !goal.use_vel && !goal.use_acc && !goal.use_jrk)
        return (w_ + 1) * (state.pos - goal.pos).norm();
      else
        return w_*(state.pos - goal.pos).norm() / v_max_;
   }

    ///Genegrate Key from state
    Key state_to_idx(const Waypoint& state) const
    {
      Vec3i pi = (state.pos/ds_).cast<int>();
      Vec3i vi = (state.vel/dv_).cast<int>();
      if(!state.use_acc )
        return std::to_string(pi(0)) + "-" + std::to_string(pi(1)) + "-" + std::to_string(pi(2)) + "-" +
          std::to_string(vi(0)) + "-" + std::to_string(vi(1)) + "-" + std::to_string(vi(2));
      else {
        Vec3i ai = (state.acc/da_).cast<int>();
        return std::to_string(pi(0)) + "-" + std::to_string(pi(1)) + "-" + std::to_string(pi(2)) + "-" +
          std::to_string(vi(0)) + "-" + std::to_string(vi(1)) + "-" + std::to_string(vi(2)) + "-" +
          std::to_string(ai(0)) + "-" + std::to_string(ai(1)) + "-" + std::to_string(ai(2));
     }
    }

    ///Recover trajectory
    void forward_action( const Waypoint& curr, int action_id, double dt, Primitive& pr) const
    {
      pr = Primitive(curr, U_[action_id], dt);
    }

    /**
     * @brief Set discretization in control space
     * @param n indicates how many samples on each semi-axes, eq: \f$ du = \frac{u_{max}}{n}\f$
     * @param use_3d if true, we also expand in z-axis
     */
    void set_discretization(int n, bool use_3d) {
      decimal_t du = u_max_ / n;

      vec_Vec3f U;
      if(use_3d) {
        for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
          for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
            for(decimal_t dz = -u_max_; dz <= u_max_; dz += u_max_ ) //here we reduce the z control
              U.push_back(Vec3f(dx, dy, dz));
      }
      else{
        for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
          for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
            U.push_back(Vec3f(dx, dy, 0));
      }
      set_U(U);
    }

    ///Set max U in each axis
    void set_U(const vec_Vec3f& U) {
      U_ = U;
    }

    ///Set max vel in each axis
    void set_v_max(decimal_t v) {
      v_max_ = v;
    }

    ///Set max acc in each axis
    void set_a_max(decimal_t a) {
      a_max_ = a;
    }

    ///Set max acc in each axis
    void set_j_max(decimal_t j) {
      j_max_ = j;
    }

    ///Set max control in each axis
    void set_u_max(decimal_t u) {
      u_max_ = u;
    }

    ///Set max amount of time step to explore 
    void set_t_max(decimal_t t) {
      t_max_ = t;
    }

    ///Set prior trajectory 
    void set_prior_trajectory(const Trajectory& traj) {
      prior_traj_ = traj;
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

    ///Set velocity tolerance for goal region
    void set_tol_acc(decimal_t acc) {
      tol_acc = acc;
    }

    ///set weight for cost in time, usually no need to change
    void set_w(decimal_t w) {
      w_ = w;
    }

    ///Set derivative order for cost in effort, dont need to set manually
    void set_wi(int wi) {
      wi_ = wi;
    }

    ///Set alpha
    void set_alpha(int alpha) {
      alpha_ = alpha;
    }

    ///Set goal state
    virtual void set_goal(const Waypoint& state) {}

    ///Print out params
    void info() {
      printf(ANSI_COLOR_YELLOW "\n");
      printf("++++++++++ PLANNER +++++++++++\n");
      printf("+    alpha: %d                 +\n", alpha_);
      printf("+       dt: %.2f               +\n", dt_);
      printf("+        w: %.2f               +\n", w_);
      printf("+       wi: %d                 +\n", wi_);
      printf("+    v_max: %.2f               +\n", v_max_);
      printf("+    a_max: %.2f               +\n", a_max_);
      printf("+    j_max: %.2f               +\n", j_max_);
      printf("+    u_max: %.2f               +\n", u_max_);
      printf("+    t_max: %.2f               +\n", t_max_);
      printf("+    U num: %zu                +\n", U_.size());
      printf("+  tol_dis: %.2f               +\n", tol_dis);
      printf("+  tol_vel: %.2f               +\n", tol_vel);
      printf("+  tol_acc: %.2f               +\n", tol_acc);
      printf("++++++++++ PLANNER +++++++++++\n");
      printf(ANSI_COLOR_RESET "\n");
    }

    ///Check if a point is in free space
    virtual bool is_free(const Vec3f& pt) const { 
      printf("Used Null is_free() for pt\n");
      return true; 
    }

    ///Check if a primitive is in free space
    virtual bool is_free(const Primitive& pr) const { 
      printf("Used Null is_free() for pr\n");
      return true; 
    }

    ///Retrieve dt
    decimal_t get_dt() {
      return dt_;
    }

    /**
     * @brief Get successor
     * @param curr The node to expand
     * @param succ The array stores valid successors
     * @param succ_idx The array stores successors' Key
     * @param succ_cost The array stores cost along valid edges
     * @param action_idx The array stores corresponding idx of control for each successor
     */
    virtual void get_succ( const Waypoint& curr, 
        std::vector<Waypoint>& succ,
        std::vector<Key>& succ_idx,
        std::vector<double>& succ_cost,
        std::vector<int>& action_idx,
       std::vector<double>& dts ) const
    {
      printf("Used Null get_succ()\n");
      succ.push_back(curr);
      succ_idx.push_back( state_to_idx(curr) );
      succ_cost.push_back(0);
      action_idx.push_back(0);
    }

    ///Flag shows that if the goal is outside map
    bool goal_outside() {
      return goal_outside_;
    }

    ///Copy polyhedra
    virtual Polyhedra polyhedra() {
      return Polyhedra();
    }

    virtual vec_Ellipsoid ellipsoids() {
      return vec_Ellipsoid();
    }

    bool goal_outside_;
    double w_ = 10; 
    ///order of derivatives for effort
    int wi_ = -1; 
    int alpha_ = 0;

    double tol_dis = 1.0;
    double tol_vel = 1.0;
    double tol_acc = 1.0;
    double u_max_;
    double v_max_ = -1;
    double a_max_ = -1;
    double j_max_ = -1;
    double t_max_ = -1;
    double dt_ = 1.0;
    double ds_ = 0.0001, dv_ = 0.001, da_ = 0.01;

    double max_jrk_diff_ = 17;
    ///Array of constant control input
    vec_Vec3f U_;
    Waypoint goal_node_;
    Trajectory prior_traj_;
};
}


#endif
