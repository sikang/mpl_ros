#ifndef ENV_INT_H_
#define ENV_INT_H_

#include <memory>
#include <vector>
#include <primitive/primitive.h>

namespace MPL {
typedef std::string Key;

class env_base
{
  public:
    env_base(){}
    ~env_base(){}

    vec_Vec3f ps() {
      return ps_;
    }

    std::vector<Primitive> primitives() {
      return primitives_;
    }

    bool goal_outside() {
      return goal_outside_;
    }

    void set_discretization(int n, bool use_3d) {
      decimal_t du = u_max_ / n;
      dv_ = std::min(0.2 * du * dt_, 0.4);
      vel_ori_ = Vec3f(-v_max_, -v_max_, 0);
      vel_dim_ = (-2*vel_ori_/dv_).cast<int>();

      ds_ = 0.5 * dv_ * dt_;
      pos_ori_ = Vec3f(-100, -100, -100);
      pos_dim_ = ((Vec3f(100, 100, 100) - pos_ori_)/ds_).cast<int>();

      U_.clear();
      if(use_3d) {
        for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
          for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
            for(decimal_t dz = -0.5; dz <= 0.5; dz += 0.5 )
              U_.push_back(Vec3f(dx, dy, dz));
      }
      else{
        for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
          for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
            U_.push_back(Vec3f(dx, dy, 0));
      }
    }


    bool is_goal(const Waypoint& state) const
    {
      bool goaled = (state.pos - goal_node_.pos).norm() < 1;
      if(goaled && goal_node_.use_vel)
        goaled = (state.vel - goal_node_.vel).norm() < dv_ * 5;
      return goaled;
    }

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

    Key state_to_idx(const Waypoint& state) const
    {
      Vec3i pi = ((state.pos - pos_ori_)/ds_).cast<int>();
      Vec3i vi = ((state.vel - vel_ori_)/dv_).cast<int>();
      return std::to_string(pi(0)) + "-" + std::to_string(pi(1)) + "-" + std::to_string(pi(2)) +
        std::to_string(vi(0)) + "-" + std::to_string(vi(1)) + "-" + std::to_string(vi(2));
    }

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


    void set_v_max(decimal_t v) {
      v_max_ = v;
    }

    void set_a_max(decimal_t a) {
      a_max_ = a;
      u_max_ = a_max_;
    }

    void set_dt(decimal_t dt) {
      dt_ = dt;
    }

    decimal_t get_dt() {
      return dt_;
    }

    void info() {
      printf(ANSI_COLOR_YELLOW "\n");
      printf("++++++++++ PLANNER +++++++++++\n");
      printf("+       dt: %.2f               +\n", dt_);
      printf("+       dv: %.2f               +\n", dv_);
      printf("+        w: %.2f               +\n", w_);
      printf("+       wi: %d                 +\n", wi_);
      printf("+    v_max: %.2f               +\n", v_max_);
      printf("+    a_max: %.2f               +\n", a_max_);
      printf("+ neighbors: %zu               +\n", U_.size());
      printf("++++++++++ PLANNER +++++++++++\n");
      printf(ANSI_COLOR_RESET "\n");
    }

    virtual bool is_free(const Vec3f& pt) const { 
      printf("Used Null is_free()\n");
      return false; 
    }

    virtual void set_goal(const Waypoint& state) {}


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


    bool goal_outside_;
    double w_ = 10;
    int wi_ = 1;

    double u_max_ = 1;
    double v_max_ = 2;
    double a_max_ = 1;
    double dt_ = 1.0;
    double ds_, dv_;

    vec_Vec3f U_;
    Waypoint goal_node_;

    Vec3f pos_ori_, vel_ori_;
    Vec3i pos_dim_, vel_dim_;

    mutable vec_Vec3f ps_;
    mutable std::vector<Primitive> primitives_;


};
}


#endif
