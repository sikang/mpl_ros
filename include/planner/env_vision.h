#ifndef ENV_MP_H
#define ENV_MP_H
#include <planner/env_int.h>
#include <primitive/primitive.h>
#include <collision_checking/vision_queue_util.h>

namespace mrsl
{
  class env_vision : public env_int<Waypoint>
  {
    protected:
      Waypoint goal_node_;    // discrete coordinates of the goal node
      std::unique_ptr<VisionQueueUtil> map_util_;
      decimal_t w_ = 10;

      bool goal_outside_;
      vec_Vec3f U_;
      int wi_ = 1;
      decimal_t u_max_ = 1;
      decimal_t v_max_ = 2;
      decimal_t a_max_ = 1;
      decimal_t dt_ = 1.0;
      decimal_t ds_, dv_;
      Vec3f pos_ori_, vel_ori_;
      Vec3i pos_dim_, vel_dim_;
      int pos_length_;

      vec_Vec3f ps_;
      std::vector<Primitive> primitives_;

    public:

      env_vision() {
        map_util_.reset(new VisionQueueUtil());
      }

      vec_Vec3f ps() {
        return ps_;
      }

      vec_Vec3f cloud() {
        return map_util_->cloud();
      }

      std::vector<Primitive> primitives() {
        return primitives_;
      }

      cv::Mat image(const Trajectory& traj) {
        return map_util_->img(0, traj);
      }

      void add_image(const cv::Mat& img, const Aff3f& TF, const CameraInfo& info) {
        map_util_->addImage(img, TF, info);
      }

      void set_discretization(bool use_3d) {
        decimal_t du = 1;
        dv_ = du * dt_;
        wi_ = 1;
        vel_ori_ = Vec3f(-v_max_, -v_max_, 0);
        vel_dim_ = (-2*vel_ori_/dv_).cast<int>();

        ds_ = 0.5 * dv_ * dt_;
        pos_ori_ = Vec3f(-10, -10, 0);
        pos_dim_ = ((Vec3f(10, 10, 5) - pos_ori_)/ds_).cast<int>();
        pos_length_ = pos_dim_(0) * pos_dim_(1) * pos_dim_(2);

        if(use_3d) {
          for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
            for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
              for(decimal_t dz = -u_max_; dz <= u_max_; dz += du )
                U_.push_back(Vec3f(dx, dy, dz));
        }
        else{
          for(decimal_t dx = -u_max_; dx <= u_max_; dx += du )
            for(decimal_t dy = -u_max_; dy <= u_max_; dy += du )
              U_.push_back(Vec3f(dx, dy, 0));
        }
 
      }

      void set_goal(const Waypoint& goal) {
        ps_.clear();
        primitives_.clear();
        goal_node_ = goal;

        if (map_util_->isOutside(goal.pos)) {
          printf(ANSI_COLOR_GREEN "goal out side! " ANSI_COLOR_RESET "\n");
          goal_outside_ = true;
        }
      }

      bool is_goal(const Waypoint& state) const
      {
        return (state.pos - goal_node_.pos).norm() <= 1 &&
          (state.vel - goal_node_.vel).norm() <= 0.1;
      }

      double get_heur(const Waypoint& state) const
      {
        //return 0;
        //return w_*(state.n.pos - goal_node_.n.pos).lpNorm<Eigen::Infinity>() / v_max_;
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


      int state_to_idx(const Waypoint& state) const
      {
        Vec3i pi = ((state.pos - pos_ori_)/ds_).cast<int>();
        Vec3i vi = ((state.vel - vel_ori_)/dv_).cast<int>();
        int pos_id = pi(0) + pi(1)*pos_dim_(0) + pi(2)*pos_dim_(0)*pos_dim_(1);
        return pos_id + pos_length_ * vi(0) + pos_length_ * vel_dim_(0) * vi(1);
      }

      bool is_free(const Vec3f& pt) const {
        return map_util_->isFree(pt);
      }

      bool is_free(const Primitive& pr) const {
        return map_util_->isFree(pr);
      }

      void get_succ( const Waypoint& curr, 
          std::vector<Waypoint>& succ,
          std::vector<int>& succ_idx,
          std::vector<double>& succ_cost,
          std::vector<int>& action_idx ) const
      {
        //ps_.push_back(curr.pos);
        succ.clear();
        succ_idx.clear();
        succ_cost.clear();
        action_idx.clear();

        if (map_util_->isOutside(curr.pos)) {
          if(goal_outside_) {
            succ.push_back(goal_node_);
            succ_idx.push_back(state_to_idx(goal_node_));
            succ_cost.push_back(2*get_heur(curr));
            action_idx.push_back(-1);
          }
          return;
        }

        if(!map_util_->isFree(curr.pos))
          return;


        for(int i = 0; i < (int)U_.size(); i++) {
          Primitive p(curr, U_[i], dt_);
          Waypoint tn = p.evaluate(dt_);
          if(p.valid_vel(v_max_)) {
            if(!is_free(p))
              continue;
            tn.use_pos = true;
            tn.use_vel = true;
            tn.use_acc = false;

            succ.push_back(tn);
            succ_idx.push_back(state_to_idx(tn));
            succ_cost.push_back(p.J(wi_) + w_*dt_);
            action_idx.push_back(i);
            //primitives_.push_back(p);
          }

        }

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



  };

}

#endif
