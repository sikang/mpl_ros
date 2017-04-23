#ifndef ENV_SFC_H
#define ENV_SFC_H
#include <planner/env_base.h>
#include <collision_checking/sfc_util.h>

namespace mrsl {
class env_sfc : public env_base
{
  protected:
    std::unique_ptr<SFCUtil> map_util_;

  public:

    env_sfc(const Polyhedra& polys) {
      map_util_.reset(new SFCUtil(polys));
    }

    Polyhedra map() {
      return map_util_->sfc();
    }

    void set_goal(const Waypoint& goal) {
      ps_.clear();
      primitives_.clear();
      goal_node_ = goal;
      goal_outside_ = false;

      if (!map_util_->isFree(goal.pos)) {
        printf(ANSI_COLOR_GREEN "goal out side! " ANSI_COLOR_RESET "\n");
        goal_outside_ = true;
      }
    }


    bool is_free(const Vec3f& pt) const {
      return map_util_->isFree(pt);
    }

    bool is_free(const Primitive& pr) const {
      return map_util_->isFree(pr, dt_);
    }

    void get_succ( const Waypoint& curr, 
        std::vector<Waypoint>& succ,
        std::vector<Key>& succ_idx,
        std::vector<double>& succ_cost,
        std::vector<int>& action_idx ) const
    {
      //printf("expand!\n");
      ps_.push_back(curr.pos);
      succ.clear();
      succ_idx.clear();
      succ_cost.clear();
      action_idx.clear();

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

};

}

#endif
