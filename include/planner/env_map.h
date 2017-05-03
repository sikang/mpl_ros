#ifndef ENV_MP_H
#define ENV_MP_H
#include <planner/env_base.h>
#include <primitive/primitive.h>
#include <collision_checking/voxel_map_util.h>
#include <collision_checking/sub_voxel_map_util.h>

namespace MPL {
class env_map : public env_base
{
  protected:
    std::shared_ptr<VoxelMapUtil> map_util_;

  public:

    env_map(std::shared_ptr<VoxelMapUtil> map_util)
      : map_util_(map_util)
    {}

    ~env_map() {}

    void set_goal(const Waypoint& goal) {
      goal_node_ = goal;

      const Vec3i goal_int = map_util_->floatToInt(goal_node_.pos);
      if (map_util_->isOutSide(goal_int)) {
        printf(ANSI_COLOR_GREEN "goal out side! " ANSI_COLOR_RESET "\n");
        goal_outside_ = true;
      }
      else
        goal_outside_ = false;
    }

    bool is_free(const Vec3f& pt) const {
      return map_util_->isFree(map_util_->floatToInt(pt));
    }


    bool is_free(const Primitive& p) const {
      std::vector<Waypoint> pts = p.sample(5);
      for(const auto& pt: pts){
        Vec3i pn = map_util_->floatToInt(pt.pos);
        if(map_util_->isOccupied(pn) ||
           (!goal_outside_ && map_util_->isOutSide(pn)))
          return false;
      }

      return true;
    }

    void get_succ( const Waypoint& curr, 
        std::vector<Waypoint>& succ,
        std::vector<Key>& succ_idx,
        std::vector<double>& succ_cost,
        std::vector<int>& action_idx ) const
    {
      succ.clear();
      succ_idx.clear();
      succ_cost.clear();
      action_idx.clear();

      const Vec3i pn = map_util_->floatToInt(curr.pos);
      if (goal_outside_ && map_util_->isOutSide(pn)) {
        succ.push_back(goal_node_);
        succ_idx.push_back(state_to_idx(goal_node_));
        succ_cost.push_back(2*get_heur(curr));
        action_idx.push_back(-1);
      }

      if(map_util_->isOutSide(pn))
        return;

      ps_.push_back(curr.pos);
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
        }

      }

    }

};
}


#endif
