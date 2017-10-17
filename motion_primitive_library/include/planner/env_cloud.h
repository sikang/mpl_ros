/**
 * @file env_cloud.h
 * @biref environment for planning using point cloud
 */

#ifndef ENV_CLOUD_H
#define ENV_CLOUD_H
#include <planner/env_base.h>
#include <primitive/primitive.h>
#include <collision_checking/decomp_util.h>

namespace MPL {

/**
 * @brief Voxel map environment
 */
class env_cloud : public env_base
{
  protected:
    std::unique_ptr<DecompUtil> map_util_;
  public:

    ///Simple constructor
    env_cloud() {}
    ///Simple constructor
    env_cloud(const vec_Vec3f& obs, decimal_t r, const Vec3f& ori, const Vec3f& dim) {
      map_util_.reset(new DecompUtil(r));
      map_util_->setObstacles(obs);
      map_util_->set_region(ori, dim);
    }

    ~env_cloud() {}

    ///Set goal state
    void set_goal(const Waypoint& goal) {
      goal_node_ = goal;
      goal_outside_ = false;
    }

    ///Check if a point is in free space
    bool is_free(const Vec3f& pt) const {
      return true;
      //return map_util_->isFree(pt);
    }

    /**
     * @brief Get successor
     * @param curr The node to expand
     * @param succ The array stores valid successors
     * @param succ_idx The array stores successors' Key
     * @param succ_cost The array stores cost along valid edges
     * @param action_idx The array stores corresponding idx of control for each successor
     * @param action_dts The array stores corresponding dt of control for each successor
     *
     * When goal is outside, extra step is needed for finding optimal trajectory
     * Here we use Heuristic function and multiply with 2
     */
    bool get_succ( const Waypoint& curr, 
        std::vector<Waypoint>& succ,
        std::vector<Key>& succ_idx,
        std::vector<double>& succ_cost,
        std::vector<int>& action_idx,
        std::vector<double>& action_dts ) const
    {
      succ.clear();
      succ_idx.clear();
      succ_cost.clear();
      action_idx.clear();
      action_dts.clear();

      //ws_.push_back(curr);
      if(t_max_ > 0 && curr.t >= t_max_)
        return false;
      ps_.push_back(curr.pos);

      for(int i = 0; i < (int) U_.size(); i++) {
        if((U_[i] - curr.jrk).lpNorm<Eigen::Infinity>() > max_jrk_diff_ || (U_[i] - curr.jrk).norm() > max_jrk_diff_ * 1.414)
          continue;
        Primitive pr(curr, U_[i], dt_);
        Waypoint tn = pr.evaluate(dt_);
        if(pr.valid_vel(v_max_) && pr.valid_acc(a_max_)) {
          decimal_t dt = dt_;
          bool valid = map_util_->isFree(pr, dt);
          if(valid) {
            //primitives_.push_back(pr);
            tn.use_pos = curr.use_pos;
            tn.use_vel = curr.use_vel;
            tn.use_acc = curr.use_acc;
            tn.use_jrk = curr.use_jrk;
            tn.t = curr.t + dt_;

            succ.push_back(tn);
            succ_idx.push_back(state_to_idx(tn));
            succ_cost.push_back(pr.J(wi_) + w_*dt_);
            action_idx.push_back(i);
            action_dts.push_back(dt_);
          }
        }
      }
      return true;
    }

    Polyhedra polyhedra() {
      return map_util_->polyhedra();
    }

};
}


#endif
