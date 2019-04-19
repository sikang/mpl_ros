/**
 * @file env_cloud.h
 * @brief environment for planning using ellipsoid model with point cloud
 */

#ifndef MPL_ENV_CLOUD_H
#define MPL_ENV_CLOUD_H
#include <mpl_external_planner/ellipsoid_planner/ellipsoid_util.h>
#include <mpl_planner/common/env_base.h>

namespace MPL {

/**
 * @brief Point cloud environment
 */
class env_cloud : public env_base<3> {
 protected:
  std::unique_ptr<EllipsoidUtil> map_util_;

 public:
  /// Simple constructor
  env_cloud() {}
  /// Simple constructor
  env_cloud(const vec_Vec3f &obs, decimal_t r, const Vec3f &ori,
            const Vec3f &dim) {
    map_util_.reset(new EllipsoidUtil(r));
    map_util_->setObstacles(obs);
    map_util_->setBoundingBox(ori, dim);
  }

  ~env_cloud() {}

  /// Check if a point is in free space
  bool is_free(const Vec3f &pt) const {
    return true;
    // return map_util_->isFree(pt);
  }

  /**
   * @brief Get successor
   * @param curr The node to expand
   * @param succ The array stores valid successors
   * @param succ_cost The array stores cost along valid edges
   * @param action_idx The array stores corresponding idx of control for each
   * successor
   *
   * When goal is outside, extra step is needed for finding optimal trajectory
   * Here we use Heuristic function and multiply with 2
   */
  void get_succ(const Waypoint3D &curr, vec_E<Waypoint3D> &succ,
                std::vector<decimal_t> &succ_cost,
                std::vector<int> &action_idx) const {
    succ.clear();
    succ_cost.clear();
    action_idx.clear();

    // expanded_nodes_.push_back(curr.pos);
    // ws_.push_back(curr);
    for (int i = 0; i < (int)U_.size(); i++) {
      Primitive3D pr(curr, U_[i], dt_);
      Waypoint3D tn = pr.evaluate(dt_);
      if (tn == curr || !validate_primitive(pr, v_max_, a_max_, j_max_) ||
          !map_util_->isFree(pr))
        continue;
      tn.t = curr.t + dt_;
      succ.push_back(tn);
      succ_cost.push_back(pr.J(pr.control()) + w_ * dt_);
      action_idx.push_back(i);
    }
  }
};
}  // namespace MPL

#endif
