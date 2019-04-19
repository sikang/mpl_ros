/**
 * @file env_poly_map.h
 * @biref environment for planning using polyhedron
 */

#ifndef MPL_ENV_POLY_MAP_H
#define MPL_ENV_POLY_MAP_H
#include <mpl_external_planner/poly_map_planner/poly_map_util.h>
#include <mpl_planner/common/env_base.h>

namespace MPL {

/**
 * @brief Polyhedron environment
 */
template <int Dim>
class env_poly_map : public env_base<Dim> {
 public:
  /// Simple constructor
  env_poly_map() {}
  /// Simple constructor
  env_poly_map(const std::shared_ptr<PolyMapUtil<Dim>> &map_util) {
    map_util_ = map_util;
  }

  ~env_poly_map() {}

  /// Check if a point is inside bounding box and outside static obstacles
  bool is_free(const Vecf<Dim> &pt) const { return map_util_->isInside(pt); }

  /// Overloard original function
  bool is_free(const Primitive<Dim> &pr) const { return true; }

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
  void get_succ(const Waypoint<Dim> &curr, vec_E<Waypoint<Dim>> &succ,
                std::vector<decimal_t> &succ_cost,
                std::vector<int> &action_idx) const {
    succ.clear();
    succ_cost.clear();
    action_idx.clear();

    this->expanded_nodes_.push_back(curr.pos);

    for (size_t i = 0; i < this->U_.size(); i++) {
      Primitive<Dim> pr(curr, this->U_[i], this->dt_);
      Waypoint<Dim> tn = pr.evaluate(this->dt_);
      if (!map_util_->isInside(tn.pos) ||
          !validate_primitive(pr, this->v_max_, this->a_max_, this->j_max_))
        continue;
      decimal_t cost = map_util_->isFree(pr, curr.t)
                           ? calculate_intrinsic_cost(pr)
                           : std::numeric_limits<decimal_t>::infinity();
      tn.t = curr.t + this->dt_;
      tn.enable_t = true;
      succ.push_back(tn);
      succ_cost.push_back(cost);
      action_idx.push_back(i);
    }
  }

  decimal_t calculate_intrinsic_cost(const Primitive<Dim> &pr) const {
    return pr.J(pr.control()) + 0.001 * pr.J(Control::VEL) +
           this->w_ * this->dt_;
  }

 protected:
  std::shared_ptr<PolyMapUtil<Dim>> map_util_;
};
}  // namespace MPL

#endif
