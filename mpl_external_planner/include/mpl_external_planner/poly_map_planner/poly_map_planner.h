/**
 * @file poly_map_planner.h
 * @brief motion planning in polyhedron map
 */

#ifndef MPL_POLY_MAP_PLANNER_H
#define MPL_POLY_MAP_PLANNER_H

#include <mpl_external_planner/poly_map_planner/env_poly_map.h>
#include <mpl_planner/common/planner_base.h>

namespace MPL {

/**
 * @brief Motion primitive planner using polyhedron
 */
template <int Dim>
class PolyMapPlanner : public PlannerBase<Dim, Waypoint<Dim>> {
 public:
  /**
   * @brief Simple constructor
   */
  PolyMapPlanner(bool verbose = false) {
    this->planner_verbose_ = verbose;
    if (this->planner_verbose_)
      printf(ANSI_COLOR_CYAN
             "[PolyMapPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
  }
  /// Set map util
  void setMap(const Vecf<Dim> &ori, const Vecf<Dim> &dim) {
    map_util_.reset(new PolyMapUtil<Dim>());
    map_util_->setBoundingBox(ori, dim);
    this->ENV_.reset(new MPL::env_poly_map<Dim>(map_util_));
  }

  void setStartTime(decimal_t t) { map_util_->setStartTime(t); }

  void setStaticObstacles(const vec_E<PolyhedronObstacle<Dim>> &obs) {
    map_util_->setStaticObstacle(obs);
  }

  void setLinearObstacles(const vec_E<PolyhedronLinearObstacle<Dim>> &obs) {
    map_util_->setLinearObstacle(obs);
  }

  void setNonlinearObstacles(
      const vec_E<PolyhedronNonlinearObstacle<Dim>> &obs) {
    map_util_->setNonlinearObstacle(obs);
  }

  vec_E<Polyhedron<Dim>> getPolyhedrons(decimal_t time) const {
    return map_util_->getPolyhedrons(time);
  }

  vec_E<PolyhedronLinearObstacle<Dim>> getLinearObstacles() const {
    return map_util_->getLinearObstacles();
  }

  Polyhedron<Dim> getBoundingBox() const { return map_util_->getBoundingBox(); }

  void updateNodes() {
    blocked_prs_.clear();
    cleared_prs_.clear();

    if (!this->ss_ptr_) return;
    // this->ss_ptr_->checkValidation();

    std::vector<std::pair<Waypoint<Dim>, int>> blocked_nodes;
    std::vector<std::pair<Waypoint<Dim>, int>> cleared_nodes;
    for (const auto &it : this->ss_ptr_->hm_) {
      const auto &succNode_ptr = it.second;

      for (int i = 0; i < (int)succNode_ptr->pred_coord.size(); i++) {
        Primitive<Dim> pr;
        this->ENV_->forward_action(succNode_ptr->pred_coord[i],
                                   succNode_ptr->pred_action_id[i], pr);
        if (!map_util_->isFree(pr, succNode_ptr->pred_coord[i].t)) {
          if (!std::isinf(succNode_ptr->pred_action_cost[i])) {
            blocked_nodes.push_back(std::make_pair(it.first, i));
            blocked_prs_.push_back(pr);
          }
        } else {
          if (std::isinf(succNode_ptr->pred_action_cost[i])) {
            cleared_nodes.push_back(std::make_pair(it.first, i));
            cleared_prs_.push_back(pr);
          }
        }
      }
    }

    this->ss_ptr_->increaseCost(blocked_nodes);
    this->ss_ptr_->decreaseCost(cleared_nodes, this->ENV_);
  }

  vec_E<Primitive<Dim>> getBlockedPrimitives() { return blocked_prs_; }

  vec_E<Primitive<Dim>> getClearedPrimitives() { return cleared_prs_; }

 protected:
  std::shared_ptr<PolyMapUtil<Dim>> map_util_;
  vec_E<Primitive<Dim>> blocked_prs_;
  vec_E<Primitive<Dim>> cleared_prs_;
};

typedef PolyMapPlanner<2> PolyMapPlanner2D;

typedef PolyMapPlanner<3> PolyMapPlanner3D;
}  // namespace MPL

#endif
