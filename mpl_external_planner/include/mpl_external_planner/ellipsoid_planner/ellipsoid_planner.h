/**
 * @file ellipsoid_planner.h
 * @brief motion planning in SE(3) using ellipsoids for collision checking
 */

#ifndef MPL_ELLIPSOID_PLANNER_H
#define MPL_ELLIPSOID_PLANNER_H

#include <mpl_external_planner/ellipsoid_planner/env_cloud.h>
#include <mpl_planner/common/planner_base.h>

namespace MPL {

/**
 * @brief Motion primitive planner using point cloud
 */
class EllipsoidPlanner : public PlannerBase<3, Waypoint3D> {
 public:
  /**
   * @brief Simple constructor
   * @param verbose enable print out
   */
  EllipsoidPlanner(bool verbose) {
    planner_verbose_ = verbose;
    if (planner_verbose_)
      printf(ANSI_COLOR_CYAN
             "[EllipsoidPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
  }
  /// Set map util
  void setMap(const vec_Vec3f &obs, decimal_t r, const Vec3f &ori,
              const Vec3f &dim) {
    ENV_.reset(new MPL::env_cloud(obs, r, ori, dim));
  }
};
}  // namespace MPL

#endif
