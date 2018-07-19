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
     printf(ANSI_COLOR_CYAN "[PolyMapPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
 }
 /// Set map util
 void setMap(const Vecf<Dim> &ori, const Vecf<Dim> &dim,
             const vec_E<Polyhedron<Dim>>& polys) {
   this->ENV_.reset(new MPL::env_poly_map<Dim>(ori, dim, polys));
 }
};

typedef PolyMapPlanner<2> PolyMapPlanner2D;

typedef PolyMapPlanner<3> PolyMapPlanner3D;
}

#endif
