#include <mpl_external_planner/ellipsoid_planner/ellipsoid_planner.h>

using namespace MPL;

EllipsoidPlanner::EllipsoidPlanner(bool verbose) {
  planner_verbose_ = verbose;
  if (planner_verbose_)
    printf(ANSI_COLOR_CYAN
           "[EllipsoidPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

void EllipsoidPlanner::setMap(const vec_Vec3f &obs, decimal_t r,
                              const Vec3f &ori, const Vec3f &dim) {
  ENV_.reset(new MPL::env_cloud(obs, r, ori, dim));
}

