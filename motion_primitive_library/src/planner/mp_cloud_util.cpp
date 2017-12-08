#include <planner/mp_cloud_util.h>

using namespace MPL;

MPCloudUtil::MPCloudUtil(bool verbose)
{
  planner_verbose_= verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "[MPCloudUtil] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

void MPCloudUtil::setMap(const vec_Vec3f& obs, decimal_t r, const Vec3f& ori, const Vec3f& dim) {
  ENV_.reset(new MPL::env_cloud(obs, r, ori, dim));
}

Polyhedra MPCloudUtil::getPolyhedra() {
  return ENV_->polyhedra();
}

