#include <planner/mp_map_util.h>

using namespace MPL;

MPMapUtil::MPMapUtil(bool verbose) {
  planner_verbose_ = verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "[MPPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

void MPMapUtil::setMapUtil(std::shared_ptr<VoxelMapUtil> map_util) {
  ENV_.reset(new MPL::env_map(map_util));
}

void MPMapUtil::setMapUtil(std::shared_ptr<SubVoxelMapUtil> map_util) {
  ENV_.reset(new MPL::env_map(map_util));
}
