#include <planner/mp_sfc_util.h>

MPSFCUtil::MPSFCUtil(bool verbose)
{
  planner_verbose_= verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "MPSFCUtil PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

void MPSFCUtil::setMap(const Polyhedra& polys) {
  ENV_.reset(new mrsl::env_sfc(polys));
}

bool MPSFCUtil::plan(const Waypoint &start, const Waypoint &goal) {
  path_.clear();
  primitives_.clear();

  if(planner_verbose_) {
    printf("start pos: [%f, %f, %f], vel: [%f, %f, %f], acc: [%f, %f, %f]\n",
        start.pos(0), start.pos(1), start.pos(2),
        start.vel(0), start.vel(1), start.vel(2),
        start.acc(0), start.acc(1), start.acc(2));
    printf("goal pos: [%f, %f, %f], vel: [%f, %f, %f], acc: [%f, %f, %f]\n",
        goal.pos(0), goal.pos(1), goal.pos(2),
        goal.vel(0), goal.vel(1), goal.vel(2),
        goal.acc(0), goal.acc(1), goal.acc(2));
    if(!ENV_->is_free(start.pos)) {
      if(planner_verbose_)
        printf(ANSI_COLOR_RED "MPSFCUtil: start is not free!" ANSI_COLOR_RESET "\n");
      return false;
    }
  }

  mrsl::ARAStar<Waypoint> AA;
  std::vector<int> action_idx;
  std::list<Waypoint> path;

  //ENV_->reset();
  ENV_->set_goal(goal);
  if(ENV_->goal_outside()) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "MPSFCUtil: Goal is outside!\n" ANSI_COLOR_RESET);
  }
  else
    AA.Astar(start, ENV_->state_to_idx(start), *ENV_, path, action_idx, epsilon_, 1000);

  if (path.empty()) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "MPSFCUtil: Cannot find a path!" ANSI_COLOR_RESET "\n");
    return false;
  }

  path_.push_back(start);
  for (const auto &it_node : path)
    path_.push_back(it_node);

  //std::reverse(path_.begin(), path_.end());
  return true;
}

