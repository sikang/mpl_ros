#include <planner/mp_vision_util.h>

MPVisionUtil::MPVisionUtil(bool verbose)
{
  planner_verbose_ = verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "[MPPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
  ENV_.reset(new mrsl::env_vision());
}

cv::Mat MPVisionUtil::getImage() { return ENV_->image(getTraj()); }

void MPVisionUtil::addImage(const cv::Mat& img, const Aff3f& TF, const CameraInfo& info) {
  ENV_->add_image(img, TF, info);
}

bool MPVisionUtil::plan(const Waypoint &start, const Waypoint &goal) {
  path_.clear();

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
        printf(ANSI_COLOR_RED "[MPPlanner] start is not free!" ANSI_COLOR_RESET "\n");
      return false;
    }
  }

  mrsl::ARAStar<Waypoint> AA;
  std::vector<int> action_idx;
  std::list<Waypoint> path;

  ENV_->set_goal(goal);

  AA.Astar(start, ENV_->state_to_idx(start), *ENV_, path, action_idx, epsilon_, max_num_);

  if (path.empty()) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "[MPPlanner] Cannot find a path!" ANSI_COLOR_RESET "\n");
    return false;
  }

  path_.push_back(start);
  int i = 0;
  for (const auto& it: path) {
    if(ENV_->goal_outside() && i == (int) path.size() - 1)
      break;
    path_.push_back(it);
    i++;
  }

  return true;
}

