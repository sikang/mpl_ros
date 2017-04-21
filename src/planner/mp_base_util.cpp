#include <planner/mp_base_util.h>

MPBaseUtil::MPBaseUtil()
{
  planner_verbose_ = false;
}

std::vector<Waypoint> MPBaseUtil::getPath() {
  return path_;
}

std::vector<Primitive> MPBaseUtil::getPrimitives() { 
  return ENV_->primitives(); 
}

vec_Vec3f MPBaseUtil::getPs() { return ENV_->ps(); }

void MPBaseUtil::setEpsilon(decimal_t eps) {
  epsilon_ = eps;
}

void MPBaseUtil::setDt(decimal_t dt) {
  ENV_->set_dt(dt);
  ENV_->set_discretization(false);
}

void MPBaseUtil::setAmax(decimal_t a_max) {
  ENV_->set_a_max(a_max);
}

void MPBaseUtil::setVmax(decimal_t v_max) {
  ENV_->set_v_max(v_max);
}

Trajectory MPBaseUtil::getTraj() {
  std::vector<Primitive> ps;

  for(int i = 0; i < (int)path_.size()-1; i++){
    Waypoint nw1 = path_[i];
    Waypoint nw2 = path_[i+1];

    nw1.use_pos = true;
    nw1.use_vel = true;
    nw1.use_acc = false;

    nw2.use_pos = true;
    nw2.use_vel = true;
    nw2.use_acc = false;

    Primitive p(nw1, nw2, ENV_->get_dt());
    ps.push_back(p);
  }

  return Trajectory(ps);
}
