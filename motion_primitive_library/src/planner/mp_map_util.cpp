#include <planner/mp_map_util.h>

using namespace MPL;

MPMapUtil::MPMapUtil(bool verbose) {
  planner_verbose_ = verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "[MPPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

void MPMapUtil::setMapUtil(std::shared_ptr<VoxelMapUtil> map_util) {
  ENV_.reset(new MPL::env_map(map_util));
  map_util_ = map_util;
}

void MPMapUtil::setMapUtil(std::shared_ptr<SubVoxelMapUtil> map_util) {
  ENV_.reset(new MPL::env_map(map_util));
}

vec_Vec3f MPMapUtil::getLinkedNodes() const {
  lhm_.clear();
  vec_Vec3f linked_pts;
  for(const auto& it: sss_ptr_->hm) {
    if(it.second && !it.second->pred_hashkey.empty()) {
      for(unsigned int i = 0; i < it.second->pred_hashkey.size(); i++) {
        Key key = it.second->pred_hashkey[i];
        Primitive pr;
        ENV_->forward_action( sss_ptr_->hm[key]->coord, it.second->pred_action_id[i], pr );
        double max_v = std::max(std::max(pr.max_vel(0), pr.max_vel(1)), pr.max_vel(2));
        int n = std::ceil(max_v * pr.t() / map_util_->getRes());
        int prev_id = -1;
        std::vector<Waypoint> ws = pr.sample(n);
        for(const auto& w: ws) {
          int id = map_util_->getIndex(map_util_->floatToInt(w.pos));
          if(id != prev_id) {
            linked_pts.push_back(map_util_->intToFloat(map_util_->floatToInt(w.pos)));
            lhm_[id].push_back(std::make_pair(it.second->hashkey, i));
            prev_id = id;
          }
        }
      }
    }
  }

  printf("number of linked_pts: %zu\n", linked_pts.size());
  return linked_pts;
}


vec_Vec3f MPMapUtil::updateAffectedNodes(const vec_Vec3i& pns) { 
  vec_Vec3f pts;
  std::vector<std::pair<Key, int>> affected_states;
  for(const auto& it: pns) {
    int id = map_util_->getIndex(it);
    auto search = lhm_.find(id);
    if(search != lhm_.end()) {
      for(const auto& node: lhm_[id]) {
        affected_states.push_back(node);
        pts.push_back(sss_ptr_->hm[node.first]->coord.pos);
      }
    }
  }
  printf("number of affected pts: %zu\n", pts.size());

  sss_ptr_->pruneStateSpace(affected_states);
  return pts;
}
