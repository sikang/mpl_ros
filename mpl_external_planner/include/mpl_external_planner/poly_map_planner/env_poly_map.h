/**
 * @file env_poly_map.h
 * @biref environment for planning using polyhedron
 */

#ifndef MPL_ENV_POLY_MAP_H
#define MPL_ENV_POLY_MAP_H
#include <mpl_external_planner/poly_map_planner/poly_map_util.h>
#include <mpl_planner/common/env_base.h>

namespace MPL {

/**
 * @brief Polyhedron environment
 */
template <int Dim>
class env_poly_map : public env_base<Dim> {
protected:
  std::shared_ptr<PolyMapUtil<Dim>> map_util_;

public:
  /// Simple constructor
  env_poly_map() {}
  /// Simple constructor
  env_poly_map(const std::shared_ptr<PolyMapUtil<Dim>>& map_util) {
    map_util_ = map_util;
  }

  ~env_poly_map() {}

  /// Check if a point is in free space
  bool is_free(const Vecf<Dim> &pt) const {
    return true;
    //return map_util_->isFree(pt, t);
  }

	///Genegrate Key from state
	Key state_to_idx(const Waypoint<Dim>& state) const {
		int ti = state.t/this->dt_;
		const Veci<Dim> pi = this->round(state.pos, this->ds_);
		if(state.control == Control::VEL)
			return this->to_string(pi) + std::to_string(ti);
		else if(state.control == Control::ACC) {
			const Veci<Dim> vi = this->round(state.vel, this->dv_);
			return this->to_string(pi) + this->to_string(vi) + std::to_string(ti);
		}
		else if(state.control == Control::JRK) {
			const Veci<Dim> vi = this->round(state.vel, this->dv_);
			const Veci<Dim> ai = this->round(state.acc, this->da_);
			return this->to_string(pi) + this->to_string(vi) + this->to_string(ai) +
				std::to_string(ti);
		}
		else if(state.control == Control::SNP) {
			const Veci<Dim> vi = this->round(state.vel, this->dv_);
			const Veci<Dim> ai = this->round(state.acc, this->da_);
			const Veci<Dim> ji = this->round(state.jrk, this->dj_);
			return this->to_string(pi) + this->to_string(vi) + this->to_string(ai) +
				this->to_string(ji) + std::to_string(ti);
		}
		else if(state.control == Control::VELxYAW) {
			int yawi = std::round(state.yaw/this->dyaw_);
			return this->to_string(pi) + std::to_string(yawi) + "-" + std::to_string(ti);
		}
		else if(state.control == Control::ACCxYAW) {
			const Veci<Dim> vi = this->round(state.vel, this->dv_);
			int yawi = std::round(state.yaw/this->dyaw_);
			return this->to_string(pi) + this->to_string(vi) +
				std::to_string(yawi) + "-" + std::to_string(ti);
		}
		else if(state.control == Control::JRKxYAW) {
			const Veci<Dim> vi = this->round(state.vel, this->dv_);
			const Veci<Dim> ai = this->round(state.acc, this->da_);
			int yawi = std::round(state.yaw/this->dyaw_);
			return this->to_string(pi) + this->to_string(vi) + this->to_string(ai) +
				std::to_string(yawi) + "-" + std::to_string(ti);
		}
		else if(state.control == Control::SNPxYAW) {
			const Veci<Dim> vi = this->round(state.vel, this->dv_);
			const Veci<Dim> ai = this->round(state.acc, this->da_);
			const Veci<Dim> ji = this->round(state.jrk, this->dj_);
			int yawi = std::round(state.yaw/this->dyaw_);
			return this->to_string(pi) + this->to_string(vi) + this->to_string(ai) +
        this->to_string(ji) + std::to_string(yawi) + "-" + std::to_string(ti);
		}
		else
			return std::to_string(ti);
	}

  /**
   * @brief Get successor
   * @param curr The node to expand
   * @param succ The array stores valid successors
   * @param succ_idx The array stores successors' Key
   * @param succ_cost The array stores cost along valid edges
   * @param action_idx The array stores corresponding idx of control for each
   * successor
   *
   * When goal is outside, extra step is needed for finding optimal trajectory
	 * Here we use Heuristic function and multiply with 2
	 */
	void get_succ(const Waypoint<Dim> &curr, vec_E<Waypoint<Dim>> &succ,
								std::vector<Key> &succ_idx, std::vector<decimal_t> &succ_cost,
								std::vector<int> &action_idx) const {
		succ.clear();
		succ_idx.clear();
		succ_cost.clear();
		action_idx.clear();

		for (size_t i = 0; i < this->U_.size(); i++) {
			Primitive<Dim> pr(curr, this->U_[i], this->dt_);
			Waypoint<Dim> tn = pr.evaluate(this->dt_);
			if(!map_util_->isFree(tn.pos, curr.t+this->dt_) ||
				 !validate_primitive(pr, this->v_max_, this->a_max_, this->j_max_))
				continue;
			if(!map_util_->isFree(pr, curr.t))
				continue;
			tn.t = curr.t + this->dt_;
			succ.push_back(tn);
			succ_idx.push_back(state_to_idx(tn));
			succ_cost.push_back(pr.J(pr.control()) + this->w_ * this->dt_);
			action_idx.push_back(i);
		}
	}
};
}

#endif
