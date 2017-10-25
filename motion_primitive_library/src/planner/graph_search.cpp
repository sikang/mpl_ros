#include <planner/graph_search.h>
#include <planner/env_base.h>

using namespace MPL;

void StateSpace::getSubStateSpace(int id) {
  goalNode_ptr = nullptr;
  if(best_child_.empty())
    return;

  StatePtr currNode_ptr = best_child_[id];
  currNode_ptr->pred_action_cost.clear();
  currNode_ptr->pred_action_id.clear();
  currNode_ptr->pred_hashkey.clear();
  currNode_ptr->t = 0;

  hashMap<State> new_hm;
  priorityQueue<State> epq;
  currNode_ptr->heapkey = epq.push(std::make_pair(currNode_ptr->g, currNode_ptr));

  double init_g = currNode_ptr->g;
  for(auto& it: hm) {
    if(it.second) {
      it.second->g = std::numeric_limits<double>::infinity();
      it.second->rhs = std::numeric_limits<double>::infinity();
      it.second->iterationopened = false;
      //it.second->iterationclosed = 0;
      it.second->pred_action_cost.clear();
      it.second->pred_action_id.clear();
      it.second->pred_hashkey.clear();
    }
  }

  currNode_ptr->g = init_g;
  while(!epq.empty()) {
    for(unsigned int i = 0; i < currNode_ptr->succ_hashkey.size(); i++) {
      Key key = currNode_ptr->succ_hashkey[i];
      if(key.empty())
        continue;
      StatePtr& child_ptr = hm[key];
      child_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
      child_ptr->pred_action_cost.push_back(currNode_ptr->succ_action_cost[i]);
      child_ptr->pred_action_id.push_back(i);

      double tentative_gval = currNode_ptr->g + currNode_ptr->succ_action_cost[i];

      if(tentative_gval < child_ptr->g || !child_ptr->iterationopened) {
        child_ptr->t = currNode_ptr->t + dt;
        child_ptr->g = tentative_gval;
        if( child_ptr->iterationopened)
        {
          (*child_ptr->heapkey).first = tentative_gval;     // update heap element
          epq.increase( child_ptr->heapkey );       // update heap
        }
        else {
          child_ptr->heapkey = epq.push( std::make_pair(tentative_gval, child_ptr) );
          child_ptr->iterationopened = true;
        }
      }
    }

    currNode_ptr = epq.top().second; epq.pop();
    new_hm[currNode_ptr->hashkey] = currNode_ptr;
  }

  hm = new_hm;

  pq.clear();
  for(auto& it: hm) {
    if(!it.second->iterationclosed) {
      it.second->heapkey = pq.push( std::make_pair(it.second->g + eps * it.second->h, it.second) );
      it.second->iterationopened = true;
      it.second->rhs = it.second->g;
      it.second->g = std::numeric_limits<double>::infinity();
    }
  }

}


void StateSpace::increaseCost(std::vector<std::pair<Key, int> > states) {
  goalNode_ptr = nullptr;
  if(states.empty())
    return;
  for(const auto& affected_node: states) {
    int id = affected_node.second;
    hm[affected_node.first]->pred_action_cost[id] = std::numeric_limits<double>::infinity();
    //hm[affected_node.first]->pred_action_cost[id] = 1000;
    updateNode(hm[affected_node.first]);
    Key parent_key = hm[affected_node.first]->pred_hashkey[id];
    int succ_act_id = hm[affected_node.first]->pred_action_id[id];
    hm[parent_key]->succ_action_cost[succ_act_id] = std::numeric_limits<double>::infinity();
  }

}

void StateSpace::decreaseCost(std::vector<std::pair<Key, int> > states, const env_base& ENV) {
  goalNode_ptr = nullptr;
  if(states.empty())
    return;
  for(const auto& affected_node: states) {
    int id = affected_node.second;
    Key parent_key = hm[affected_node.first]->pred_hashkey[id];
    Primitive pr;
    ENV.forward_action( hm[parent_key]->coord, hm[affected_node.first]->pred_action_id[id], pr );
    if(ENV.is_free(pr)) {
      hm[affected_node.first]->pred_action_cost[id] = pr.J(ENV.wi_) + ENV.w_*ENV.dt_;
      updateNode(hm[affected_node.first]);
      int succ_act_id = hm[affected_node.first]->pred_action_id[id];
      hm[parent_key]->succ_action_cost[succ_act_id] = hm[affected_node.first]->pred_action_cost[id];
    }
  }
}

void StateSpace::updateNode(StatePtr currNode_ptr) {
  //printf("update node at t: %f, rhs: %f\n", currNode_ptr->t, currNode_ptr->rhs);
  double parent_t = currNode_ptr->t - dt;
  // if currNode is not start, update its rhs
  if(currNode_ptr->rhs != 0) {
    currNode_ptr->rhs = std::numeric_limits<double>::infinity();
    for(unsigned int i = 0; i < currNode_ptr->pred_hashkey.size(); i++) {
      Key key = currNode_ptr->pred_hashkey[i];
      if(!hm[key])
        continue;
      if(currNode_ptr->rhs > hm[key]->g + currNode_ptr->pred_action_cost[i]) {
        currNode_ptr->rhs = hm[key]->g + currNode_ptr->pred_action_cost[i];
        parent_t = hm[key]->t;
      }
    }
  }


  // if currNode is in openset, remove it
  if(currNode_ptr->iterationopened && !currNode_ptr->iterationclosed ) {
    pq.erase(currNode_ptr->heapkey);
    currNode_ptr->iterationclosed = true;
  }

  // if currNode's g value is not equal to its rhs, put it into openset
  //if(currNode_ptr->g != currNode_ptr->rhs || !currNode_ptr->iterationopened) {
  if(currNode_ptr->g != currNode_ptr->rhs) {
    double fval = std::min(currNode_ptr->g, currNode_ptr->rhs) + eps * currNode_ptr->h;

    currNode_ptr->heapkey = pq.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
    currNode_ptr->t = parent_t + dt;
    
    //printf("t: %f, curr g: %f, rhs: %f, h: %f, f: %f\n", 
    //    currNode_ptr->t, currNode_ptr->g, currNode_ptr->rhs, currNode_ptr->h, fval);
 
  }

}


double GraphSearch::Astar(const Waypoint& start_coord, Key start_idx,
    const env_base& ENV, std::shared_ptr<StateSpace> sss_ptr, 
    Trajectory& traj, int max_expand, double max_t)
{
  traj.segs.clear();
  sss_ptr->best_child_.clear();
  // Check if done
  if( ENV.is_goal(start_coord) )
    return 0;
  
  // Initialize start node
  StatePtr currNode_ptr = sss_ptr->hm[start_idx];
  if(sss_ptr->pq.empty()) {
    if(verbose_) 
      printf(ANSI_COLOR_GREEN "Start new node!\n" ANSI_COLOR_RESET);
    currNode_ptr = std::make_shared<State>(State(start_idx, start_coord));
    currNode_ptr->t = 0;
    currNode_ptr->g = 0;
    currNode_ptr->h = ENV.get_heur(start_coord, currNode_ptr->t);
    double fval = currNode_ptr->g + sss_ptr->eps * currNode_ptr->h;
    currNode_ptr->heapkey = sss_ptr->pq.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
    sss_ptr->hm[start_idx] = currNode_ptr;
  }

  int expand_iteration = 0;
  while(true)
  {
    expand_iteration++;
    // get element with smallest cost
    currNode_ptr = sss_ptr->pq.top().second;     
    sss_ptr->pq.pop(); 
    currNode_ptr->iterationclosed = true; // Add to closed list

    if((max_t > 0 && currNode_ptr->t >= max_t && !std::isinf(currNode_ptr->g)) || ENV.is_goal(currNode_ptr->coord)) {
      if(verbose_) 
        printf(ANSI_COLOR_GREEN "MaxExpandTime [%f] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_t);
      break;
    }

    // Get successors
    std::vector<Waypoint> succ_coord;
    std::vector<MPL::Key> succ_idx;
    std::vector<double> succ_cost;
    std::vector<int> succ_act_idx;

    ENV.get_succ( currNode_ptr->coord, succ_coord, succ_idx, succ_cost, succ_act_idx);
    currNode_ptr->succ_hashkey.resize(ENV.U_.size(), Key());
    currNode_ptr->succ_action_cost.resize(ENV.U_.size(), std::numeric_limits<double>::infinity());

    // Process successors
    for( unsigned s = 0; s < succ_coord.size(); ++s )
    {
      // Get child
      StatePtr& child_ptr = sss_ptr->hm[ succ_idx[s] ];
      if( !child_ptr )
      {
	child_ptr = std::make_shared<State>(State(succ_idx[s], succ_coord[s]) );
	child_ptr->t = currNode_ptr->t + ENV.dt_;
	child_ptr->h = ENV.get_heur( child_ptr->coord, child_ptr->t); 
	child_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
        child_ptr->pred_action_id.push_back(succ_act_idx[s]);
        child_ptr->pred_action_cost.push_back(succ_cost[s]);
      }

      // store the hashkey
      currNode_ptr->succ_hashkey[succ_act_idx[s]] = succ_idx[s];
      currNode_ptr->succ_action_cost[succ_act_idx[s]] = succ_cost[s];

      //see if we can improve the value of succstate
      //taking into account the cost of action
      double tentative_gval = currNode_ptr->g + succ_cost[s];

      if( tentative_gval < child_ptr->g )
      {
	child_ptr->pred_hashkey.front() = currNode_ptr->hashkey;  // Assign new parent
	child_ptr->pred_action_id.front() = succ_act_idx[s];
	child_ptr->pred_action_cost.front() = succ_cost[s];
	child_ptr->t = currNode_ptr->t + ENV.dt_;
	child_ptr->g = tentative_gval;    // Update gval

	double fval = child_ptr->g + (sss_ptr->eps) * child_ptr->h;

	// if currently in OPEN, update
	if( child_ptr->iterationopened && !child_ptr->iterationclosed)
	{
          if(verbose_) {
            if((*child_ptr->heapkey).first < fval) {
              std::cout << "UPDATE fval(old) = " << (*child_ptr->heapkey).first << std::endl;
              std::cout << "UPDATE fval = " << fval << std::endl;
            }
          }

	  (*child_ptr->heapkey).first = fval;     // update heap element
	  //sss_ptr->pq.update(child_ptr->heapkey);
	  sss_ptr->pq.increase( child_ptr->heapkey );       // update heap
	}
	// if currently in CLOSED
	else if( child_ptr->iterationopened && child_ptr->iterationclosed)
	{
	  printf(ANSI_COLOR_RED "ASTAR ERROR!\n" ANSI_COLOR_RESET);
	  // child_ptr->heapkey = sss_ptr->pq.push( std::make_pair(fval,child_ptr) );
	  // child_ptr->iterationopened = sss_ptr->searchiteration;
	  // child_ptr->iterationclosed = 0;
	}
	else // new node, add to heap
	{
	  //std::cout << "ADD fval = " << fval << std::endl;
	  child_ptr->heapkey = sss_ptr->pq.push( std::make_pair(fval, child_ptr));
	  child_ptr->iterationopened = true;
	}
      } //
    } // Process successors    


    if(max_expand > 0 && expand_iteration >= max_expand) {
      printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
      return std::numeric_limits<double>::infinity();
    }

    if( sss_ptr->pq.empty()) {
      printf(ANSI_COLOR_RED "Priority queue is empty!!!!!!\n\n" ANSI_COLOR_RESET);
      return std::numeric_limits<double>::infinity();
    }
  }

  if(verbose_) {
    //printf(ANSI_COLOR_GREEN "topKey: %f, goal g: %f, rhs: %f!\n" ANSI_COLOR_RESET, sss_ptr->pq.top().first, goalNode_ptr->g, goalNode_ptr->rhs);
    double fval = std::min(currNode_ptr->g, currNode_ptr->rhs) + sss_ptr->eps * currNode_ptr->h;
    printf(ANSI_COLOR_GREEN "currNode key: %f, g: %f, rhs: %f!\n" ANSI_COLOR_RESET, fval, currNode_ptr->g, currNode_ptr->rhs);
    printf(ANSI_COLOR_GREEN "Expand [%d] nodes!\n" ANSI_COLOR_RESET, expand_iteration);
  }

  double pcost = currNode_ptr->g;
  traj = recoverTraj(currNode_ptr, sss_ptr, ENV, start_idx);

  return pcost;
}



double GraphSearch::LPAstar(const Waypoint& start_coord, Key start_idx, 
    const env_base& ENV, std::shared_ptr<StateSpace> sss_ptr, 
    Trajectory& traj, int max_expand, double max_t)
{
  traj.segs.clear();
  sss_ptr->best_child_.clear();
  // Check if done
  if( ENV.is_goal(start_coord) )
    return 0;
  
  // Initialize start node
  StatePtr currNode_ptr = sss_ptr->hm[start_idx];
  if(sss_ptr->pq.empty()) {
    if(verbose_)
      printf(ANSI_COLOR_GREEN "Start new node!\n" ANSI_COLOR_RESET);
    currNode_ptr = std::make_shared<State>(State(start_idx, start_coord));
    currNode_ptr->t = 0;
    currNode_ptr->g = std::numeric_limits<double>::infinity();
    currNode_ptr->rhs = 0;
    currNode_ptr->h = ENV.get_heur(start_coord, currNode_ptr->t);
    double fval = std::min(currNode_ptr->g, currNode_ptr->rhs) + sss_ptr->eps * currNode_ptr->h;
    currNode_ptr->heapkey = sss_ptr->pq.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
    sss_ptr->hm[start_idx] = currNode_ptr;
  }
  // Initialize null goal node
  StatePtr goalNode_ptr = sss_ptr->goalNode_ptr;
  if(!goalNode_ptr) {
    if(verbose_)
      printf(ANSI_COLOR_GREEN "Initialize goal!\n" ANSI_COLOR_RESET);
    goalNode_ptr = std::make_shared<State>(State(Key(), Waypoint()));
  }

  int expand_iteration = 0;
  while(sss_ptr->pq.top().first < std::min(goalNode_ptr->g, goalNode_ptr->rhs) || goalNode_ptr->rhs != goalNode_ptr->g)
  {
    expand_iteration++;
    // get element with smallest cost
    currNode_ptr = sss_ptr->pq.top().second;     
    /*
    printf("[%d] expand, t: %f, g: %f, rhs: %f, h: %f, fval: %f\n", 
        expand_iteration, currNode_ptr->t,
        currNode_ptr->g, currNode_ptr->rhs, currNode_ptr->h, sss_ptr->pq.top().first);
        */

    sss_ptr->pq.pop(); 
    currNode_ptr->iterationclosed = true; // Add to closed list



    // Get successors
    std::vector<Waypoint> succ_coord;
    std::vector<MPL::Key> succ_idx;
    std::vector<double> succ_cost;
    std::vector<int> succ_act_idx;

    ENV.get_succ( currNode_ptr->coord, succ_coord, succ_idx, succ_cost, succ_act_idx);
    currNode_ptr->succ_hashkey.resize(ENV.U_.size(), Key());
    currNode_ptr->succ_action_cost.resize(ENV.U_.size(), std::numeric_limits<double>::infinity());

    std::vector<StatePtr> nodes_ptr;
    // Process successors
    for( unsigned s = 0; s < succ_coord.size(); ++s )
    {
      // Get child
      StatePtr& child_ptr = sss_ptr->hm[ succ_idx[s] ];
      if( !(child_ptr) ) {
        child_ptr = std::make_shared<State>(State(succ_idx[s], succ_coord[s]) );
        child_ptr->h = ENV.get_heur( child_ptr->coord, currNode_ptr->t + ENV.dt_);   // compute heuristic        
      }

      // store the hashkey
      currNode_ptr->succ_hashkey[succ_act_idx[s]] = succ_idx[s];
      currNode_ptr->succ_action_cost[succ_act_idx[s]] = succ_cost[s];

      child_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
      child_ptr->pred_action_cost.push_back(succ_cost[s]);
      child_ptr->pred_action_id.push_back(succ_act_idx[s]);

      nodes_ptr.push_back(child_ptr);
    }

    if(currNode_ptr->g > currNode_ptr->rhs) 
      currNode_ptr->g = currNode_ptr->rhs;
    else {
      currNode_ptr->g = std::numeric_limits<double>::infinity();
      nodes_ptr.push_back(currNode_ptr);
    }

    if((max_t > 0 && currNode_ptr->t >= max_t && !std::isinf(currNode_ptr->rhs)) || ENV.is_goal(currNode_ptr->coord)) {
      if(verbose_)
        printf(ANSI_COLOR_GREEN "MaxExpandTime [%f] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_t);
      break;
    }


    for(auto& it: nodes_ptr)
      sss_ptr->updateNode(it);

    if(max_expand > 0 && expand_iteration >= max_expand) {
      if(verbose_)
        printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
      return std::numeric_limits<double>::infinity();
    }

    if( sss_ptr->pq.empty()) {
      if(verbose_)
        printf(ANSI_COLOR_RED "Priority queue is empty!!!!!!\n\n" ANSI_COLOR_RESET);
      return std::numeric_limits<double>::infinity();
    }
  }

  if(verbose_) {
    //printf(ANSI_COLOR_GREEN "topKey: %f, goal g: %f, rhs: %f!\n" ANSI_COLOR_RESET, sss_ptr->pq.top().first, goalNode_ptr->g, goalNode_ptr->rhs);
    double fval = std::min(currNode_ptr->g, currNode_ptr->rhs) + sss_ptr->eps * currNode_ptr->h;
    printf(ANSI_COLOR_GREEN "currNode key: %f, g: %f, rhs: %f!\n" ANSI_COLOR_RESET, fval, currNode_ptr->g, currNode_ptr->rhs);
    printf(ANSI_COLOR_GREEN "Expand [%d] nodes!\n" ANSI_COLOR_RESET, expand_iteration);
  }

  if(currNode_ptr->iterationclosed && expand_iteration == 0 && sss_ptr->goalNode_ptr) {
    printf(ANSI_COLOR_GREEN "Recover directly!\n" ANSI_COLOR_RESET);
    currNode_ptr = sss_ptr->goalNode_ptr;
  }
  else {
    currNode_ptr->g = std::numeric_limits<double>::infinity();
    sss_ptr->updateNode(currNode_ptr);
  }

  sss_ptr->goalNode_ptr = currNode_ptr;
  // Recover trajectory
  double pcost = currNode_ptr->g;
  traj = recoverTraj(currNode_ptr, sss_ptr, ENV, start_idx);
  return pcost;  
}

 
Trajectory GraphSearch::recoverTraj(StatePtr currNode_ptr, std::shared_ptr<StateSpace> sss_ptr, const env_base& ENV, const Key& start_idx) {
  // Recover trajectory
  std::vector<Primitive> prs;
  while( !currNode_ptr->pred_hashkey.empty())
  {
    int min_id = -1;
    double min_gval = std::numeric_limits<double>::infinity();
    for(unsigned int i = 0; i < currNode_ptr->pred_hashkey.size(); i++) {
      Key key = currNode_ptr->pred_hashkey[i];
      if(min_gval > sss_ptr->hm[key]->g + currNode_ptr->pred_action_cost[i]) {
        min_gval = sss_ptr->hm[key]->g + currNode_ptr->pred_action_cost[i];
        min_id = i;
      }
    }

    if(min_id >= 0) {
      Key key = currNode_ptr->pred_hashkey[min_id];
      int action_idx = currNode_ptr->pred_action_id[min_id];
      currNode_ptr = sss_ptr->hm[key];
      Primitive pr;
      ENV.forward_action( currNode_ptr->coord, action_idx, pr );
      prs.push_back(pr);
      sss_ptr->best_child_.push_back(currNode_ptr);
      if(verbose_) {
        std::cout << currNode_ptr->t << std::endl;
        printf("action id: %d, action dt: %f\n", action_idx, pr.t());
      }
    }
    else 
      break;
    if(currNode_ptr->hashkey == start_idx)
      break;
  }

  std::reverse(prs.begin(), prs.end());
  std::reverse(sss_ptr->best_child_.begin(), sss_ptr->best_child_.end());
  return Trajectory(prs);
}
