#include <planner/astar.h>
#include <planner/env_base.h>
#include <primitive/primitive.h>

using namespace MPL;

/*
void ARAStateSpace::getSubStateSpace(int id) {
  if(best_child_.empty())
    return;

  ARAStatePtr currNode_ptr = best_child_[id];
  currNode_ptr->parent = nullptr;

  hashMap<ARAState> new_hm;
  priorityQueue<ARAState> epq;
  currNode_ptr->heapkey = epq.push(std::make_pair(currNode_ptr->g, currNode_ptr));

  double init_g = currNode_ptr->g;
  for(auto& it: hm) {
    if(it.second) {
      it.second->g = std::numeric_limits<double>::infinity();
      it.second->iterationopened = false;
      //it.second->iterationclosed = 0;
    }
  }

  currNode_ptr->g = init_g;
  while(!epq.empty()) {
    for(unsigned int i = 0; i < currNode_ptr->succ.size(); i++) {
      Key key = currNode_ptr->succ[i].first;
      if(key.empty())
        continue;
      auto search = hm.find(key);
      if(search == hm.end())
        continue;
      ARAStatePtr& child_ptr = hm[key];
      if(!child_ptr)
        continue;
      double tentative_gval = currNode_ptr->g + currNode_ptr->succ[i].second;

      if(tentative_gval < child_ptr->g) {

        child_ptr->coord.t = currNode_ptr->coord.t + currNode_ptr->dt;
        child_ptr->parent = currNode_ptr;
        child_ptr->parent_action_id = i;
        child_ptr->g = tentative_gval;

        // if currently in OPEN, update
        if( child_ptr->iterationopened)
        {
          (*child_ptr->heapkey).first = tentative_gval;     // update heap element
          epq.increase( child_ptr->heapkey );       // update heap
        }
        // if currently in CLOSED
        else if( child_ptr->iterationclosed == searchiteration)
        {
          printf(ANSI_COLOR_RED "ASTAR getSubStateSpace ERROR!\n" ANSI_COLOR_RESET);
        }
        else {
          child_ptr->heapkey = epq.push( std::make_pair(tentative_gval, child_ptr) );
          child_ptr->iterationopened = true;
        }
      }
    }

    currNode_ptr = epq.top().second; epq.pop();
    //currNode_ptr->iterationclosed = searchiteration;
    new_hm[currNode_ptr->hashkey] = currNode_ptr;
  }

  hm = new_hm;

  pq.clear();
  for(auto& it: hm) {
    if(id > 0)
      it.second->coord.t -= best_child_[id]->dt;
    if(!it.second->iterationclosed) {
      it.second->heapkey = pq.push( std::make_pair(it.second->g + eps * it.second->h, it.second) );
      it.second->iterationopened = true;
    }
  }
}
*/

void ARAStateSpace::pruneStateSpace(std::vector<std::pair<Key, int> > states) {
  if(states.empty())
    return;
  for(const auto& affected_node: states) {
    int id = affected_node.second;
    hm[affected_node.first]->pred_action_cost[id] = std::numeric_limits<double>::infinity();
    updateNode(hm[affected_node.first]);
  }

  //getSubStateSpace(0);
}

double ARAStar::Astar(const Waypoint& start_coord, Key start_idx, 
    const env_base& ENV, std::shared_ptr<ARAStateSpace> sss_ptr, 
    Trajectory& traj, int max_expand, double max_t)
{
  traj.segs.clear();
  sss_ptr->best_child_.clear();
  // Check if done
  if( ENV.is_goal(start_coord) )
    return 0;
  
  // Initialize start node
  ARAStatePtr currNode_ptr = sss_ptr->hm[start_idx];
  if(sss_ptr->pq.empty()) {
    printf(ANSI_COLOR_GREEN "Start new node!\n" ANSI_COLOR_RESET);
    currNode_ptr = std::make_shared<ARAState>(ARAState(start_idx, start_coord));
    currNode_ptr->t = 0;
    currNode_ptr->g = std::numeric_limits<double>::infinity();
    currNode_ptr->rhs = 0;
    currNode_ptr->h = ENV.get_heur(start_coord);
    double fval = std::min(currNode_ptr->g, currNode_ptr->rhs) + currNode_ptr->h;
    currNode_ptr->heapkey = sss_ptr->pq.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
    sss_ptr->hm[start_idx] = currNode_ptr;
  }
  // Initialize null goal node
  ARAStatePtr goalNode_ptr= std::make_shared<ARAState>(ARAState(Key(), Waypoint()));

  int expand_iteration = 0;
  while(sss_ptr->pq.top().first < std::min(goalNode_ptr->g, goalNode_ptr->rhs) || goalNode_ptr->rhs != goalNode_ptr->g)
  {
    expand_iteration++;
    // not close a node when it is close to the goal
    currNode_ptr = sss_ptr->pq.top().second;     
    if( ENV.is_goal(currNode_ptr->coord) ) break;
    sss_ptr->pq.pop(); 
    currNode_ptr->iterationclosed = true; // Add to closed list

    // Get successors
    std::vector<Waypoint> succ_coord;
    std::vector<MPL::Key> succ_idx;
    std::vector<double> succ_cost;
    std::vector<int> succ_act_idx;

    ENV.get_succ( currNode_ptr->coord, succ_coord, succ_idx, succ_cost, succ_act_idx);
    currNode_ptr->succ.resize(ENV.U_.size(), std::make_pair(Key(""), 0));

    //printf("[%d] expand, g: %f, h: %f\n", expands, currNode_ptr->g, currNode_ptr->h);

    std::vector<ARAStatePtr> nodes_ptr;
    // Process successors
    for( unsigned s = 0; s < succ_coord.size(); ++s )
    {
      // Get child
      ARAStatePtr& child_ptr = sss_ptr->hm[ succ_idx[s] ];
      if( !(child_ptr) ) {
        child_ptr = std::make_shared<ARAState>(ARAState(succ_idx[s], succ_coord[s]) );
        child_ptr->h = ENV.get_heur( child_ptr->coord );   // compute heuristic        
      }

      if(ENV.is_goal(child_ptr->coord)) {
        child_ptr->h = 0;
        printf("Find goal!\n");
        goalNode_ptr = child_ptr;
      }

      // store the hashkey
      currNode_ptr->succ[succ_act_idx[s]] = std::make_pair(succ_idx[s], succ_cost[s]);
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

    for(auto& it: nodes_ptr)
      sss_ptr->updateNode(it);

    if(max_t > 0 && currNode_ptr->t >= max_t) {
      printf(ANSI_COLOR_GREEN "MaxExpandTime [%f] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_t);
      break;
    }

    if(max_expand > 0 && expand_iteration >= max_expand) {
      printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
      return std::numeric_limits<double>::infinity();
    }

    if( sss_ptr->pq.empty()) {
      printf(ANSI_COLOR_RED "Priority queue is empty!!!!!!\n\n" ANSI_COLOR_RESET);
      return std::numeric_limits<double>::infinity();
    }
  }

  printf(ANSI_COLOR_GREEN "topKey: %f, goal g: %f, rhs: %f!\n" ANSI_COLOR_RESET, sss_ptr->pq.top().first, goalNode_ptr->g, goalNode_ptr->rhs);
  printf(ANSI_COLOR_GREEN "Expand [%d] nodes!\n" ANSI_COLOR_RESET, expand_iteration);
  
  // Recover trajectory
  double pcost = currNode_ptr->g;
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
      std::cout << currNode_ptr->t << std::endl;
      printf("action id: %d, action dt: %f\n", action_idx, pr.t());
    }
    else 
      break;
    if(currNode_ptr->hashkey == start_idx)
      break;
  }

  std::reverse(prs.begin(), prs.end());
  std::reverse(sss_ptr->best_child_.begin(), sss_ptr->best_child_.end());
  traj = Trajectory(prs);
  return pcost;  
}

 
void ARAStateSpace::updateNode(ARAStatePtr currNode_ptr) {
  double parent_t = 0;
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

  //printf("curr g: %f, rhs: %f\n", currNode_ptr->g, currNode_ptr->rhs);
  if(std::isinf(currNode_ptr->rhs)) 
    return;

  // if currNode is in openset, remove it
  if(currNode_ptr->iterationopened && !currNode_ptr->iterationclosed )
    pq.erase(currNode_ptr->heapkey);

  // if currNode's g value is not equal to its rhs, put it into openset
  if(currNode_ptr->g != currNode_ptr->rhs) {
    double fval = std::min(currNode_ptr->g, currNode_ptr->rhs) + currNode_ptr->h;
    
    //printf("curr g: %f, rhs: %f, h: %f, f: %f\n", 
    //    currNode_ptr->g, currNode_ptr->rhs, currNode_ptr->h, fval);
    currNode_ptr->heapkey = pq.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
    currNode_ptr->t = parent_t + dt;
  }

}
/*
template <class state>
bool ARAStar<state>::spin( const std::shared_ptr<ARAState<state>>& currNode_pt,
    std::shared_ptr<ARAStateSpace<state>>& sss_ptr,
    const env_base& ENV )
{
  // Get successors
  std::vector<state> succ_coord;
  std::vector<MPL::Key> succ_idx;
  std::vector<double> succ_cost;
  std::vector<int> succ_act_idx;
  std::vector<double> succ_act_dt;
  bool reached = false;

  ENV.get_succ( currNode_pt->coord, succ_coord, succ_idx, succ_cost, succ_act_idx, succ_act_dt);

  currNode_pt->succ.resize(ENV.U_.size(), std::make_pair(Key(""), 0));
  //std::cout << "num succ=" << succ_coord.size() << std::endl;
  
  // Process successors
  for( unsigned s = 0; s < succ_coord.size(); ++s )
  {
    // Get child
    std::shared_ptr<ARAState<state>>& child_pt = sss_ptr->hm[ succ_idx[s] ];
    if( !(child_pt) )
    {
      child_pt.reset( new ARAState<state>(succ_idx[s], succ_coord[s]) );
      if(ENV.is_goal(child_pt->coord))
        child_pt->h = 0;
      else
        child_pt->h = ENV.get_heur( child_pt->coord );   // compute heuristic        
    }

    // store the hashkey
    if(succ_act_idx[s] >= 0) {
      currNode_pt->succ[succ_act_idx[s]] = std::make_pair(succ_idx[s], succ_cost[s]);
      child_pt->pred_hashkey.push_back(currNode_pt->hashkey);
      child_pt->pred_action_cost.push_back(succ_cost[s]);
      child_pt->pred_action_id.push_back(succ_act_idx[s]);
    }
   
    //see if we can improve the value of succstate
    //taking into account the cost of action
    double tentative_gval = currNode_pt->g + succ_cost[s];
   
    if( tentative_gval < child_pt->g )
    {
      child_pt->parent = currNode_pt;  // Assign new parent
      child_pt->coord.t = currNode_pt->coord.t + succ_act_dt[s];
      child_pt->g = tentative_gval;    // Update gval

      double fval = child_pt->g + (sss_ptr->eps) * child_pt->h;
      //if it's set to goal directly, dont add to pq
      if(succ_act_idx[s] < 0) {
        reached = true;
        continue;
        //fval = 0;
      }
      
      // if currently in OPEN, update
      if( child_pt->iterationopened && !child_pt->iterationclosed)
      {
        if((*child_pt->heapkey).first < fval) {
          std::cout << "UPDATE fval(old) = " << (*child_pt->heapkey).first << std::endl;
          std::cout << "UPDATE fval = " << fval << std::endl;
        }

        (*child_pt->heapkey).first = fval;     // update heap element
        //sss_ptr->pq.update(child_pt->heapkey);
        sss_ptr->pq.increase( child_pt->heapkey );       // update heap
      }
      // if currently in CLOSED, reopen the node
      else // new node, add to heap
      {
        //std::cout << "ADD fval = " << fval << std::endl;
        child_pt->heapkey = sss_ptr->pq.push( std::make_pair(fval, child_pt));
        child_pt->iterationopened = true;
        child_pt->iterationclosed = false;
      }
    } //
  } // Process successors    

  return reached;
}
*/


// explicit instantiations
//template class MPL::ARAStateSpace<Waypoint>;
//template class MPL::ARAStar<Waypoint>;

