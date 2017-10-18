#include <planner/astar.h>
#include <planner/env_base.h>
#include <primitive/primitive.h>

using namespace MPL;

template <class state>
void ARAStateSpace<state>::getSubStateSpace(int ref_idx) {
  // Prune hashmap and priority queue
  hashMap<ARAState<state>> new_hm;
  priorityQueue<ARAState<state>> new_pq;

  for(auto& it: this->hm) {
    // Check if the node exist and it is not the goal
    if(it.second && it.second->parent_action_id != -1) {
      // Check if the node has already been added
      auto search = new_hm.find(it.first);
      if(search != new_hm.end()) 
        continue;
      // Check if the action idx matches the ref 
      if(!it.second->actions.empty() && it.second->actions.front() == ref_idx) {
        it.second->actions.pop_front();
        it.second->coord.t -= it.second->dt;
        if(it.second->actions.empty()) 
          it.second->parent = NULL;
        // Add current node
        // If it is in open set, add to pq
        if(it.second->iterationopened > it.second->iterationclosed) 
          it.second->heapkey = new_pq.push(std::make_pair((*it.second->heapkey).first, it.second));
        new_hm[it.second->hashkey] = it.second;
      }
    }
  }
  this->hm = new_hm;
  this->pq = new_pq;
}


template <class state>
double ARAStar<state>::Astar(const state& start_coord, Key start_idx,
    const env_base& ENV, std::shared_ptr<ARAStateSpace<state>> sss_ptr, 
    Trajectory& traj, int max_expand )
{
  traj.segs.clear();
  // Check if done
  if( ENV.is_goal(start_coord) )
    return 0;
  
  // Initialize start node
  std::shared_ptr<ARAState<state>> currNode_pt = sss_ptr->hm[start_idx];
  if( !currNode_pt ) {
    currNode_pt.reset( new ARAState<state>(start_idx, start_coord) );  
    currNode_pt->g = 0;
    currNode_pt->h = ENV.get_heur(start_coord);
    currNode_pt->iterationopened = sss_ptr->searchiteration;
    currNode_pt->iterationclosed = sss_ptr->searchiteration;
  }
  else {
    currNode_pt = sss_ptr->pq.top().second; 
    sss_ptr->pq.pop(); 
    currNode_pt->iterationclosed = sss_ptr->searchiteration; // Add to closed list
  }
   
  int expands = 0;
  bool reachedGoal = false;
  while(!reachedGoal)
  {
    expands++;
    if( ENV.is_goal(currNode_pt->coord) ) break;
    
    reachedGoal = spin( currNode_pt, sss_ptr, ENV ); // update heap
   
    bool reachMaxStep = (max_expand > 0 && expands >= max_expand);
    if(reachMaxStep)
      printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
    if( sss_ptr->pq.empty() || reachMaxStep)
      return std::numeric_limits<double>::infinity();

    // get element with smallest cost
    
    currNode_pt = sss_ptr->pq.top().second; sss_ptr->pq.pop(); 
    currNode_pt->iterationclosed = sss_ptr->searchiteration; // Add to closed list
  }
  
  // Recover trajectory
  double pcost = currNode_pt->g;
  std::vector<Primitive> prs;
  while( currNode_pt->parent)
  {
    int action_idx = currNode_pt->parent_action_id;
    currNode_pt = currNode_pt->parent;
    if(action_idx >= 0) {
      Primitive pr;
      ENV.forward_action( currNode_pt->coord, action_idx, currNode_pt->dt, pr );
      prs.push_back(pr);
      printf("action id: %d\n", action_idx);
      best_action_ = action_idx;
    }
  }

  std::reverse(prs.begin(), prs.end());
  traj = Trajectory(prs);
  return pcost;  
}


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

  //std::cout << "num succ=" << succ_coord.size() << std::endl;
  
  // Process successors
  for( unsigned s = 0; s < succ_coord.size(); ++s )
  {
    // Get child
    std::shared_ptr<ARAState<state>>& child_pt = sss_ptr->hm[ succ_idx[s] ];
    if( !(child_pt) )
    {
      //std::cout << "init child" << std::endl;
      child_pt.reset( new ARAState<state>(succ_idx[s], succ_coord[s]) );
      child_pt->h = ENV.get_heur( child_pt->coord );   // compute heuristic        
    }
   
    //see if we can improve the value of succstate
    //taking into account the cost of action
    double tentative_gval = currNode_pt->g + succ_cost[s];
   
    if( tentative_gval < child_pt->g )
    {
      child_pt->parent = currNode_pt;  // Assign new parent
      child_pt->actions = currNode_pt->actions;
      child_pt->actions.push_back(succ_act_idx[s]);
      child_pt->parent_action_id = succ_act_idx[s];
      child_pt->dt = succ_act_dt[s];
      child_pt->g = tentative_gval;    // Update gval

      double fval = child_pt->g + (sss_ptr->eps) * child_pt->h;
      //if it's set to goal directly, set fval = 0 to make sure it is on the top of pq
      if(succ_act_idx[s] < 0) {
        reached = true;
        fval = 0;
      }
      
      // if currently in OPEN, update
      if( child_pt->iterationopened > child_pt->iterationclosed)
      {
        //std::cout << "UPDATE fval(old) = " << (*child_pt->heapkey).first << std::endl;
        //std::cout << "UPDATE fval = " << fval << std::endl;
        //std::cout << "eps*h = " << (sss_ptr->eps) * child_pt->h << std::endl;
        (*child_pt->heapkey).first = fval;     // update heap element
        sss_ptr->pq.update(child_pt->heapkey);
        //sss_ptr->pq.increase( child_pt->heapkey );       // update heap
      }
      // if currently in CLOSED
      else if( child_pt->iterationclosed == sss_ptr->searchiteration)
      {
      }
      else // new node, add to heap
      {
        //std::cout << "ADD fval = " << fval << std::endl;
        child_pt->heapkey = sss_ptr->pq.push( std::make_pair(fval,child_pt) );
        child_pt->iterationopened = sss_ptr->searchiteration;
      }
    } //
  } // Process successors    

  return reached;
}


// explicit instantiations
template class MPL::ARAStateSpace<Waypoint>;
template class MPL::ARAStar<Waypoint>;

