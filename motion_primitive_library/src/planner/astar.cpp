#include <planner/astar.h>
#include <planner/env_base.h>
#include <primitive/primitive.h>

using namespace MPL;

template <class state>
void ARAStateSpace<state>::getSubStateSpace(int id) {
  if(best_child_.empty())
    return;

  std::shared_ptr<ARAState<state>> currNode_ptr = best_child_[id];
  currNode_ptr->parent = nullptr;

  hashMap<ARAState<state>> new_hm;
  priorityQueue<ARAState<state>> epq;
  currNode_ptr->heapkey = epq.push(std::make_pair(currNode_ptr->g, currNode_ptr));

  double init_g = currNode_ptr->g;
  for(auto& it: hm) {
    if(it.second) {
      it.second->g = std::numeric_limits<double>::infinity();
      it.second->iterationopened = 0;
      //it.second->iterationclosed = 0;
    }
  }

  currNode_ptr->g = init_g;
  while(!epq.empty()) {
    for(unsigned int i = 0; i < currNode_ptr->neighbors.size(); i++) {
      Key key = currNode_ptr->neighbors[i].first;
      if(key.empty())
        continue;
      auto search = hm.find(key);
      if(search == hm.end())
        continue;
      std::shared_ptr<ARAState<state>>& child_ptr = hm[key];
      if(!child_ptr)
        continue;
      double tentative_gval = currNode_ptr->g + currNode_ptr->neighbors[i].second;

      if(tentative_gval < child_ptr->g) {

        child_ptr->coord.t = currNode_ptr->coord.t + currNode_ptr->dt;
        child_ptr->parent = currNode_ptr;
        child_ptr->parent_action_id = i;
        child_ptr->g = tentative_gval;

        // if currently in OPEN, update
        if( child_ptr->iterationopened > 0)
        {
          (*child_ptr->heapkey).first = tentative_gval;     // update heap element
          epq.increase( child_ptr->heapkey );       // update heap
        }
        // if currently in CLOSED
        /*
        else if( child_ptr->iterationclosed == searchiteration)
        {
          printf(ANSI_COLOR_RED "ASTAR getSubStateSpace ERROR!\n" ANSI_COLOR_RESET);
        }
        */
        else {
          child_ptr->heapkey = epq.push( std::make_pair(tentative_gval, child_ptr) );
          child_ptr->iterationopened = searchiteration;
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
    if(it.second->iterationclosed == 0) {
      it.second->heapkey = pq.push( std::make_pair(it.second->g + eps * it.second->h, it.second) );
      it.second->iterationopened = searchiteration;
    }
  }
}

template <class state>
void ARAStateSpace<state>::pruneStateSpace(std::vector<std::shared_ptr<ARAState<state>> > states) {
  if(states.empty())
    return;
  for(const auto& affected_node: states) {
    hm[affected_node->hashkey] = nullptr;
  }

  getSubStateSpace(0);
}


template <class state>
double ARAStar<state>::Astar(const state& start_coord, Key start_idx,
    const env_base& ENV, std::shared_ptr<ARAStateSpace<state>> sss_ptr, 
    Trajectory& traj, int max_expand )
{
  traj.segs.clear();
  sss_ptr->best_child_.clear();
  // Check if done
  if( ENV.is_goal(start_coord) )
    return 0;
  
  // Initialize start node
  std::shared_ptr<ARAState<state>> currNode_pt = sss_ptr->hm[start_idx];
  
  int expands = 0;
  bool reachedGoal = false;
  while(!reachedGoal)
  {
    expands++;
    // get element with smallest cost
    if(sss_ptr->pq.empty()) {
      printf(ANSI_COLOR_GREEN "Start new node!\n" ANSI_COLOR_RESET);
      currNode_pt.reset( new ARAState<state>(start_idx, start_coord) );  
      currNode_pt->g = 0;
      currNode_pt->dt = ENV.get_dt();
      currNode_pt->h = ENV.get_heur(start_coord);
      currNode_pt->iterationopened = sss_ptr->searchiteration;
      currNode_pt->iterationclosed = sss_ptr->searchiteration;
    }
    else {
      // not close a node when it is close to the goal
      currNode_pt = sss_ptr->pq.top().second;     
      if( ENV.is_goal(currNode_pt->coord) ) { 
        //currNode_pt->coord.print();
        break;
      }
      sss_ptr->pq.pop(); 
      currNode_pt->iterationclosed = sss_ptr->searchiteration; // Add to closed list
    }

    
    reachedGoal = spin( currNode_pt, sss_ptr, ENV ); // update heap
   
    bool reachMaxStep = (max_expand > 0 && expands >= max_expand);
    if(reachMaxStep)
      printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
    if( sss_ptr->pq.empty() || reachMaxStep)
      return std::numeric_limits<double>::infinity();

 }

  printf(ANSI_COLOR_GREEN "Expand [%d] nodes!\n" ANSI_COLOR_RESET, expands);
  
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
      sss_ptr->best_child_.push_back(currNode_pt);
      std::cout << currNode_pt->coord.t << std::endl;
      printf("action id: %d, action dt: %f\n", action_idx, pr.t());
    }
  }

  std::reverse(prs.begin(), prs.end());
  std::reverse(sss_ptr->best_child_.begin(), sss_ptr->best_child_.end());
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

  currNode_pt->neighbors.resize(ENV.U_.size(), std::make_pair(Key(""), 0));
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
      if(ENV.is_goal(child_pt->coord))
        child_pt->h = 0;
      else
        child_pt->h = ENV.get_heur( child_pt->coord );   // compute heuristic        
    }

    // store the hashkey
    if(succ_act_idx[s] >= 0) {
      currNode_pt->neighbors[succ_act_idx[s]] = std::make_pair(succ_idx[s], succ_cost[s]);
    }
   
    //see if we can improve the value of succstate
    //taking into account the cost of action
    double tentative_gval = currNode_pt->g + succ_cost[s];
   
    if( tentative_gval < child_pt->g )
    {
      child_pt->parent = currNode_pt;  // Assign new parent
      child_pt->parent_action_id = succ_act_idx[s];
      child_pt->dt = succ_act_dt[s];
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
      if( child_pt->iterationopened > child_pt->iterationclosed)
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
      else if( child_pt->iterationclosed == sss_ptr->searchiteration)
      {
        //printf(ANSI_COLOR_RED "ASTAR ERROR!\n" ANSI_COLOR_RESET);
        child_pt->heapkey = sss_ptr->pq.push( std::make_pair(fval,child_pt) );
        child_pt->iterationopened = sss_ptr->searchiteration;
        child_pt->iterationclosed = 0;
      }
      else // new node, add to heap
      {
        //std::cout << "ADD fval = " << fval << std::endl;
        child_pt->heapkey = sss_ptr->pq.push( std::make_pair(fval, child_pt));
        child_pt->iterationopened = sss_ptr->searchiteration;
      }
    } //
  } // Process successors    

  return reached;
}


// explicit instantiations
template class MPL::ARAStateSpace<Waypoint>;
template class MPL::ARAStar<Waypoint>;

