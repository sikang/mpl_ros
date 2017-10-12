#include <planner/astar.h>
#include <planner/env_base.h>
#include <primitive/primitive.h>


template <class state>
double MPL::ARAStar<state>::Astar(const state& start_coord, MPL::Key start_idx,
                                  const env_base& ENV,
                                  Trajectory& traj, 
                                  double eps, int max_expand )
{
  // Check if done
  if( ENV.is_goal(start_coord) )
    return 0;
  // Initialize State Space
  std::shared_ptr<ARAStateSpace<state>> sss_ptr( new ARAStateSpace<state>(eps) );
  
  // Initialize start node
  std::shared_ptr<ARAState<state>> currNode_pt = sss_ptr->hm[start_idx];
  currNode_pt.reset( new ARAState<state>(start_idx, start_coord) );  
  currNode_pt->g = 0;
  currNode_pt->v = currNode_pt->g;  
  currNode_pt->h = ENV.get_heur(start_coord);
  currNode_pt->iterationopened = sss_ptr->searchiteration;
  currNode_pt->iterationclosed = sss_ptr->searchiteration;

  int expands = 0;
  while(true)
  {
    expands++;
    if( ENV.is_goal(currNode_pt->coord) ) break;
    
    spin( currNode_pt, sss_ptr, ENV ); // update heap
    
    bool reachMaxStep = (max_expand > 0 && expands >= max_expand);
    if(reachMaxStep)
      printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
    if( sss_ptr->pq.empty() || reachMaxStep)
      return std::numeric_limits<double>::infinity();

    // get element with smallest cost
    currNode_pt = sss_ptr->pq.top().second; sss_ptr->pq.pop(); 
    currNode_pt->v = currNode_pt->g;
    currNode_pt->iterationclosed = sss_ptr->searchiteration; // Add to closed list
  }
  
  // Recover path
  double pcost = currNode_pt->g;
  std::vector<Primitive> prs;
  while( currNode_pt->parent )
  {
    //action_idx.push_back( currNode_pt->parent_action_id );
    int action_idx = currNode_pt->parent_action_id;
    currNode_pt = currNode_pt->parent;
    std::vector<state> next_micro;
    Primitive pr;
    ENV.forward_action( currNode_pt->coord, action_idx, currNode_pt->dt, next_micro, pr );
    prs.push_back(pr);
    for( typename std::vector<state>::reverse_iterator it = next_micro.rbegin(); 
         it!=next_micro.rend(); ++it ) {}
      //path.push_front( *it );
  }

  std::reverse(prs.begin(), prs.end());
  traj = Trajectory(prs);
  return pcost;  
}


template <class state>
double MPL::ARAStar<state>::ARAstar( const state& start_coord, MPL::Key start_idx,
                                    const env_base& ENV,
                                    Trajectory& traj, std::vector<int>& action_idx,
                                    double eps, double allocated_time_secs )
{
  std::chrono::high_resolution_clock::time_point time_started = std::chrono::high_resolution_clock::now();
  int searchexpands = 0;
  int prevexpands = 0;
  double eps_final = 1;
  double eps_dec = 0.2;

  // Check if done
  if( ENV.is_goal(start_coord) ) 
    return 0;

  // Initialize State Space
  std::shared_ptr<ARAStateSpace<state>> sss_ptr( new ARAStateSpace<state>(eps) );
  sss_ptr->use_il = true;
  std::shared_ptr<ARAState<state>> currNode_pt = sss_ptr->hm[start_idx];



  //the main loop of ARA*
  double pcost = std::numeric_limits<double>::infinity();
  std::chrono::high_resolution_clock::time_point loop_time;
  while( sss_ptr->eps_satisfied > eps_final+0.00001 &&
      toc(time_started) < allocated_time_secs )
  {
    loop_time = std::chrono::high_resolution_clock::now();

    //decrease eps for all subsequent iterations
    if (std::abs(sss_ptr->eps_satisfied - sss_ptr->eps) < 0.00001) 
    {
      sss_ptr->eps = std::max(sss_ptr->eps - eps_dec,eps_final);
      ReevaluateFVals(sss_ptr);   //the priorities need to be updated
      MoveInconsToOpen(sss_ptr);  // starting a new search
    }

    //improve or compute path
    int expands = 0;
    while( toc(time_started) < allocated_time_secs )
    {
      expands++;
      // Initialize Start Node
      if( !currNode_pt )
      {
        currNode_pt.reset( new ARAState<state>(start_idx, start_coord) );
        currNode_pt->g = 0;
        currNode_pt->h = ENV.get_heur(start_coord);
        currNode_pt->iterationopened = sss_ptr->searchiteration;      
      }
      else
        currNode_pt = sss_ptr->pq.top().second;

      // Check if done
      if( ENV.is_goal(currNode_pt->coord) ) 
      { // Done! Recover path
        //path.clear(); 
        action_idx.clear();
        pcost = currNode_pt->g;
        std::vector<Primitive> prs;
        while( currNode_pt->parent )
        {
          action_idx.push_back( currNode_pt->parent_action_id );
          double dt = currNode_pt->dt;
          currNode_pt = currNode_pt->parent;
          std::vector<state> next_micro;
          Primitive pr;
          ENV.forward_action( currNode_pt->coord, action_idx.back(), dt, next_micro, pr);

          for( typename std::vector<state>::reverse_iterator it = next_micro.rbegin(); 
              it!=next_micro.rend(); ++it ) {}
          //path.push_front( *it );
        }

        std::reverse(prs.begin(), prs.end());
        Trajectory traj(prs);
        break;
      }

      // Add to closed list
      if( !sss_ptr->pq.empty() )
        sss_ptr->pq.pop();
      currNode_pt->v = currNode_pt->g;
      currNode_pt->iterationclosed = sss_ptr->searchiteration;

      // Update heap
      spin( currNode_pt, sss_ptr, ENV ); 

      if( sss_ptr->pq.empty() ) // something went wrong
        return std::numeric_limits<double>::infinity();
    }
    sss_ptr->eps_satisfied = sss_ptr->eps;
    searchexpands += expands;

    /*
    //print the solution cost and eps bound
    std::cout << "eps= " << sss_ptr->eps_satisfied << " "
    << "expands= " << searchexpands - prevexpands << " "
    << "g(goal)= " << pcost << " "
    << "time= " << toc(loop_time) << " "
    << "time(total)= " << toc(time_started) << std::endl;
    */
    prevexpands = searchexpands;
    sss_ptr->searchiteration++;
  }
  return pcost;
}


template <class state>
void MPL::ARAStar<state>::spin( const std::shared_ptr<ARAState<state>>& currNode_pt,
    std::shared_ptr<ARAStateSpace<state>>& sss_ptr,
    const env_base& ENV )
{
  // Get successors
  std::vector<state> succ_coord;
  std::vector<MPL::Key> succ_idx;
  std::vector<double> succ_cost;
  std::vector<int> succ_act_idx;
  std::vector<double> succ_act_dt;
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
    double tentative_gval = currNode_pt->v + succ_cost[s];
    
    //std::cout << "tentative_gval= " << tentative_gval << std::endl;
    //std::cout << "child_pt->g= " << child_pt->g << std::endl;
    if( tentative_gval < child_pt->g )
    {
      child_pt->parent = currNode_pt;  // Assign new parent
      child_pt->parent_action_id = succ_act_idx[s];
      child_pt->dt = succ_act_dt[s];
      child_pt->g = tentative_gval;    // Update gval
      double fval = child_pt->g + (sss_ptr->eps) * child_pt->h;
      
      // if currently in OPEN, update
      if( child_pt->iterationopened > child_pt->iterationclosed)
      {
        //std::cout << "UPDATE fval(old) = " << (*child_pt->heapkey).first << std::endl;
        //std::cout << "UPDATE fval = " << fval << std::endl;
        //std::cout << "eps*h = " << (sss_ptr->eps) * child_pt->h << std::endl;
        (*child_pt->heapkey).first = fval;     // update heap element
        sss_ptr->pq.increase( child_pt->heapkey );       // update heap
      }
      // if currently in CLOSED
      else if( child_pt->iterationclosed == sss_ptr->searchiteration)
      {
        if(sss_ptr->reopen_nodes) // reopen node
        {
          //std::cout << "REOPEN" << std::endl;
          child_pt->heapkey = sss_ptr->pq.push( std::make_pair(fval,child_pt) );
          child_pt->iterationclosed = 0;
        }
        else if( sss_ptr->use_il && !child_pt->bInconsistent ) // inconsistent node
        {
          //std::cout << "INCONS" << std::endl;
          sss_ptr->il.push_back(child_pt);
          child_pt->bInconsistent = true;
        }
      }
      else // new node, add to heap
      {
        //std::cout << "ADD fval = " << fval << std::endl;
        child_pt->heapkey = sss_ptr->pq.push( std::make_pair(fval,child_pt) );
        child_pt->iterationopened = sss_ptr->searchiteration;
      }
    } //
  } // Process successors    
}




/*
 * Private stuff
 */ 
template <class state>
void MPL::ARAStar<state>::MoveInconsToOpen(std::shared_ptr<ARAStateSpace<state>>& sss_ptr)
{
  while( !sss_ptr->il.empty() )
  {
    double fval = sss_ptr->il.front()->g + sss_ptr->eps * sss_ptr->il.front()->h;

    // insert element in OPEN
    //std::cout << "INCONS fval=" << fval << std::endl;
    sss_ptr->il.front()->heapkey = sss_ptr->pq.push(std::make_pair(fval,sss_ptr->il.front()));
    sss_ptr->il.front()->iterationopened = sss_ptr->searchiteration;
    sss_ptr->il.front()->bInconsistent = false;
    
    // remove element from INCONS
    sss_ptr->il.pop_front();    
  }
}


template <class state>
void MPL::ARAStar<state>::ReevaluateFVals(std::shared_ptr<ARAStateSpace<state>>& sss_ptr)
{
  //recompute priorities for states in OPEN and reorder it
  priorityQueue<ARAState<state>> new_pq;
  for( typename priorityQueue<ARAState<state>>::ordered_iterator it = sss_ptr->pq.ordered_begin();
       it != sss_ptr->pq.ordered_end(); ++it )
  {
    double new_fval = (*it).second->g + sss_ptr->eps * (*it).second->h;
    (*it).second->heapkey = new_pq.push( std::make_pair( new_fval, (*it).second) );
    
    //std::cout << "REEVAL fval=" << new_fval << std::endl;
  }
  sss_ptr->pq = std::move(new_pq);
}



// explicit instantiations
template class MPL::ARAStar<Waypoint>;

