#ifndef __ENV_INT_H_
#define __ENV_INT_H_

#include <vector>                           // std::vector
namespace mrsl
{

template <class state>
class env_int
{
public:
  env_int(){}
  ~env_int(){}
    
  virtual bool is_goal(const state&) const
  {
    return true;
  }
  
  virtual void get_succ( const state& curr, 
                         std::vector<state>& succ,
                         std::vector<int>& succ_idx,
                         std::vector<double>& succ_cost,
                         std::vector<int>& action_idx ) const
  {
    succ.push_back(curr);
    succ_idx.push_back( state_to_idx(curr) );
    succ_cost.push_back(0);
    action_idx.push_back(0);
  }
  
  virtual double get_heur(const state&) const
  {
    return 0;
  }

  // apply action to current state and return the micro states along the resulting path
  virtual void forward_action( const state& curr, int action_id, std::vector<state>& next_micro ) const
  {
    next_micro.push_back( curr );
  }

  virtual int state_to_idx(const state&) const
  {
    return 0;
  }

};

}


#endif
