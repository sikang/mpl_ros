/**
 * @file graph_search.h
 * @brief backend of graph search, implemetation of A* and Lifelong Planning A*
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map
#include <array>                          // std::array
#include <primitive/trajectory.h>

namespace MPL
{
  ///Declare `env' class
  class env_base;

  ///Key for hashmap
  typedef std::string Key;
  
  ///Heap element comparison
  template <class state>
  struct compare_pair
  {
    bool operator()(const std::pair<double,std::shared_ptr<state>>& p1, 
                    const std::pair<double,std::shared_ptr<state>>& p2) const
    {
      if( (p1.first >= p2.first - 0.000001) && (p1.first <= p2.first + 0.000001) )
      {
        // if equal compare gvals
        return std::min(p1.second->g, p1.second->rhs) > std::min(p2.second->g, p2.second->rhs);
      }
      return p1.first > p2.first;
    }
  };  

  ///Define hashmap type
  template <class state>
  using hashMap = std::unordered_map<Key, std::shared_ptr<state> >;

  ///Define priority queue
  template <class state>
  using priorityQueue = boost::heap::d_ary_heap<std::pair<double,std::shared_ptr<state>>, boost::heap::mutable_<true>, boost::heap::arity<2>, boost::heap::compare< compare_pair<state> >>;
  
  ///Lattice of the graph in graph search
  struct State
  {
    // location data
    Key hashkey;
    Waypoint coord;                            // discrete coordinates of this node
    double t;
    // hashkey of successors
    std::vector<Key> succ_hashkey;
    std::vector<double> succ_action_cost;
    // hashkey of predicessors
    std::vector<Key> pred_hashkey;
    std::vector<int> pred_action_id;
    std::vector<double> pred_action_cost;

    // pointer to heap location
    typename priorityQueue<State>::handle_type heapkey;
    
    // plan data
    double g = std::numeric_limits<double>::infinity();
    double rhs = std::numeric_limits<double>::infinity();
    double h;
    bool iterationopened = false;
    bool iterationclosed = false;
    
    State( Key hashkey, const Waypoint& coord )
      : hashkey(hashkey), coord(coord)//, parent(nullptr)
    {}

 };

  using StatePtr = std::shared_ptr<State>;
  
  ///State space
  struct StateSpace
  {
    priorityQueue<State> pq;
    hashMap<State> hm;
    double eps;
    double dt;

    StateSpace(double eps = 1): eps(eps){}
    void getSubStateSpace(int id =1);
    void increaseCost(std::vector<std::pair<Key, int> > states);
    void decreaseCost(std::vector<std::pair<Key, int> > states, const env_base& ENV);
    void updateNode(StatePtr currNode_ptr);

    std::vector<std::shared_ptr<State>> best_child_;
    StatePtr goalNode_ptr = nullptr;
  };

  
  /**
   * @brief GraphSearch class
   *
   * Implement A* and Lifelong Planning A*
   */
  class GraphSearch
  {
    public:
      /**
       * @brief Simple empty constructor
       *
       * @param verbose enable print out infos, default is set to False
       */
      GraphSearch(bool verbose = false) : verbose_(verbose) {};

      /**
       * @brief Astar graph search
       *
       * @param start_coord start state
       * @param start_idx index for the start state 
       * @param ENV object of `env_base' class
       * @param sss_ptr workspace input
       * @param traj output trajectory
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       * @param max_t max time horizon of expanded states, default value is -1 which means there is no limitation
       */
      double Astar(const Waypoint& start_coord, Key start_idx, const env_base& ENV, std::shared_ptr<StateSpace> sss_ptr, 
          Trajectory& traj, int max_expand = -1, double max_t = 0);
      /**
       * @brief Lifelong Planning Astar graph search
       *
       * @param start_coord start state
       * @param start_idx index for the start state 
       * @param ENV object of `env_base' class
       * @param sss_ptr workspace input
       * @param traj output trajectory
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       * @param max_t max time horizon of expanded states, default value is -1 which means there is no limitation
       */
      double LPAstar(const Waypoint& start_coord, Key start_idx, const env_base& ENV, std::shared_ptr<StateSpace> sss_ptr, 
          Trajectory& traj, int max_expand = -1, double max_t = 0);
    private:
      ///Recover trajectory 
      Trajectory recoverTraj(StatePtr ptr, std::shared_ptr<StateSpace> sss_ptr, const env_base& ENV, const Key& start_idx);
      bool verbose_ = false;
 
 };
}
#endif
