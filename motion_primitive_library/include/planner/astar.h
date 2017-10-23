/**
 * @file astar.h
 * @brief backend of graph search, implemetation of A* and ARA*
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map
#include <array>                          // std::array
#include <list>                           // std::queue
#include <chrono>                         // std::chrono::high_resolution_clock
#include <primitive/trajectory.h>

namespace MPL
{
  ///Use `env' class
  class env_base;
  
  ///Key for hashmap
  typedef std::string Key;
  
  ///Heap element comparison
  template <class arastate>
  struct compare_pair
  {
    bool operator()(const std::pair<double,std::shared_ptr<arastate>>& p1, 
                    const std::pair<double,std::shared_ptr<arastate>>& p2) const
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
  template <class arastate>
  using hashMap = std::unordered_map<Key, std::shared_ptr<arastate> >;

  ///Define priority queue
  template <class arastate>
  using priorityQueue = boost::heap::d_ary_heap<std::pair<double,std::shared_ptr<arastate>>, boost::heap::mutable_<true>, boost::heap::arity<2>, boost::heap::compare< compare_pair<arastate> >>;
  
  struct ARAState
  {
    // location data
    Key hashkey;
    Waypoint coord;                            // discrete coordinates of this node
    double t;
    // hashkey of successors
    std::vector<std::pair<Key, double>> succ;
    // hashkey of predicessors
    std::vector<Key> pred_hashkey;
    std::vector<int> pred_action_id;
    std::vector<double> pred_action_cost;

    // pointer to heap location
    typename priorityQueue<ARAState>::handle_type heapkey;
    
    // plan data
    double g = std::numeric_limits<double>::infinity();
    double rhs = std::numeric_limits<double>::infinity();
    double h;
    bool iterationopened = false;
    bool iterationclosed = false;
    
    ARAState( Key hashkey, const Waypoint& coord )
      : hashkey(hashkey), coord(coord)//, parent(nullptr)
    {}

 };

  using ARAStatePtr = std::shared_ptr<ARAState>;
  
  ///State space
  struct ARAStateSpace
  {
    priorityQueue<ARAState> pq;
    hashMap<ARAState> hm;
    double eps;
    double dt;

    //double eps_satisfied = std::numeric_limits<double>::infinity();
    //const unsigned int searchiteration = 1;
    //bool use_il = false;    
    //bool reopen_nodes = false;

    ARAStateSpace(double eps = 1): eps(eps){}
    void getSubStateSpace(int id =1);
    void pruneStateSpace(std::vector<std::pair<Key, int> > states);
    void updateNode(ARAStatePtr currNode_ptr);

    std::vector<std::shared_ptr<ARAState>> best_child_;
  };

  
  /**
   * @brief ARAStar class
   *
   * Implement A* and ARA*
   */
  class ARAStar
  {
    public:
      /**
       * @brief Astar graph search
       *
       * @param start_coord start state
       * @param start_idx index for the start state 
       * @param ENV object of `env_base' class
       * @param sss_ptr workspace input
       * @param traj output trajectory
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       */
      double Astar(const Waypoint& start_coord, Key start_idx, const env_base& ENV, std::shared_ptr<ARAStateSpace> sss_ptr, 
          Trajectory& traj, int max_expand = -1, double max_t = 0);
      /*
      double ARAstar(const state& start_coord, Key start_idx, const env_base& ENV,
          Trajectory& traj, std::vector<int>& action_idx, double eps = 1,
          double allocated_time_secs = std::numeric_limits<double>::infinity() );
          */
  };
}
#endif
