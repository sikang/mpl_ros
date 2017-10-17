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
#include <list>                           // std::list
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
        return (p1.second->g) < (p2.second->g);
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
  
  ///ARA state definition
  template <class state>
  struct ARAState
  {
    // location data
    state coord;                            // discrete coordinates of this node
    std::shared_ptr<ARAState<state>> parent; // pointer to parent node
    int parent_action_id = -1;
    double dt = 1.0;
    Key hashkey;
    // pointer to heap location
    typename priorityQueue<ARAState<state>>::handle_type heapkey;
    
    // plan data
    double g = std::numeric_limits<double>::infinity();
    double h;
    unsigned int iterationopened = 0;
    unsigned int iterationclosed = 0;
    
    ARAState( Key hashkey, const state& coord )
      : hashkey(hashkey), coord(coord)//, parent(nullptr)
    {}
  };
  
  ///State space
  template <class state>
  struct ARAStateSpace
  {
    //std::list<std::shared_ptr<ARAState<state>>> il;
    priorityQueue<ARAState<state>> pq;
    hashMap<ARAState<state>> hm;
    
    double eps;
    //double eps_satisfied = std::numeric_limits<double>::infinity();
    const unsigned int searchiteration = 1;
    //bool use_il = false;    
    //bool reopen_nodes = false;

    ARAStateSpace(double eps = 1): eps(eps){}
  };
  
  /**
   * @brief ARAStar class
   *
   * Implement A* and ARA*
   */
  template <class state>
  class ARAStar
  {
    public:
      /**
       * @brief Astar graph search
       *
       * @param start_coord start state
       * @param start_idx index for the start state 
       * @param ENV object of `env_base' class
       * @param traj output trajectory
       * @param eps weight of heuristic, default value is 1
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       */
      double Astar(const state& start_coord, Key start_idx, const env_base& ENV,
          Trajectory& traj, double eps = 1 , int max_expand = -1, bool replan = false);
      /*
      double ARAstar(const state& start_coord, Key start_idx, const env_base& ENV,
          Trajectory& traj, std::vector<int>& action_idx, double eps = 1,
          double allocated_time_secs = std::numeric_limits<double>::infinity() );
          */
    private:
      bool spin(const std::shared_ptr<ARAState<state>>& currNode_pt,
          std::shared_ptr<ARAStateSpace<state>>& sss_ptr,
          const env_base& ENV );
      std::shared_ptr<ARAStateSpace<state>> sss_ptr;
  };
}
#endif
