#ifndef __ASTAR_NX_H_
#define __ASTAR_NX_H_

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map
#include <array>                          // std::array
#include <list>                           // std::list
#include <chrono>                         // std::chrono::high_resolution_clock

namespace MPL
{
  // Forward declaration
  //template <class state>
  class env_base;
  
  typedef std::string Key;
  
  // heap element comparison
  template <class arastate>
  struct compare_pair
  {
    bool operator()(const std::pair<double,std::shared_ptr<arastate>>& p1, 
                    const std::pair<double,std::shared_ptr<arastate>>& p2) const
    {
      if( (p1.first >= p2.first - 0.000001) && (p1.first <= p2.first +0.000001) )
      {
        // if equal compare gvals
        return (p1.second->g) < (p2.second->g);
      }
      return p1.first > p2.first;
    }
  };  

  template <class arastate>
  using hashMap = std::unordered_map<Key, std::shared_ptr<arastate> >;
  
  template <class arastate>
  using priorityQueue = boost::heap::d_ary_heap<std::pair<double,std::shared_ptr<arastate>>, boost::heap::mutable_<true>, boost::heap::arity<2>, boost::heap::compare< compare_pair<arastate> >>;
  
  // ARA state definition
  template <class state>
  struct ARAState
  {
    // location data
    state coord;                            // discrete coordinates of this node
    std::shared_ptr<ARAState<state>> parent; // pointer to parent node
    int parent_action_id = -1;
    Key hashkey;
    // pointer to heap location
    typename priorityQueue<ARAState<state>>::handle_type heapkey;
    
    // plan data
    double g = std::numeric_limits<double>::infinity();
    double h;
    unsigned int iterationopened = 0;
    unsigned int iterationclosed = 0;
    
    // ARA* info:
    double v = std::numeric_limits<double>::infinity();
    bool bInconsistent = false;
    //ARAState(){}
    
    ARAState( Key hashkey, const state& coord )
      : hashkey(hashkey), coord(coord)//, parent(nullptr)
    {}
  };
  
  template <class state>
  struct ARAStateSpace
  {
    std::list<std::shared_ptr<ARAState<state>>> il;
    priorityQueue<ARAState<state>> pq;
    hashMap<ARAState<state>> hm;
    
    double eps;
    double eps_satisfied = std::numeric_limits<double>::infinity();
    unsigned int searchiteration = 1;
    bool use_il = false;    
    bool reopen_nodes = false;

    ARAStateSpace(double eps = 1): eps(eps){}
  };
  
  template <class state>
  class ARAStar
  {
  public:
    double Astar( const state& start_coord, Key start_idx, const env_base& ENV,
                 std::list<state>& path, std::vector<int>& action_idx, double eps = 1 , int max_expand = -1);
    double ARAstar( const state& start_coord, Key start_idx, const env_base& ENV,
                    std::list<state>& path, std::vector<int>& action_idx, double eps = 1,
                    double allocated_time_secs = std::numeric_limits<double>::infinity() );
  private:
    void spin(const std::shared_ptr<ARAState<state>>& currNode_pt,
              std::shared_ptr<ARAStateSpace<state>>& sss_ptr,
              const env_base& ENV );
    void MoveInconsToOpen(std::shared_ptr<ARAStateSpace<state>>& sss_ptr);
    void ReevaluateFVals(std::shared_ptr<ARAStateSpace<state>>& sss_ptr);
    static double toc(std::chrono::high_resolution_clock::time_point& t2){
      return std::chrono::duration<double>( std::chrono::high_resolution_clock::now()
                                            - t2 ).count();
    }
  };
}
#endif
