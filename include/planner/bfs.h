/**
 * @file bfs.h
 * @brief Breadth first search class
 */
#ifndef BFS_H
#define BFS_H
#include <stack>
#include <memory>
#include <primitive/primitive.h>
#include <collision_checking/voxel_map_util.h>

namespace MPL {
  ///Search graph build from BFS
  class Graph {
    public:
      ///Simple constructor
      Graph();
      ///Expanded points
      vec_Vec3f ps();
      ///Expanded nodes
      std::vector<Waypoint> nodes();
      ///Expanded edges
      std::vector<Primitive> edges();
      ///Clear the graph
      void clear();
      ///Insert node
      void addNode(const Waypoint& node);
      ///Insert edge
      void addEdge(const Primitive& pr);
      ///Print info
      void info();
    private:
      std::vector<Waypoint> nodes_;
      std::vector<Primitive> edges_;
  };

  /**
   * @brief Breadth First Search (BFS) class
   *
   * Currently still under testing, only works for voxel map
   */
  class BFS
  {
    public:
      ///Simple constructor
      BFS();
      ///Get expanded points
      vec_Vec3f getPs();
      ///Get the edges in graph
      std::vector<Primitive> getPrimitives();

      ///Set dt for each primitive
      void setDt(decimal_t dt);
      ///Set map util for collision checking
      void setMapUtil(std::shared_ptr<VoxelMapUtil> map_util);
      /**
       * @brief Set discretization params
       * @param u_max Max control input
       * @param n Number of discrete controls in semi-axis
       * @param use_3d If true, expand in 3D 
       */
      void setDiscretization(decimal_t u_max, int n, bool use_3d);

      /**
       * @brief Construct the graph
       * @param start Origin node in graph
       * @param goal Used for setting value function
       * @param num Finite time horizon
       */
      void createGraph(const Waypoint& start, const Waypoint& goal, int num);
      ///Get the optimal trajectory in graph
      std::vector<Primitive> recoverTraj(int num);
    private:
      bool isFree(const Primitive& p);
      decimal_t collision(const Waypoint& node);
      decimal_t heuristic(const Waypoint& node, const Waypoint& goal);

      bool planner_verbose_;
      decimal_t dt_ = 1.0;
      decimal_t v_max_ = 10;
      decimal_t w_ = 10;
      decimal_t w_collision_ = 20;
      vec_Vec3f U_;
      Graph graph_;
      std::shared_ptr<VoxelMapUtil> map_util_;
      vec_Vec3i collision_neighbors_;

  };
}
#endif
