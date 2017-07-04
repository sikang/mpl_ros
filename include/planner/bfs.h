#ifndef BFS_H
#define BFS_H
#include <stack>
#include <memory>
#include <primitive/primitive.h>
#include <collision_checking/voxel_map_util.h>

namespace MPL {
  class Graph {
    public:
      Graph();
      vec_Vec3f ps();
      std::vector<Waypoint> nodes();
      std::vector<Primitive> edges();
      void clear();
      void addNode(const Waypoint& node);
      void addEdge(const Primitive& pr);
      void info();
    private:
      std::vector<Waypoint> nodes_;
      std::vector<Primitive> edges_;
  };

  class BFS
  {
    public:
      BFS();
      vec_Vec3f getPs();
      std::vector<Primitive> getPrimitives();

      void setDt(decimal_t dt);
      void setMapUtil(std::shared_ptr<VoxelMapUtil> map_util);
      void setDiscretization(decimal_t u_max, int n, bool use_3d);

      void createGraph(const Waypoint& start, const Waypoint& goal, int num);
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
