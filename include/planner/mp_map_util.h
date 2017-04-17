#include <planner/astar.h>
#include <planner/env_map.h>
#include <primitive/trajectory.h>

class MPMapUtil
{
  public:
    MPMapUtil(bool verbose);
    bool plan(const Waypoint &start, const Waypoint &goal);
    std::vector<Waypoint> getPath();
    Trajectory getTraj();
    void setVmax(decimal_t v);
    void setAmax(decimal_t a);
    void setDt(decimal_t a);
    void setEpsilon(decimal_t eps);
    void setMapUtil(std::shared_ptr<VoxelMapUtil> map_util);

    // Debug objs
    std::vector<Primitive> primitives_;
    vec_Vec3f ps_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 protected:
    bool _planner_verbose;
    std::unique_ptr<mrsl::env_map> ENV_;
    std::vector<Waypoint> _path;
    decimal_t _epsilon = 1.0;

};


