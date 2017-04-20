#include <planner/astar.h>
#include <planner/env_sfc.h>
#include <primitive/trajectory.h>

class MPSFCUtil
{
  public:
    MPSFCUtil(bool verbose);
    bool plan(const Waypoint &start, const Waypoint &goal);
    std::vector<Waypoint> getPath();
    Trajectory getTraj();
    void setVmax(decimal_t v);
    void setAmax(decimal_t a);
    void setDt(decimal_t a);
    void setEpsilon(decimal_t eps);
    void setMap(const Polyhedra& poly);

    // Debug objs
    std::vector<Primitive> primitives_;
    vec_Vec3f ps_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 protected:
    bool planner_verbose_;
    std::unique_ptr<mrsl::env_sfc> ENV_;
    std::vector<Waypoint> path_;
    decimal_t epsilon_ = 1.0;

};


