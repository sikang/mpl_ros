#ifndef MP_BASE_UTIL_H
#define MP_BASE_UTIL_H

#include <planner/astar.h>
#include <planner/env_base.h>
#include <primitive/trajectory.h>

class MPBaseUtil
{
  public:
    MPBaseUtil();
    std::vector<Waypoint> getPath();
    Trajectory getTraj();
    std::vector<Primitive> getPrimitives();
    vec_Vec3f getPs();
 
    void setVmax(decimal_t v);
    void setAmax(decimal_t a);
    void setDt(decimal_t a);
    void setEpsilon(decimal_t eps);

    virtual bool plan(const Waypoint &start, const Waypoint &goal) = 0;

    // Debug objs
    std::vector<Primitive> primitives_;
    vec_Vec3f ps_;


    bool planner_verbose_;
    std::unique_ptr<mrsl::env_base> ENV_;
    std::vector<Waypoint> path_;
    decimal_t epsilon_ = 1.0;

};


#endif
