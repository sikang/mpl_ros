/**
 * @file mp_base_util.h
 * @brief base class for mp_planner
 *
 * Base classes for planning
 */

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
    void setMaxNum(int num);
    void setMode(int n, bool use_3d);
    void setTol(decimal_t tol_dis, decimal_t tol_vel);

    virtual bool plan(const Waypoint &start, const Waypoint &goal) = 0;

    ///Debug pimitives
    std::vector<Primitive> primitives_;
    ///Stores the expanded nodes
    vec_Vec3f ps_;

    ///Env class
    std::unique_ptr<MPL::env_base> ENV_;
    std::vector<Waypoint> path_;
    decimal_t epsilon_ = 1.0;
    ///Maxmum number of expansion allowd, -1 means no limitation
    int max_num_ = -1;

    ///Enabled to display debug message
    bool planner_verbose_;
};


#endif
