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

/**
 * @brief Motion primitive base util class
 */
class MPBaseUtil
{
  public:
    ///Simple constructor
    MPBaseUtil();
    ///Get nodes on the optimal trajectory
    std::vector<Waypoint> getWs();
    ///Get optimal trajectory
    Trajectory getTraj();
    ///Get expanded primitives
    std::vector<Primitive> getPrimitives();
    ///Get expanded nodes
    vec_Vec3f getPs();
    ///Get time allocation
    std::vector<decimal_t> getDts();
 
    ///Set max vel in each axis
    void setVmax(decimal_t v);
    ///Set max acc in each axis
    void setAmax(decimal_t a);
    ///Set max jerk in each axis
    void setJmax(decimal_t j);
    ///Set max control in each axis
    void setUmax(decimal_t u);
    ///Set max time step to explore
    void setTmax(decimal_t t);
    ///Set prior trajectory
    void setPriorTrajectory(const Trajectory& traj);
    ///Set dt for each primitive
    void setDt(decimal_t dt);
    ///Set weight for cost in time
    void setW(decimal_t w);
    ///Set alpha in time offset
    void setAlpha(int alpha);
    ///Set greedy searching param 
    void setEpsilon(decimal_t eps);
    ///Set max number of expansion
    void setMaxNum(int num);
    ///Enable U through discretization 
    void setU(int n, bool use_3d);
    ///Set U 
    void setU(const vec_Vec3f& U);
    ///Set effort degree
    void setMode(const Waypoint& p);

    ///Set tolerance in geometric and dynamic spaces
    void setTol(decimal_t tol_dis, decimal_t tol_vel, decimal_t tol_acc = 0.0);
    ///Planning thread
    virtual bool plan(const Waypoint &start, const Waypoint &goal);

  protected:
    ///Env class
    std::unique_ptr<MPL::env_base> ENV_;
    ///Intermediate nodes in optimal trajectory
    std::vector<Waypoint> ws_;
    ///Optimal trajectory
    Trajectory traj_;
    ///Time allocation for each segment
    std::vector<decimal_t> dts_;
    ///Greedy searching parameter
    decimal_t epsilon_ = 1.0;
    ///Maxmum number of expansion allowd, -1 means no limitation
    int max_num_ = -1;

    ///Enabled to display debug message
    bool planner_verbose_;
};


#endif
