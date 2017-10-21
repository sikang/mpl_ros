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
    ///Check if it has been planned
    bool initialized();
    ///Get nodes on the optimal trajectory
    std::vector<Waypoint> getWs() const;
    ///Get optimal trajectory
    Trajectory getTraj() const;
    ///Get expanded primitives
    std::vector<Primitive> getPrimitives() const;
    ///Get ps in open set
    vec_Vec3f getOpenSet() const;
    ///Get ps in close set
    vec_Vec3f getCloseSet() const;
    ///Get expanded node
    vec_Vec3f getExpandedNodes() const;
    /**
     * @brief Prune state space
     * @param id the id of start state as the branch of corresponding tree
     */
    void getSubStateSpace(int id);

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
   /**
     * @brief Planning thread
     * @param replan default as false, such that plan from scratch; set to be true, the planner reuses the state space for planning
     */
    virtual bool plan(const Waypoint &start, const Waypoint &goal, bool replan = false);

  protected:
    ///Env class
    std::unique_ptr<MPL::env_base> ENV_;
    ///Planner workspace
    std::shared_ptr<MPL::ARAStateSpace<Waypoint>> sss_ptr_;
    ///Intermediate nodes in optimal trajectory
    std::vector<Waypoint> ws_;
    ///Optimal trajectory
    Trajectory traj_;
    ///Greedy searching parameter
    decimal_t epsilon_ = 1.0;
    ///Maxmum number of expansion allowd, -1 means no limitation
    int max_num_ = -1;


    ///Enabled to display debug message
    bool planner_verbose_;
};


#endif
