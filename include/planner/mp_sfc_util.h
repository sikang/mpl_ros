/**
 * @file mp_sfc_util.h
 * @brief Motion primitive util class in Safe Flight Corridor (SFC)
 */
#include <planner/env_sfc.h>
#include <planner/mp_base_util.h>

/**
 * @brief Motion primitive planner in SFC
 */
class MPSFCUtil: public MPBaseUtil
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPSFCUtil(bool verbose);
    ///Planning from start to goal
    bool plan(const Waypoint &start, const Waypoint &goal);
    ///Set SFC
    void setMap(const Polyhedra& poly);
};


