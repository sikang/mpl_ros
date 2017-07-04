/**
 * @file mp_map_util.h
 * @brief motion primitive map util
 */
#include <planner/env_map.h>
#include <planner/mp_base_util.h>

/**
 * @brief Motion primitive map util class
 */
class MPMapUtil : public MPBaseUtil
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPMapUtil(bool verbose);
    ///Planning from start to goal
    bool plan(const Waypoint &start, const Waypoint &goal);
    ///Set map util
    void setMapUtil(std::shared_ptr<MPL::VoxelMapUtil> map_util);
    ///Set sub map util
    void setMapUtil(std::shared_ptr<MPL::SubVoxelMapUtil> map_util);
};


