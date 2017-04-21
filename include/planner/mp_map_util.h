#include <planner/env_map.h>
#include <planner/mp_base_util.h>

class MPMapUtil : public MPBaseUtil
{
  public:
    MPMapUtil(bool verbose);
    bool plan(const Waypoint &start, const Waypoint &goal);
    void setMapUtil(std::shared_ptr<VoxelMapUtil> map_util);
};


