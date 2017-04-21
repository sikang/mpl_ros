#include <planner/env_sfc.h>
#include <planner/mp_base_util.h>

class MPSFCUtil: public MPBaseUtil
{
  public:
    MPSFCUtil(bool verbose);
    bool plan(const Waypoint &start, const Waypoint &goal);
    void setMap(const Polyhedra& poly);
};


