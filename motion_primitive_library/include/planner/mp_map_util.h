/**
 * @file mp_map_util.h
 * @brief motion primitive map util
 */
#include <planner/env_map.h>
#include <planner/mp_base_util.h>

/**
 * @brief Motion primitive planner in voxel map
 */
class MPMapUtil : public MPBaseUtil
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPMapUtil(bool verbose);
    ///Set map util
    void setMapUtil(std::shared_ptr<MPL::VoxelMapUtil> map_util);
    ///Set sub map util
    void setMapUtil(std::shared_ptr<MPL::SubVoxelMapUtil> map_util);
    ///Get linked voxels
    vec_Vec3f getLinkedNodes();
    ///Remove affected nodes
    void removeAffectedNodes(const vec_Vec3i& pns);

  protected:
    std::shared_ptr<MPL::VoxelMapUtil> map_util_;
};


