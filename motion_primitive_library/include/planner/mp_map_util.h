/**
 * @file mp_map_util.h
 * @brief motion primitive map util
 */
#include <planner/env_map.h>
#include <planner/mp_base_util.h>

using linkedHashMap = std::unordered_map<int, std::vector<std::pair<MPL::Key, int>>>;
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
    /**
     * @brief Get linked voxels
     *
     * For each voxel in the map, we link it to all the primitives that passed through.
     */
    vec_Vec3f getLinkedNodes() const;
    /**
     * @brief Add the cost of affected nodes that have been blocked by the new obstacle
     *
     * @param pns the coordinate of new added obstacles
     */
    vec_Vec3f updateBlockedNodes(const vec_Vec3i& pns);
    /**
     * @brief Reduce the cost of affected nodes that have been presented by the cleared obstacle
     *
     * @param pns the coordinate of cleared obstacles
     */
    vec_Vec3f updateClearedNodes(const vec_Vec3i& pns);

  protected:
    std::shared_ptr<MPL::VoxelMapUtil> map_util_;
    mutable linkedHashMap lhm_;
};


