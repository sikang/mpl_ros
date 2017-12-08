/**
 * @file mapping_ros_util.h
 * @brief Simple API for VoxelMapUtil class
 */
#pragma once
#include <collision_checking/voxel_map_util.h>
#include <collision_checking/sub_voxel_map_util.h>
//#include <collision_checking/jps_voxel_map_util.h>
//#include <collision_checking/jps_sub_voxel_map_util.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <sensor_msgs/PointCloud.h>

void setMap(std::shared_ptr<MPL::VoxelMapUtil> map_util, const planning_ros_msgs::VoxelMap& msg);
void getMap(std::shared_ptr<MPL::VoxelMapUtil> map_util, planning_ros_msgs::VoxelMap& map);

//void setMap(MPL::SubVoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg);
//void getMap(MPL::SubVoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map);

//void setMap(JPS::VoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg);
//void getMap(JPS::VoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map);

//void setMap(JPS::SubVoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg);
//void getMap(JPS::SubVoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map);


//void toMPLMapUtil(JPS::VoxelMapUtil* jps_map_util, MPL::VoxelMapUtil* mpl_map_util);
//void toJPSMapUtil(MPL::VoxelMapUtil* mpl_map_util, JPS::VoxelMapUtil* jps_map_util);
