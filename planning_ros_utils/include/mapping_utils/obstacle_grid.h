#ifndef OBSTACLE_GRID_H
#define OBSTACLE_GRID_H

#include <motion_primitive_library/data_utils.h>
#include <ros_utils/data_ros_utils.h>
#include <boost/multi_array.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <planning_ros_msgs/VoxelMap.h>

class ObstacleGrid
{
 public:
  ObstacleGrid(Vec3f origin, Vec3f dim, float res);

  void clear();
  sensor_msgs::PointCloud getCloud();
  planning_ros_msgs::VoxelMap getMap();
  nav_msgs::OccupancyGrid getOccMap();

  bool allocate(const Vec3f &new_dim_d, const Vec3f &new_ori_d);
  void addCloud(const vec_Vec3f &pts);

 private:
  bool getVoxel(Vec3i pose, char *&voxel);
  void inc(char &val);
  void dec(char &val);

  Vec3f intToFloat(const Vec3i &pp);
  Vec3i floatToInt(const Vec3f &pt);

  Vec3i dim_;
  Vec3i origin_;
  Vec3f origin_d_;
  float res_;
  boost::multi_array<char, 3> map_;
  boost::multi_array<char, 2> map_2d_;

  char val_free;
  char val_occ;
  char val_unknown;
  char val_even;

};

#endif
