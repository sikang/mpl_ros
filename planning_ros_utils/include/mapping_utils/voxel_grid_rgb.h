#ifndef VOXEL_GRID_RGB_H
#define VOXEL_GRID_RGB_H

#include <mapping_utils/rgb_type.h>
#include <motion_primitive_library/data_utils.h>
#include <ros_utils/data_ros_utils.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <boost/multi_array.hpp>

class VoxelGridRGB {
  public:
    typedef boost::multi_array<rgb3, 3> array_type_rgb;
    VoxelGridRGB(float res);
    VoxelGridRGB(Vec3f origin, Vec3f dim, float res);

    void clear();
    sensor_msgs::PointCloud getCloud();
    planning_ros_msgs::VoxelMap getMap();

    bool allocate(const Vec3f &new_dim_d, const Vec3f &new_ori_d);
    void addCloud(const Aff3f &TF, const sensor_msgs::PointCloud &cloud);

  private:
    bool getRGB(Vec3i pose, rgb3 *&rgb);
    void addColor(const Vec3f &end, const rgb3 &color);
    Vec3f intToFloat(const Vec3i &pp);
    Vec3i floatToInt(const Vec3f &pt);

    array_type_rgb map_rgb_;

    Vec3i dim_;
    Vec3i origin_;
    Vec3f origin_d_;
    float res_;
    float expand_dim_;

    char val_free = 0;
    char val_occ = 100;
    char val_unknown = -1;
    char val_even = 63;
    char val_max = 100;
    char val_inc = 0;

};

#endif
