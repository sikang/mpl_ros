#include <mapping_utils/obstacle_grid.h>

ObstacleGrid::ObstacleGrid(Vec3f origin, Vec3f dim, float res)
{
  val_unknown = -1;
  val_occ = 100;
  val_free = 0;
  val_even = 50;

  origin_ = Vec3i::Zero();
  origin_d_ = Vec3f::Zero();
  dim_ = Vec3i::Zero();

  res_ = res;
  allocate(dim, origin);
}

void ObstacleGrid::clear()
{
  std::fill(map_.data(), map_.data() + map_.num_elements(), val_free);
  std::fill(map_2d_.data(), map_2d_.data() + map_2d_.num_elements(),
            val_free);
}

sensor_msgs::PointCloud ObstacleGrid::getCloud()
{
  sensor_msgs::PointCloud cloud;
  for(int l = 0; l < dim_(0); l++)
  {
    for(int w = 0; w < dim_(1); w++)
    {
      for(int h = 0; h < dim_(2); h++)
      {
        if(map_[l][w][h] >= val_even)
        {
          geometry_msgs::Point32 p;
          Vec3f pt = intToFloat(Vec3i(l, w, h));
          p.x = pt(0);
          p.y = pt(1);
          p.z = pt(2);
          cloud.points.push_back(p);
        }
      }
    }
  }
  return cloud;
}

nav_msgs::OccupancyGrid ObstacleGrid::getOccMap()
{
  nav_msgs::OccupancyGrid occ_map;

  occ_map.info.resolution = res_;
  occ_map.info.origin.position.x = origin_d_(0);
  occ_map.info.origin.position.y = origin_d_(1);
  occ_map.info.origin.position.z = 0.0;
  occ_map.info.origin.orientation.x = 0.0;
  occ_map.info.origin.orientation.y = 0.0;
  occ_map.info.origin.orientation.z = 0.0;
  occ_map.info.origin.orientation.w = 1.0;

  occ_map.info.width = dim_(0);
  occ_map.info.height = dim_(1);

  occ_map.data.resize(occ_map.info.width * occ_map.info.height, val_unknown);
  for(int x = 0; x < dim_(0); x++)
  {
    for(int y = 0; y < dim_(1); y++)
    {
      if(map_2d_[x][y] != val_unknown)
        occ_map.data[occ_map.info.width * y + x] = map_2d_[x][y];
    }
  }
  return occ_map;
}

planning_ros_msgs::VoxelMap ObstacleGrid::getMap()
{
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_(0);
  voxel_map.origin.y = origin_d_(1);
  voxel_map.origin.z = origin_d_(2);
  voxel_map.dim.x = dim_(0);
  voxel_map.dim.y = dim_(1);
  voxel_map.dim.z = dim_(2);

  voxel_map.info.resolution = res_;
  voxel_map.info.val_unknown = val_unknown;
  voxel_map.info.val_free = val_free;
  voxel_map.info.val_occupied = val_occ;

  voxel_map.data.resize(dim_(0) * dim_(1) * dim_(2), val_unknown);
  Vec3i n;
  for(n(0) = 0; n(0) < dim_(0); n(0)++)
  {
    for(n(1) = 0; n(1) < dim_(1); n(1)++)
    {
      for(n(2) = 0; n(2) < dim_(2); n(2)++)
      {
        if(map_[n(0)][n(1)][n(2)] >= val_even)
        {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_occ;
        }
        else if(map_[n(0)][n(1)][n(2)] != val_unknown)
        {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_free;
        }
      }
    }
  }
  return voxel_map;
}

bool ObstacleGrid::allocate(const Vec3f &new_dim_d, const Vec3f &new_ori_d)
{
  Vec3i new_dim(new_dim_d(0) / res_, new_dim_d(1) / res_, new_dim_d(2) / res_);
  Vec3i new_ori(new_ori_d(0) / res_, new_ori_d(1) / res_, new_ori_d(2) / res_);
  if(new_dim(2) == 0 && new_ori(2) == 0)
    new_dim(2) = 1;

  if(new_dim(0) == dim_(0) && new_dim(1) == dim_(1) && new_dim(2) == dim_(2) &&
     new_ori(0) == origin_(0) && new_ori(1) == origin_(1) &&
     new_ori(2) == origin_(2))
    return false;
  else
  {
    boost::multi_array<char, 3> new_map(
        boost::extents[new_dim(0)][new_dim(1)][new_dim(2)]);
    std::fill(new_map.data(), new_map.data() + new_map.num_elements(),
              val_unknown);
    boost::multi_array<char, 2> new_map_2d(
        boost::extents[new_dim(0)][new_dim(1)]);
    std::fill(new_map_2d.data(), new_map_2d.data() + new_map_2d.num_elements(),
              val_unknown);
    for(int l = 0; l < new_dim(0); l++)
    {
      for(int w = 0; w < new_dim(1); w++)
      {
        for(int h = 0; h < new_dim(2); h++)
        {
          if(l + new_ori(0) >= origin_(0) && w + new_ori(1) >= origin_(1) &&
             h + new_ori(2) >= origin_(2) &&
             l + new_ori(0) < origin_(0) + dim_(0) &&
             w + new_ori(1) < origin_(1) + dim_(1) &&
             h + new_ori(2) < origin_(2) + dim_(2))
          {
            int new_l = l + new_ori(0) - origin_(0);
            int new_w = w + new_ori(1) - origin_(1);
            int new_h = h + new_ori(2) - origin_(2);

            new_map[l][w][h] = map_[new_l][new_w][new_h];
          }
        }
        if(l + new_ori(0) >= origin_(0) && w + new_ori(1) >= origin_(1) &&
           l + new_ori(0) < origin_(0) + dim_(0) &&
           w + new_ori(1) < origin_(1) + dim_(1))
        {
          int new_l = l + new_ori(0) - origin_(0);
          int new_w = w + new_ori(1) - origin_(1);

          new_map_2d[l][w] = map_2d_[new_l][new_w];
        }
      }
    }

    map_.resize(boost::extents[new_dim(0)][new_dim(1)][new_dim(2)]);
    map_ = new_map;
    map_2d_.resize(boost::extents[new_dim(0)][new_dim(1)]);
    map_2d_ = new_map_2d;

    dim_ = new_dim;
    origin_ = new_ori;
    origin_d_ = new_ori_d;

    return true;
  }
}

bool ObstacleGrid::getVoxel(Vec3i pose, char *&voxel)
{
  if(pose(0) < 0 || pose(1) < 0 || pose(2) < 0 || pose(0) >= dim_(0) ||
     pose(1) >= dim_(1) || pose(2) >= dim_(2))
    return false;
  voxel = &(map_[pose(0)][pose(1)][pose(2)]);
  return true;
}

void ObstacleGrid::inc(char &val)
{
  val = val_occ;
}

void ObstacleGrid::addCloud(const vec_Vec3f &pts)
{
  for(auto it : pts)
  {
    char *voxel;
    Vec3i tn = floatToInt(it);
    if(getVoxel(tn, voxel))
      inc(*voxel);
  }
}

Vec3f ObstacleGrid::intToFloat(const Vec3i &pp) {
  return (pp.cast<decimal_t>() + Vec3f::Constant(0.5)) * res_ + origin_d_;
}

Vec3i ObstacleGrid::floatToInt(const Vec3f &pt) {
  return ((pt - origin_d_) / res_).cast<int>();
}

