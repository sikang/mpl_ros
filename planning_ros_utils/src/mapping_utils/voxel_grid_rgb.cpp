#include <mapping_utils/voxel_grid_rgb.h>

VoxelGridRGB::VoxelGridRGB(float res) {
  res_ = res;
  expand_dim_ = 1.0;

  Vec3f origin(0, 0, 0);
  Vec3f dim(0, 0, 3);

  origin_ = Vec3i::Zero();
  origin_d_ = Vec3f::Zero();
  dim_ = Vec3i::Zero();

  allocate(dim, origin);
}

VoxelGridRGB::VoxelGridRGB(Vec3f origin, Vec3f dim, float res) {
  res_ = res;
  expand_dim_ = 0.0;

  origin_ = Vec3i::Zero();
  origin_d_ = Vec3f::Zero();
  dim_ = Vec3i::Zero();

  allocate(dim, origin);
}

void VoxelGridRGB::clear() {
  std::fill(map_rgb_.data(), map_rgb_.data() + map_rgb_.num_elements(), rgb3(0, 0, 0, val_free));
}

// =========== get rgb value ========================== //
bool VoxelGridRGB::getRGB(Vec3i pose, rgb3 *&rgb) {
  if (pose(0) < 0 || pose(1) < 0 || pose(2) < 0 || pose(0) >= dim_(0) ||
      pose(1) >= dim_(1) || pose(2) >= dim_(2))
    return false;
  rgb = &(map_rgb_[pose(0)][pose(1)][pose(2)]);
  return true;
}

// =========== Allocate volume ============== //
bool VoxelGridRGB::allocate(const Vec3f &new_dim_d, const Vec3f &new_ori_d) {
  Vec3i new_dim(new_dim_d(0) / res_, new_dim_d(1) / res_, new_dim_d(2) / res_);
  Vec3i new_ori(new_ori_d(0) / res_, new_ori_d(1) / res_, new_ori_d(2) / res_);

  if (new_dim(0) == dim_(0) && new_dim(1) == dim_(1) && new_dim(2) == dim_(2) &&
      new_ori(0) == origin_(0) && new_ori(1) == origin_(1) &&
      new_ori(2) == origin_(2))
    return false;
  else {
    array_type_rgb new_map_rgb(boost::extents[new_dim(0)][new_dim(1)][new_dim(2)]);
    std::fill(new_map_rgb.data(), new_map_rgb.data() + new_map_rgb.num_elements(), rgb3(0, 0, 0, val_free));

    for (int l = 0; l < new_dim(0); l++) {
      for (int w = 0; w < new_dim(1); w++) {
        for (int h = 0; h < new_dim(2); h++) {
          if(l + new_ori(0) >= origin_(0) && 
              w + new_ori(1) >= origin_(1) &&
              h + new_ori(2) >= origin_(2) &&
              l + new_ori(0) < origin_(0) + dim_(0) &&
              w + new_ori(1) < origin_(1) + dim_(1) &&
              h + new_ori(2) < origin_(2) + dim_(2))
          {

            int new_l = l + new_ori(0) - origin_(0);
            int new_w = w + new_ori(1) - origin_(1);
            int new_h = h + new_ori(2) - origin_(2);

            new_map_rgb[l][w][h] = map_rgb_[new_l][new_w][new_h];
          }
        }
      }
    }

    map_rgb_.resize(boost::extents[new_dim(0)][new_dim(1)][new_dim(2)]);
    map_rgb_ = new_map_rgb;


    dim_ = new_dim;
    origin_ = new_ori;
    origin_d_ = new_ori_d;

    return true;
  }
}

Vec3f VoxelGridRGB::intToFloat(const Vec3i &pp) {
  return (pp.cast<decimal_t>() + Vec3f::Constant(0.5)) * res_ + origin_d_;
}

Vec3i VoxelGridRGB::floatToInt(const Vec3f &pt) {
  return ((pt - origin_d_) / res_).cast<int>();
}

void VoxelGridRGB::addColor(const Vec3f &end, const rgb3 &color) {
  rgb3 *rgb;
  Vec3i end_i = floatToInt(end);
  if (getRGB(end_i, rgb))
    *rgb += color;
}

void VoxelGridRGB::addCloud(const Aff3f &TF, const sensor_msgs::PointCloud& cloud) {
  vec_Vec3f pts = cloud_to_vec(cloud);
  pts = transform_vec3(pts, TF);

  if (expand_dim_ > 0) {
    Vec3f max = origin_d_ + dim_.cast<decimal_t>() * res_;
    Vec3f min = origin_d_;
    unsigned int dim = 2;
    for (auto it : pts) {
      for (unsigned int i = 0; i < dim; i++) {
        if (max(i) <= it(i))
          max(i) = it(i);
        else if (it(i) < min(i))
          min(i) = it(i);
      }
    }

    Vec3f new_origin_d = origin_d_;
    Vec3f new_dim_d(dim_(0) * res_, dim_(1) * res_, dim_(2) * res_);

    for (unsigned int i = 0; i < dim; i++) {
      if (min(i) < origin_d_(i)) {
        new_origin_d(i) = min(i) - expand_dim_;
        new_dim_d(i) += origin_d_(i) - new_origin_d(i);
      }
      if (max(i) > origin_d_(i) + dim_(i) * res_)
        new_dim_d(i) += expand_dim_ + max(i) - (origin_d_(i) + dim_(i) * res_);
    }
    allocate(new_dim_d, new_origin_d);
  }

  Vec3f robot(TF.translation().x(), TF.translation().y(), TF.translation().z());

  for (unsigned int i = 0; i < pts.size(); i++) {
    rgb3 color(cloud.channels[0].values[i]);
    color.a = 127;
    addColor(pts[i], color);
  }
}

sensor_msgs::PointCloud VoxelGridRGB::getCloud() {
  sensor_msgs::PointCloud cloud;
  cloud.channels.resize(3);
  cloud.channels[0].name = std::string("r");
  cloud.channels[1].name = std::string("g");
  cloud.channels[2].name = std::string("b");
  for (int l = 0; l < dim_(0); l++) {
    for (int w = 0; w < dim_(1); w++) {
      for (int h = 0; h < dim_(2); h++) {
        if (map_rgb_[l][w][h].a > val_even) {
          cloud.channels[0].values.push_back(map_rgb_[l][w][h].r / 255.0);
          cloud.channels[1].values.push_back(map_rgb_[l][w][h].g / 255.0);
          cloud.channels[2].values.push_back(map_rgb_[l][w][h].b / 255.0);

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


planning_ros_msgs::VoxelMap VoxelGridRGB::getMap() {
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

  voxel_map.data.resize(dim_(0) * dim_(1) * dim_(2), val_free);
  Vec3i n;
  for(n(0) = 0; n(0) < dim_(0); n(0)++) {
    for(n(1) = 0; n(1) < dim_(1); n(1)++) {
      for(n(2) = 0; n(2) < dim_(2); n(2)++) {
        if(map_rgb_[n(0)][n(1)][n(2)].a > val_free) {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_occ;
        }
        else if(map_rgb_[n(0)][n(1)][n(2)].a != val_free) {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_unknown;
        }
      }
    }
  }
  return voxel_map;
}

