/**
 * @file voxel_map_util.h
 * @brief VoxelMapUtil classes
 */

#ifndef VOXEL_MAP_UTIL_H
#define VOXEL_MAP_UTIL_H

#include <collision_checking/map_util_base.h>

class VoxelMapUtil
    : public MapUtilBase<Vec3i, Vec3f, std::vector<signed char>> {
public:
  VoxelMapUtil() : MapUtilBase() {}

  void info() {
    printf("Map Info ============\n");

    printf("   res: [%f]\n", res_);
    printf("   origin: [%f, %f, %f]\n", origin_d_(0), origin_d_(1), origin_d_(2));
    Vec3f dim_d = dim_.cast<decimal_t>() * res_;
    printf("   dim: [%f, %f, %f]\n", dim_d(0), dim_d(1), dim_d(2));
    printf("   size: [%d]\n", dim_(0) * dim_(1) *dim_(2));
  }

  Vec3f intToFloat(const Vec3i &pp) {
    return (pp.cast<decimal_t>() + Vec3f::Constant(0.5)) * res_ + origin_d_;
  }

  Vec3i floatToInt(const Vec3f &pt) {
    return ((pt - origin_d_) / res_).cast<int>();
  }

  bool isOutSide(const Vec3i &pn) {
    return pn(0) < 0 || pn(0) >= dim_(0) ||
      pn(1) < 0 || pn(1) >= dim_(1) ||
      pn(2) < 0 || pn(2) >= dim_(2);
  }

  int getIndex(const Vec3i &pp) {
    return pp(0) + dim_(0) * pp(1) + dim_(0) * dim_(1) * pp(2);
  }

  vec_Vec3i rayTrace(const Vec3f &pt1, const Vec3f &pt2) {
    Vec3f diff = pt2 - pt1;
    decimal_t k = 0.8;
    int max_diff = (diff / res_).lpNorm<Eigen::Infinity>() / k;
    decimal_t s = 1.0 / max_diff;
    Vec3f step = diff * s;

    vec_Vec3i pns;
    Vec3i prev_pn(-1, -1, -1);
    for (int n = 1; n < max_diff; n++) {
      Vec3f pt = pt1 + step * n;
      Vec3i new_pn = floatToInt(pt);
      if (isOutSide(new_pn))
        break;
      if (new_pn != prev_pn)
        pns.push_back(new_pn);
      prev_pn = new_pn;
    }

    return pns;
  }

  void dilate(decimal_t r, decimal_t h) {
    int dilate_xy = std::ceil(r / res_);
    int dilate_z = std::ceil(h / res_); // assum robot is 0.2 m high
    dilate_neighbor_.clear();
    Vec3i n;
    for (n(0) = -dilate_xy; n(0) <= dilate_xy; n(0)++) {
      for (n(1) = -dilate_xy; n(1) <= dilate_xy; n(1)++) {
        for (n(2) = -dilate_z; n(2) <= dilate_z; n(2)++) {
          decimal_t d = res_ * n.topRows(2).norm() - r;
          // if (d >= -0.1 && d <= res_ * 0.71)
          if (d <= res_ * 0.71)
            dilate_neighbor_.push_back(n);
        }
      }
    }

    /*
    Vec3i pn = floatToInt(Vec3f::Zero());
    for (const auto &it : dilate_neighbor_) {
      if (!isOutSide(pn + it))
        map_[getIndex(pn + it)] = val_occ;
    }
    */
  }

  vec_Vec3f getFreeCloud() {
    vec_Vec3f cloud;
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isFree(n))
            cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  vec_Vec3f getUnknownCloud() {
    vec_Vec3f cloud;
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isUnKnown(n))
            cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  vec_Vec3f getCloud() {
    vec_Vec3f cloud;
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isOccupied(n))
            cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  void dilating() {
    std::vector<signed char> map = map_;
    Vec3i n = Vec3i::Zero();
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isOccupied(n)) {
            for (const auto &it : dilate_neighbor_) {
              if (!isOutSide(n + it))
                map[getIndex(n + it)] = val_occ;
            }
          }
        }
      }
    }
    map_ = map;
  }

  void addPoints(const vec_Vec3f &pts) {
    for (const auto &pt : pts) {
      if(pt(2) < -1.0)
        continue;
      Vec3i pn_raw = floatToInt(pt);
      for(int z = 0; z < dim_(2); z++) {
        Vec3i pn = pn_raw;
        pn(2) = z;
        if (!isOutSide(pn)) {
          for (const auto &it : dilate_neighbor_) {
            if (!isOutSide(pn + it))
            map_[getIndex(pn + it)] = val_occ;
          }

        }
      }
    }
  }

  bool isEdge(const Vec3i &n) {
    return n(0) == 0 || n(0) == dim_(0) - 1 ||
      n(1) == 0 || n(1) == dim_(1) - 1;
    //n(2) == 0 || n(2) == dim_(2) - 1;
  }

  void freeUnKnown() {
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isUnKnown(n))
            map_[getIndex(n)] = val_free;
        }
      }
    }
  }


  void freeUnKnown(decimal_t theta) {
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isUnKnown(n)){
            Vec3f pt = intToFloat(n);
            decimal_t t = atan2(pt(2), sqrt(pt(0)*pt(0) + pt(1)*pt(1)));
            //if(fabs(t) <= theta && !isBlocked(Vec3f(0, 0, 0), pt))
            if(fabs(t) <= theta)
              map_[getIndex(n)] = val_free;
          }
        }
      }
    }
  }

  void clear()
  {
    std::fill(map_.begin(), map_.end(), val_unknown);
  }

  void clearAround(const Vec3i& n)
  {
    if(!isOutSide(n))
      map_[getIndex(n)] = val_free;
  }

  void clearArround(const Vec3i& pn)
  {
    if(!isOutSide(pn))
      map_[getIndex(pn)] = val_free;
    int dilate_xy = std::ceil(1.0 / res_);
    int dilate_z = 2; // assum robot is 0.2 m high

    Vec3i n;
    for (n(0) = -dilate_xy; n(0) <= dilate_xy; n(0)++) {
      for (n(1) = -dilate_xy; n(1) <= dilate_xy; n(1)++) {
        for (n(2) = -dilate_z; n(2) <= dilate_z; n(2)++) {
          if(!isOutSide(pn+n))
            map_[getIndex(pn+n)] = val_free;
        }
      }
    }
  }



};

#endif
