/**
 * @file ellipsoid_util.h
 * @brief ellipsoid util class for collistion checking
 */
#ifndef MPL_ELLIPSOID_UTIL_H
#define MPL_ELLIPSOID_UTIL_H
#include <mpl_external_planner/ellipsoid_planner/primitive_ellipsoid_utils.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/make_shared.hpp>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::KdTreeFLANN<PCLPoint> KDTree;

/**
 * @brief Collision checking inside a point cloud
 *
 */
class EllipsoidUtil {
 public:
  /**
   * @brief Simple constructor
   * @param r robot radius
   * @param h robot height, default as 0.1m
   */
  EllipsoidUtil(decimal_t r, decimal_t h = 0.1) { axe_ = Vec3f(r, r, h); }

  /// Set obstacles
  void setObstacles(const vec_Vec3f &obs) {
    obs_.clear();
    for (const auto &it : obs) {
      if (bbox_.inside(it)) obs_.push_back(it);
    }

    PCLPointCloud::Ptr cloud_ptr =
        boost::make_shared<PCLPointCloud>(toPCL(obs_));
    kdtree_.setInputCloud(cloud_ptr);
  }
  /// Set bounding box
  void setBoundingBox(const Vec3f &ori, const Vec3f &dim) {
    Polyhedron3D Vs;
    Vs.add(
        Hyperplane3D(ori + Vec3f(0, dim(1) / 2, dim(2) / 2), -Vec3f::UnitX()));
    Vs.add(
        Hyperplane3D(ori + Vec3f(dim(0) / 2, 0, dim(2) / 2), -Vec3f::UnitY()));
    Vs.add(
        Hyperplane3D(ori + Vec3f(dim(0) / 2, dim(2) / 2, 0), -Vec3f::UnitZ()));
    Vs.add(Hyperplane3D(ori + dim - Vec3f(0, dim(1) / 2, dim(2) / 2),
                        Vec3f::UnitX()));
    Vs.add(Hyperplane3D(ori + dim - Vec3f(dim(0) / 2, 0, dim(2) / 2),
                        Vec3f::UnitY()));
    Vs.add(Hyperplane3D(ori + dim - Vec3f(dim(0) / 2, dim(1) / 2, 0),
                        Vec3f::UnitZ()));
    bbox_ = Vs;
  }

  /// Get polyhedra
  Polyhedron3D getBoundingBox() const { return bbox_; }

  /// Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
  bool isFree(const Primitive3D &pr) {
    vec_E<Waypoint3D> ps = pr.sample(2);
    for (const auto &it : ps) {
      if (!bbox_.inside(it.pos)) return false;
    }
    decimal_t max_v =
        std::max(std::max(pr.max_vel(0), pr.max_vel(1)), pr.max_vel(2));
    int n = std::ceil(max_v * pr.t() / axe_(0));
    auto Es = sample_ellipsoids(pr, axe_, n);

    for (const auto &E : Es) {
      float radius = axe_(0);
      pcl::PointXYZ searchPoint;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      searchPoint.x = E.d()(0);
      searchPoint.y = E.d()(1);
      searchPoint.z = E.d()(2);

      if (kdtree_.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                               pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
          if (E.inside(Vec3f(obs_[pointIdxRadiusSearch[i]]))) return false;
      }
    }

    return true;
  }
  /// Convert obstacle points into pcl point cloud
  PCLPointCloud toPCL(const vec_Vec3f &obs) {
    PCLPointCloud cloud;
    cloud.width = obs.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    for (unsigned int i = 0; i < obs.size(); i++) {
      cloud.points[i].x = obs[i](0);
      cloud.points[i].y = obs[i](1);
      cloud.points[i].z = obs[i](2);
    }
    return cloud;
  }

 private:
  /// robot size: axe = (r, r, h)
  Vec3f axe_;
  /// obstacle points
  vec_Vec3f obs_;
  /// obstacles in kd tree form
  KDTree kdtree_;
  /// Bounding box
  Polyhedron3D bbox_;
};
#endif

