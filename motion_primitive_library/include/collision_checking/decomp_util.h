/**
 * @file decomp_util.h
 * @brief Safe Flight Corridor util class
 */
#ifndef SFC_UTIL_H
#define SFC_UTIL_H
#include <primitive/primitive.h>
#include <primitive/primitive_util.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/make_shared.hpp>
#include <boost/optional.hpp>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::KdTreeFLANN<PCLPoint> KDTree;

/**
 * @brief Collision checking inside a Safe Flight Corridor (SFC)
 *
 * SFC is an ordered collection of convex polyhedra that models free space
 */
class DecompUtil {
  public:
    ///Simple constructor
    DecompUtil(decimal_t r = 0);

    void setObstacles(const vec_Vec3f& obs);

    Polyhedron virtual_wall(const Vec3f& pos);

    ///Get polyhedra
    Polyhedra polyhedra();
    ///Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
    bool isFree(const Primitive& pr);
    ///Check if a point is inside SFC
    PCLPointCloud toPCL(const vec_Vec3f &obs);

    void set_region(const Vec3f& ori, const Vec3f& dim);
 private:
    ///Check if a point is inside the given polyhedron
    bool insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs);
    bool insideEllipsoid(const Ellipsoid& E, const vec_Vec3f& O);

    decimal_t r_ = 0;
    Vec3f axe_;

    vec_Vec3f obs_;
    KDTree kdtree_;
    boost::optional<Polyhedron> Vs_;
};
#endif


