/**
 * @file sfc_util.h
 * @brief Safe Flight Corridor util class
 */
#ifndef SFC_UTIL_H
#define SFC_UTIL_H
#include <primitive/primitive.h>

/**
 * @brief Collision checking inside a Safe Flight Corridor (SFC)
 *
 * SFC is an ordered collection of convex polyhedra that models free space
 */
class SFCUtil {
  public:
    ///Simple constructor
    SFCUtil();
    ///Constructor with polyhedra as input
    SFCUtil(const Polyhedra& polys);

    ///Get polyhedra
    Polyhedra sfc();
    ///Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
    bool isFree(const Primitive& pr, decimal_t dt);
    ///Check if a point is inside SFC
    bool isFree(const Vec3f& pt);

  private:
    ///Check if a point is inside the given polyhedron
    bool insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs);
    ///Check if a primitive is inside the SFC from \f$t: t1\rightarrow t2\f$
    bool insideSFC(const Primitive& pr, const Polyhedra& polys, 
        decimal_t t1, decimal_t t2);
 
    ///The SFC object
    Polyhedra polys_;
};
#endif


