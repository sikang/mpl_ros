#ifndef SFC_UTIL_H
#define SFC_UTIL_H
#include <primitive/primitive.h>

class SFCUtil {
  public:
    SFCUtil();
    SFCUtil(const Polyhedra& polys);

    Polyhedra sfc();
    bool isFree(const Primitive& pr, decimal_t dt);
    bool isFree(const Vec3f& pt);

  private:
    bool insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs);
    bool insideSFC(const Primitive& pr, const Polyhedra& polys, 
        decimal_t t1, decimal_t t2);
 
    Polyhedra polys_;
};
#endif


