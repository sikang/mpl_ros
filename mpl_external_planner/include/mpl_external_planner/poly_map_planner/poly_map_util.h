/**
 * @file poly_map_util.h
 * @brief Poly map util class for collistion checking
 */
#ifndef MPL_POLY_MAP_UTIL_H
#define MPL_POLY_MAP_UTIL_H
#include <mpl_external_planner/poly_map_planner/simple_obstacle.h>
#include <mpl_basis/primitive.h>

/**
 * @brief Collision checking inside a polygonal map
 *
 */
template <int Dim>
class PolyMapUtil {
  public:
    /**
     * @brief Simple constructor
     */
    PolyMapUtil() {}

    ///Set polyhedron obstacles
    void addObstacle(const Polyhedron<Dim>& o) {
      poly_obs_.push_back(o);
    }

    ///Set ellipsoid obstacles
    void addObstacle(const Ellipsoid<Dim>& o) {
      elli_obs_.push_back(o);
    }

    ///Set bounding box for 2D
    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      setBoundingBox(const Vec2f& ori, const Vec2f& dim) {
        Polyhedron2D Vs;
        Vs.add(Hyperplane2D(ori + Vec2f(0, dim(1) / 2), -Vec2f::UnitX()));
        Vs.add(Hyperplane2D(ori + Vec2f(dim(0) / 2, 0), -Vec2f::UnitY()));
        Vs.add(Hyperplane2D(ori + dim - Vec2f(0, dim(1) / 2), Vec2f::UnitX()));
        Vs.add(Hyperplane2D(ori + dim - Vec2f(dim(0) / 2, 0), Vec2f::UnitY()));
        bbox_ = Vs;
      }

    ///Set bounding box for 3D
    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      setBoundingBox(const Vec3f& ori, const Vec3f& dim) {
        Polyhedron3D Vs;
        Vs.add(Hyperplane3D(ori + Vec3f(0, dim(1) / 2, dim(2) / 2), -Vec3f::UnitX()));
        Vs.add(Hyperplane3D(ori + Vec3f(dim(0) / 2, 0, dim(2) / 2), -Vec3f::UnitY()));
        Vs.add(Hyperplane3D(ori + Vec3f(dim(0) / 2, dim(2) / 2, 0), -Vec3f::UnitZ()));
        Vs.add(Hyperplane3D(ori + dim - Vec3f(0, dim(1) / 2, dim(2) / 2), Vec3f::UnitX()));
        Vs.add(Hyperplane3D(ori + dim - Vec3f(dim(0) / 2, 0, dim(2) / 2), Vec3f::UnitY()));
        Vs.add(Hyperplane3D(ori + dim - Vec3f(dim(0) / 2, dim(1) / 2, 0), Vec3f::UnitZ()));
        bbox_ = Vs;
      }

    bool isFree(const Vecf<Dim>& pt) const {
      if (!bbox_.inside(pt))
        return false;
      for(const auto& poly: poly_obs_) {
        if(poly.inside(pt))
          return false;
      }
      for(const auto& elli: elli_obs_) {
        if(elli.inside(pt))
          return false;
      }
      return true;
    }

    ///Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
    bool isFree(const Primitive<Dim> &pr) const {
      vec_E<Waypoint<Dim>> ps = pr.sample(5);
      for (const auto &it : ps) {
        if(!isFree(it.pos))
          return false;
      }

      return true;
    }

    ///Get bounding polyhedron
    Polyhedron3D getBoundingBox() const {
      return bbox_;
    }


    vec_E<Ellipsoid<Dim>> getEllipsoidObs() const {
      return elli_obs_;
    }


    vec_E<Polyhedron<Dim>> getPolyhedronObs() const {
      return poly_obs_;
    }

  private:
    ///Static polyhedron obstacle
    vec_E<Polyhedron<Dim>> poly_obs_;
    ///Static ellipsoid obstacle
    vec_E<Ellipsoid<Dim>> elli_obs_;
    ///Bounding box
    Polyhedron<Dim> bbox_;
};
#endif


