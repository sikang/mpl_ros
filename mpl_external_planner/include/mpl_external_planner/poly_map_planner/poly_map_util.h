/**
 * @file poly_map_util.h
 * @brief Poly map util class for collistion checking
 */
#ifndef MPL_POLY_MAP_UTIL_H
#define MPL_POLY_MAP_UTIL_H
#include <mpl_external_planner/poly_map_planner/primitive_geometry_utils.h>

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

    ///Set static polyhedron obstacles
    void addStaticObstacle(const PolyhedronObstacle<Dim>& o) {
      static_obs_.push_back(o);
    }

    ///Set linear polyhedron obstacles
    void addLinearObstacle(const PolyhedronLinearObstacle<Dim>& o) {
      linear_obs_.push_back(o);
    }

    ///Set non-linear polyhedron obstacles
    void addNonlinearObstacle(const PolyhedronNonlinearObstacle<Dim>& o) {
      nonlinear_obs_.push_back(o);
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

		bool isFree(const Vecf<Dim>& pt, decimal_t t) const {
			if (!bbox_.inside(pt))
				return false;
			for(const auto& poly: static_obs_) {
				if(poly.inside(pt))
					return false;
			}
			for(const auto& poly: linear_obs_) {
				if(poly.inside(pt, t))
					return false;
			}
			for(const auto& poly: nonlinear_obs_) {
				if(poly.inside(pt, t))
					return false;
			}
			return true;
		}

		///Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
    bool isFree(const Primitive<Dim> &pr, decimal_t t) const {
      for(const auto& poly: static_obs_) {
        if(collide(pr, poly))
          return false;
      }

      for(const auto& poly: linear_obs_) {
        if(collide(pr, poly, t))
          return false;
      }

      for(const auto& poly: nonlinear_obs_) {
        if(collide(pr, poly, t))
          return false;
      }
      return true;
    }

    ///Get bounding polyhedron
    Polyhedron<Dim> getBoundingBox() const {
      return bbox_;
    }

    vec_E<Polyhedron<Dim>> getPolyhedrons(decimal_t time) const {
      vec_E<Polyhedron<Dim>> poly_obs;
      for(const auto& it: static_obs_)
        poly_obs.push_back(it.poly(time));
      for(const auto& it: linear_obs_)
        poly_obs.push_back(it.poly(time));
      for(const auto& it: nonlinear_obs_)
        poly_obs.push_back(it.poly(time));

      return poly_obs;
    }

  private:
    ///Static polyhedron obstacle
    vec_E<PolyhedronObstacle<Dim>> static_obs_;
    ///Static polyhedron obstacle
    vec_E<PolyhedronLinearObstacle<Dim>> linear_obs_;
    ///Static polyhedron obstacle
    vec_E<PolyhedronNonlinearObstacle<Dim>> nonlinear_obs_;
    ///Bounding box
    Polyhedron<Dim> bbox_;
};
#endif


