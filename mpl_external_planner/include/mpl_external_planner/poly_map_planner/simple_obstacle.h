#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>

namespace MPL {
  ///Based obstacle class
  template <class Geometry>
  class BaseObstacle {
    public:
      ///Empty constructor
      BaseObstacle();
      ///Get current obstacle geometry
      Geometry getGeometry() { return geo_; }
      ///Update obstacle state according to dt
      virtual void update(double dt) {}
    protected:
      ///Obstacle geometry
      Geometry geo_;
  };


}
