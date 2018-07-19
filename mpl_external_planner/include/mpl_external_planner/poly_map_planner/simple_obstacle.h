#include <decomp_geometry/polyhedron.h>

///Based obstacle class
template <int Dim>
struct PolyhedronObstacle : Polyhedron<Dim> {
///Obstacle geometry
	Vecf<Dim> v_{Vecf<Dim>::Zero()};
};

typedef PolyhedronObstacle<2> PolyhedronObstacle2D;

typedef PolyhedronObstacle<3> PolyhedronObstacle3D;

