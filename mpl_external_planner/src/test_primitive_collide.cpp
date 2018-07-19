#include <mpl_external_planner/poly_map_planner/primitive_geometry_utils.h>

int main() {

  Waypoint2D s;
  s.pos = Vec2f::Zero();
  s.vel = Vec2f(0, 0);
  s.acc = Vec2f::Zero();
  s.jrk = Vec2f::Zero();
  s.control = Control::ACC;

  Vec2f u = Vec2f::Zero();

  decimal_t t = 2;
  Primitive2D pr(s, u, t);

  print(pr);


  PolyhedronObstacle2D obs;
  obs.add(Hyperplane2D(Vec2f(1, 0), Vec2f(-1, 0)));
  obs.v_ = Vec2f(-1, 0);

  collide(pr, obs, 0);

  return 0;
}
