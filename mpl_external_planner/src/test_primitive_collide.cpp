#include <mpl_external_planner/poly_map_planner/primitive_geometry_utils.h>

int main() {
  // construct primitive
  Waypoint2D s;
  s.pos = Vec2f(0, 0);
  s.vel = Vec2f(0, 0);
  s.acc = Vec2f::Zero();
  s.jrk = Vec2f::Zero();
  s.control = Control::VEL;

  Vec2f u = Vec2f(1.0, 0);

  decimal_t t = 2;
  Primitive2D pr(s, u, t);

  print(pr);

  // construct nonlinear obstacle
  Waypoint2D s1;
  s1.pos = Vec2f(1.0, 0);
  s1.vel = Vec2f(0, 0);
  s1.acc = Vec2f::Zero();
  s1.jrk = Vec2f::Zero();
  s1.control = Control::VEL;

  Vec2f u1 = Vec2f(0, 0);

  decimal_t t1 = 1;
  Primitive2D seg1(s1, u1, t1);

  /*
  Waypoint2D s2 = seg1.evaluate(seg1.t());
  Vec2f u2 = Vec2f(-1, 0);
  decimal_t t2 = 1;

  Primitive2D seg2(s2, u2, t2);
  */

  printf("seg1: \n");
  print(seg1);
  // printf("seg2: \n");
  // print(seg2);

  vec_E<Primitive2D> segs;
  segs.push_back(seg1);
  // segs.push_back(seg2);

  Trajectory2D traj(segs);

  Polyhedron2D geometry;
  geometry.add(Hyperplane2D(Vec2f(0, 0), Vec2f(-1, 0)));
  PolyhedronNonlinearObstacle2D obs(geometry, traj, 0);

  collide(pr, obs, 0.0);

  return 0;
}
