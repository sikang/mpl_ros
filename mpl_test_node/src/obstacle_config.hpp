#include <mpl_external_planner/poly_map_planner/poly_map_planner.h>

template <int Dim>
struct ObstacleCourse {
  ObstacleCourse() {}
  vec_E<PolyhedronObstacle<Dim>> static_obs;
  vec_E<PolyhedronLinearObstacle<Dim>> linear_obs;
  vec_E<PolyhedronNonlinearObstacle<Dim>> nonlinear_obs;

  /// Generate trajectory: going back and forth once, `t` is the segment
  /// duration
  Trajectory<Dim> back_and_forth(const Vecf<Dim> &start, const Vecf<Dim> &v,
                                 decimal_t t, Control::Control control) {
    Waypoint<Dim> s1;
    s1.pos = start;
    s1.vel = Vecf<Dim>::Zero();
    s1.acc = Vecf<Dim>::Zero();
    s1.jrk = Vecf<Dim>::Zero();
    s1.control = control;

    Vecf<Dim> u1 = v;

    decimal_t t1 = t;
    Primitive<Dim> seg1(s1, u1, t1);

    Waypoint<Dim> s2 = seg1.evaluate(seg1.t());
    Vec2f u2 = -u1;
    decimal_t t2 = t;

    Primitive<Dim> seg2(s2, u2, t2);

    vec_E<Primitive<Dim>> segs;
    segs.push_back(seg1);
    segs.push_back(seg2);

    return Trajectory<Dim>(segs);
  }

  /// Generate linear square trajectory
  Trajectory<Dim> square(const Vecf<Dim> &start, const Vecf<Dim> &v,
                         decimal_t t, bool clockwise = true) {
    Waypoint<Dim> s1;
    s1.pos = start;
    s1.vel = Vecf<Dim>::Zero();
    s1.acc = Vecf<Dim>::Zero();
    s1.jrk = Vecf<Dim>::Zero();
    s1.control = Control::VEL;

    Vecf<Dim> u1 = v;

    Primitive<Dim> seg1(s1, u1, t);

    Waypoint<Dim> s2 = seg1.evaluate(seg1.t());
    Vec2f u2(v(1), -v(0));
    if (!clockwise) u2 = -u2;
    Primitive<Dim> seg2(s2, u2, t);

    Waypoint<Dim> s3 = seg2.evaluate(seg2.t());
    Vec2f u3 = -u1;
    Primitive<Dim> seg3(s3, u3, t);

    Waypoint<Dim> s4 = seg3.evaluate(seg3.t());
    Vec2f u4 = -u2;
    Primitive<Dim> seg4(s4, u4, t);

    vec_E<Primitive<Dim>> segs;
    segs.push_back(seg1);
    segs.push_back(seg2);
    segs.push_back(seg3);
    segs.push_back(seg4);

    return Trajectory<Dim>(segs);
  }
};

/// Include linear obstacles
struct ObstacleCourse2DConfig0 : ObstacleCourse<2> {
  ObstacleCourse2DConfig0() {
    Polyhedron2D rec1;
    rec1.add(Hyperplane2D(Vec2f(5.5, 0), -Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(6.5, 0), Vec2f::UnitX()));
    rec1.add(Hyperplane2D(Vec2f(6, -1.5), -Vec2f::UnitY()));
    rec1.add(Hyperplane2D(Vec2f(6, 1.5), Vec2f::UnitY()));
    linear_obs.push_back(
        PolyhedronLinearObstacle2D(rec1, Vec2f::Zero(), Vec2f(-0.5, 0)));

    Polyhedron2D rec2;
    rec2.add(Hyperplane2D(Vec2f(15.5, 2.0), -Vec2f::UnitX()));
    rec2.add(Hyperplane2D(Vec2f(16.5, 2.0), Vec2f::UnitX()));
    rec2.add(Hyperplane2D(Vec2f(16, 0.5), -Vec2f::UnitY()));
    rec2.add(Hyperplane2D(Vec2f(16, 3.5), Vec2f::UnitY()));
    static_obs.push_back(PolyhedronObstacle2D(rec2, Vec2f::Zero()));

    Polyhedron2D rec3;
    rec3.add(Hyperplane2D(Vec2f(9.5, -2.0), -Vec2f::UnitX()));
    rec3.add(Hyperplane2D(Vec2f(16.5, -2.0), Vec2f::UnitX()));
    rec3.add(Hyperplane2D(Vec2f(14, -2.5), -Vec2f::UnitY()));
    rec3.add(Hyperplane2D(Vec2f(14, -1.5), Vec2f::UnitY()));
    linear_obs.push_back(
        PolyhedronLinearObstacle2D(rec3, Vec2f::Zero(), Vec2f(0.0, 0.25)));

    Polyhedron2D rec4;
    rec4.add(Hyperplane2D(Vec2f(15.5, 0.5), -Vec2f::UnitX()));
    rec4.add(Hyperplane2D(Vec2f(16.5, 0.5), Vec2f::UnitX()));
    rec4.add(Hyperplane2D(Vec2f(16, -0.5), -Vec2f::UnitY()));
    rec4.add(Hyperplane2D(Vec2f(16, 0.5), Vec2f::UnitY()));
    linear_obs.push_back(
        PolyhedronLinearObstacle2D(rec4, Vec2f::Zero(), Vec2f(0.0, -0.1)));

    Polyhedron2D rec5;
    rec5.add(Hyperplane2D(Vec2f(6.5, -2.0), -Vec2f::UnitX()));
    rec5.add(Hyperplane2D(Vec2f(15.5, -2.0), Vec2f::UnitX()));
    rec5.add(Hyperplane2D(Vec2f(7, -4.5), -Vec2f::UnitY()));
    rec5.add(Hyperplane2D(Vec2f(7, -3.5), Vec2f::UnitY()));
    linear_obs.push_back(
        PolyhedronLinearObstacle2D(rec5, Vec2f::Zero(), Vec2f(-0.1, 0.3)));

    printf("Initialized obstacle config 0!\n\n");
  }
};

/// Include static and nonlinear obstacles
struct ObstacleCourse2DConfig1 : ObstacleCourse<2> {
  ObstacleCourse2DConfig1() {
    Polyhedron2D rec;
    rec.add(Hyperplane2D(Vec2f(-0.5, 0), -Vec2f::UnitX()));
    rec.add(Hyperplane2D(Vec2f(0.5, 0), Vec2f::UnitX()));
    rec.add(Hyperplane2D(Vec2f(0, -1.25), -Vec2f::UnitY()));
    rec.add(Hyperplane2D(Vec2f(0, 1.25), Vec2f::UnitY()));

    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        rec, back_and_forth(Vec2f(4, 1.25), Vec2f(0, 1), 1.5, Control::VEL),
        -3));
    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        rec, back_and_forth(Vec2f(4, -1.25), Vec2f(0, -1), 1.5, Control::VEL),
        -3));

    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        rec, back_and_forth(Vec2f(8, 1.25), Vec2f(0, 1), 1.0, Control::VEL),
        -10));
    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        rec, back_and_forth(Vec2f(8, -1.25), Vec2f(0, -1), 1.0, Control::VEL),
        -10));

    Polyhedron2D short_rec;
    short_rec.add(Hyperplane2D(Vec2f(-0.5, 0), -Vec2f::UnitX()));
    short_rec.add(Hyperplane2D(Vec2f(0.5, 0), Vec2f::UnitX()));
    short_rec.add(Hyperplane2D(Vec2f(0, -0.5), -Vec2f::UnitY()));
    short_rec.add(Hyperplane2D(Vec2f(0, 0.5), Vec2f::UnitY()));

    Polyhedron2D long_rec;
    long_rec.add(Hyperplane2D(Vec2f(-0.5, 0), -Vec2f::UnitX()));
    long_rec.add(Hyperplane2D(Vec2f(0.5, 0), Vec2f::UnitX()));
    long_rec.add(Hyperplane2D(Vec2f(0, -2), -Vec2f::UnitY()));
    long_rec.add(Hyperplane2D(Vec2f(0, 2), Vec2f::UnitY()));

    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        short_rec,
        back_and_forth(Vec2f(12, 2.0), Vec2f(0, 1), 1.5, Control::VEL), -18));
    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        long_rec,
        back_and_forth(Vec2f(12, -0.5), Vec2f(0, -1), 1.5, Control::VEL), -18));

    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        long_rec,
        back_and_forth(Vec2f(16, 0.5), Vec2f(0, 1), 1.5, Control::VEL), -20));
    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        short_rec,
        back_and_forth(Vec2f(16, -2.0), Vec2f(0, -1), 1.5, Control::VEL), -20));

    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        short_rec,
        back_and_forth(Vec2f(3, -2.0), Vec2f(1, 1), 2.0, Control::ACC), -3));

    nonlinear_obs.push_back(PolyhedronNonlinearObstacle2D(
        short_rec,
        back_and_forth(Vec2f(7, -2.0), Vec2f(-1, 1), 2.0, Control::ACC), -3));

    printf("Initialized obstacle config 1!\n\n");
  }
};
