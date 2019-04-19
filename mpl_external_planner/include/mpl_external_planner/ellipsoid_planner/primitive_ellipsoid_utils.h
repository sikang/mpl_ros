/**
 * @file ellipsoid_utils.h
 * @brief Basic ellipoid utils
 */
#ifndef MPL_PRIMITIVE_ELLIPSOID_UTILS_H
#define MPL_PRIMITIVE_ELLIPSOID_UTILS_H
#include <decomp_geometry/ellipsoid.h>
#include <mpl_basis/trajectory.h>

/**
 * @brief estimate the ellipsoid in 3D
 * @param axe the length of semi-axes of the ellipsoid
 * @param pos center of ellipsoid
 * @param acc acceleration
 */
template <int Dim>
Ellipsoid3D generate_ellipsoid(const Vec3f& axe, const Vecf<Dim>& pos,
                               const Vecf<Dim>& acc) {
  Vec3f ellipsoid_center = Dim == 2 ? Vec3f(pos(0), pos(1), 0) : pos;
  Vec3f ellipsoid_acc = Dim == 2 ? Vec3f(acc(0), acc(1), 0) : acc;

  const Vec3f b3 = (ellipsoid_acc + 9.81 * Vec3f::UnitZ()).normalized();
  const Vec3f bc(std::cos(0), std::sin(0), 0);
  const Vec3f b2 = b3.cross(bc).normalized();
  const Vec3f b1 = b2.cross(b3).normalized();
  Mat3f R;
  R << b1, b2, b3;

  Mat3f C = Mat3f::Identity();
  C(0, 0) = axe(0);
  C(1, 1) = axe(1);
  C(2, 2) = axe(2);
  C = R * C * R.transpose();
  return Ellipsoid3D(C, ellipsoid_center);
}

/// Sample N+1 ellipsoids along the primitive
template <int Dim>
vec_E<Ellipsoid3D> sample_ellipsoids(const Primitive<Dim>& pr, const Vec3f& axe,
                                     int N) {
  vec_E<Ellipsoid3D> Es(N + 1);
  decimal_t dt = pr.t() / N;
  for (int i = 0; i <= N; i++) {
    const auto pt = pr.evaluate(i * dt);
    /*
    std::cout << "t:" << i*dt << std::endl;
    std::cout << "pos: " << pt.pos.transpose() << std::endl;
    std::cout << "acc: " << pt.acc.transpose() << std::endl;
    if(std::isnan(pt.pos(0)))
      print(pr);
      */

    Es[i] = generate_ellipsoid<Dim>(axe, pt.pos, pt.acc);
  }

  return Es;
}

/// Sample N ellipsoids along the trajectory
template <int Dim>
vec_E<Ellipsoid3D> sample_ellipsoids(const Trajectory<Dim>& traj,
                                     const Vec3f& axe, int N) {
  vec_E<Ellipsoid3D> Es;

  decimal_t dt = traj.getTotalTime() / N;
  for (decimal_t t = 0; t <= traj.getTotalTime(); t += dt) {
    Command<Dim> pt;
    if (traj.evaluate(t, pt))
      Es.push_back(generate_ellipsoid<Dim>(axe, pt.pos, pt.acc));
  }

  return Es;
}

/// Approximate the maximum roll/pitch along the trajectory
template <int Dim>
void max_attitude(const Trajectory<Dim>& traj, int N) {
  decimal_t dt = traj.getTotalTime() / N;
  decimal_t max_roll = 0;
  decimal_t max_roll_time = 0;
  decimal_t max_pitch = 0;
  decimal_t max_pitch_time = 0;
  for (decimal_t t = 0; t <= traj.getTotalTime(); t += dt) {
    Command<Dim> pt;
    if (traj.evaluate(t, pt)) {
      const Vec3f b3 =
          Dim == 2 ? (Vec3f(pt.acc(0), pt.acc(1), 0) + 9.81 * Vec3f::UnitZ())
                         .normalized()
                   : (pt.acc + 9.81 * Vec3f::UnitZ()).normalized();
      decimal_t roll = std::atan2(b3(1), b3(2));
      decimal_t pitch = std::atan2(b3(0), b3(2));
      if (std::fabs(roll) > std::fabs(max_roll)) {
        max_roll = roll;
        max_roll_time = t;
      }
      if (std::fabs(pitch) > std::fabs(max_pitch)) {
        max_pitch = pitch;
        max_pitch_time = t;
      }
    }
  }

  printf("max roll: %f at [%f]\n", max_roll * 180 / M_PI, max_roll_time);
  printf("max pitch: %f at [%f]\n", max_pitch * 180 / M_PI, max_pitch_time);
}

#endif
