#include <primitive/primitive_util.h>

void print_coeffs(const Primitive& p) {
  printf("coeffs: t = %f\n", p.t());
  std::cout << "x:     " << p.traj(0).coeff().transpose() << std::endl;
  std::cout << "y:     " << p.traj(1).coeff().transpose() << std::endl;
  std::cout << "z:     " << p.traj(2).coeff().transpose() << std::endl;
}

void print_max(const Primitive& p) {
  Vec3f max_v, max_a;
  for(int i = 0; i < 3; i++) {
    max_v(i) = p.max_vel(i);
    max_a(i) = p.max_acc(i);
  }
  printf("max_vel: [%f, %f, %f]\n", max_v(0), max_v(1), max_v(2));
  printf("max_acc: [%f, %f, %f]\n", max_a(0), max_a(1), max_a(2));
}

Ellipsoid generate_ellipsoid(const Vec3f& axe, const Vec3f& pos, const Vec3f& acc) {
  const Vec3f b3 = (acc + 9.81 * Vec3f::UnitZ()).normalized();
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
  return std::make_pair(C, pos);
}


vec_Ellipsoid sample_ellipsoids(const Primitive& pr, const Vec3f& axe, int N) {
  vec_Ellipsoid Es;
  decimal_t dt = pr.t() / N;
  for(decimal_t t = 0; t <= pr.t(); t+= dt) {
    const Waypoint pt = pr.evaluate(t);
    Es.push_back(generate_ellipsoid(axe, pt.pos, pt.acc));
  }

  return Es;
}

vec_Ellipsoid sample_ellipsoids(const Trajectory& traj, const Vec3f& axe, int N) {
  vec_Ellipsoid Es;

  decimal_t dt = traj.getTotalTime() / N;
  for(decimal_t t = 0; t <= traj.getTotalTime(); t+= dt) {
    Waypoint pt;
    if(traj.evaluate(t, pt)) {
     Es.push_back(generate_ellipsoid(axe, pt.pos, pt.acc));
    }
  }

  return Es;
}

void max_attitude(const Trajectory& traj, int N) {
  decimal_t dt = traj.getTotalTime() / N;
  decimal_t max_roll = 0;
  decimal_t max_roll_time = 0;
  decimal_t max_pitch = 0;
  decimal_t max_pitch_time = 0;
  for(decimal_t t = 0; t <= traj.getTotalTime(); t+= dt) {
    Waypoint pt;
    if(traj.evaluate(t, pt)) {
      const Vec3f b3 = (pt.acc + 9.81 * Vec3f::UnitZ()).normalized();
      decimal_t roll = std::atan2(b3(1), b3(2));
      decimal_t pitch= std::atan2(b3(0), b3(2));
      if(std::fabs(roll) > std::fabs(max_roll)) {
        max_roll = roll;
        max_roll_time = t;
      }
      if(std::fabs(pitch) > std::fabs(max_pitch)) {
        max_pitch = pitch;
        max_pitch_time = t;
      }

    }
  }

  printf("max roll: %f at [%f]\n", max_roll * 180/M_PI, max_roll_time);
  printf("max pitch: %f at [%f]\n", max_pitch * 180/M_PI, max_pitch_time);
}

