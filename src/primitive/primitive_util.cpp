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


