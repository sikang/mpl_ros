/**
 * @file primitive_util.h
 * @brief Simple primitive utils
 */
#ifndef PRIMITIVE_UTIL_H
#define PRIMITIVE_UTIL_H

#include <primitive/primitive.h>
#include <primitive/trajectory.h>

///Print all coefficients in primitive p
void print_coeffs(const Primitive& p);
///Print max dynamic infomation in primitive p
void print_max(const Primitive& p);

Ellipsoid generate_ellipsoid(const Vec3f& axe, const Vec3f& pos, const Vec3f& acc);

vec_Ellipsoid sample_ellipsoids(const Primitive& pr, const Vec3f& axe, int N);

vec_Ellipsoid sample_ellipsoids(const Trajectory& traj, const Vec3f& axe, int N);

void max_attitude(const Trajectory& traj, int N);
#endif

