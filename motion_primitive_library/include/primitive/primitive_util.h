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


#endif

