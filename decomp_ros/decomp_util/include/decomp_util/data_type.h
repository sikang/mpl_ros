/**
 * @file data_type.h
 * @brief Defines all data types used in this lib

 * Mostly alias from Eigen Library.
 */

#ifndef BASIC_DATA_H
#define BASIC_DATA_H
#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

///Set red font in printf funtion 
#define ANSI_COLOR_RED "\x1b[31m"
///Set green font in printf funtion 
#define ANSI_COLOR_GREEN "\x1b[32m"
///Set yellow font in printf funtion 
#define ANSI_COLOR_YELLOW "\x1b[33m"
///Set blue font in printf funtion 
#define ANSI_COLOR_BLUE "\x1b[34m"
///Set magenta font in printf funtion 
#define ANSI_COLOR_MAGENTA "\x1b[35m"
///Set cyan font in printf funtion 
#define ANSI_COLOR_CYAN "\x1b[36m"
///Reset font color in printf funtion 
#define ANSI_COLOR_RESET "\x1b[0m"

///Pre-allocated std::vector for Eigen.
template <typename T> 
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
/*! \brief Rename the float type used in lib 

    Default is set to be double, but user can change it to float.
*/
typedef double decimal_t;

///Column vector in float of size 2.
typedef Eigen::Matrix<decimal_t, 2, 1> Vec2f;
///Column vector in int of size 2.
typedef Eigen::Vector2i Vec2i;
///Column vector in float of size 3.
typedef Eigen::Matrix<decimal_t, 3, 1> Vec3f;
///Column vector in int of size 3.
typedef Eigen::Vector3i Vec3i;
///Column vector in float of size 4.
typedef Eigen::Matrix<decimal_t, 4, 1> Vec4f;
///Column vector in float of size 6.
typedef Eigen::Matrix<decimal_t, 6, 1> Vec6f;

///Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
///Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
///Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
///Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;

///2x2 Matrix in float
typedef Eigen::Matrix<decimal_t, 2, 2> Mat2f;
///3x3 Matrix in float
typedef Eigen::Matrix<decimal_t, 3, 3> Mat3f;
///4x4 Matrix in float
typedef Eigen::Matrix<decimal_t, 4, 4> Mat4f;
///6x6 Matrix in float
typedef Eigen::Matrix<decimal_t, 6, 6> Mat6f;

///Column vector in float with dynamic size
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> VecDf;
///Nx3 matrix in float 
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> MatD3f;
///NxN matrix in float
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, Eigen::Dynamic> MatDf;

//Allias of Eigen::Translation
typedef Eigen::Translation<decimal_t, 3> Trans3f;
//Allias of Eigen::AngleAxis
typedef Eigen::AngleAxis<decimal_t> Anglef;

///Allias of Eigen::Quaterniond
typedef Eigen::Quaternion<decimal_t> Quatf;
///Allias of Eigen::Affine2d
typedef Eigen::Transform<decimal_t, 2, Eigen::Affine> Aff2f;
///Allias of Eigen::Affine3d
typedef Eigen::Transform<decimal_t, 3, Eigen::Affine> Aff3f;
///std::pair of Eigen::Vector3d
typedef std::pair<Vec3f, Vec3f> pair_Vec3f;

///Ellipsoid: first is the Affine Transform, second is the center
typedef std::pair<Mat3f, Vec3f> Ellipsoid;
///Vector of Ellipsoids
typedef vec_E<Ellipsoid> vec_Ellipsoid;

///[A, b] for \f$Ax <= b\f$
typedef std::pair<MatD3f, VecDf> LinearConstraint3f; 
///Vector of LinearConstraint
typedef vec_E<LinearConstraint3f> vec_LinearConstraint3f;

struct Face {
  Vec3f p;
  Vec3f n;
  bool pass;

  Face(Vec3f _p, Vec3f _n):
    p(_p), n(_n), pass(true) {}
  Face(Vec3f _p, Vec3f _n, bool _pass):
    p(_p), n(_n), pass(_pass) {}
};

typedef vec_E<Face> Polyhedron; // composed by planes with form (p, n)
typedef vec_E<Polyhedron> Polyhedra;

typedef vec_E<vec_Vec3f> BoundVec3f; // compose by extreme points

#endif
