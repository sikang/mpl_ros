/**
 * @file seed_decomp.h
 * @brief SeedDecomp Class
 */
#ifndef SEED_DECOMP_H
#define SEED_DECOMP_H

#include <decomp_util/data_type.h>
#include <decomp_util/geometry_utils.h>


/**
 * @brief Seed Decomp Class
 *
 * Dilate around the given point
 */
class SeedDecomp{
  public:
    ///Simple constructor
    SeedDecomp() {};
    /**
     * @brief Basic constructor
     * @param p1 One end of the line seg
     * @param p2 The other end of the line seg
     */
    SeedDecomp(const Vec3f &p);

    /**
     * @brief Adding virtual bounding box around line seg
     * @param x Distance in x axis
     * @param y Distance in y axis
     * @param v Distance in z axis
     *
     * This virtual bounding box is parallel to the line segment, the x,y,z axes are not w.r.t the world coordinate system, but instead, x-axis is parallel to the line, y-axis is perpendicular to the line and world z-axis, z-axis is perpendiculat to the line and y-axis 
     */
    void set_virtual_dim(decimal_t x, decimal_t y, decimal_t z);
    ///Import obstacle points
    void set_obstacles(const vec_Vec3f &obs);
    ///Retrieve the radius of sphere
    decimal_t radius() const { return radius_; }
    ///Retieve obstacel points
    vec_Vec3f obs() const { return obs_; }
    ///Retrieve ellipsoid
    Ellipsoid ellipsoid() const { return ellipsoid_; }
    ///Retrieve polyhedron
    Polyhedron polyhedron() const { return polyhedron_; }
    /**
     * @brief Infalte the line segment
     * @param radius Robot radius
     */
    void dilate(decimal_t radius);
    void dilate(const Vec3f& axes, const Mat3f& R);
    void dilate(const Ellipsoid& E);
    /**
     * @brief Shrink the polyhedron 
     * @param thr Shrinking distance
     */
    void shrink(decimal_t thr);
 
  private:
    void add_virtual_wall(Polyhedron &Vs);

    decimal_t radius_ = 1.0;
    vec_Vec3f obs_;
    Vec3f p_;
    Ellipsoid ellipsoid_;
    Polyhedron polyhedron_;
    decimal_t virtual_x_ = 5;
    decimal_t virtual_y_ = 5;
    decimal_t virtual_z_ = 2;
};

#endif 
