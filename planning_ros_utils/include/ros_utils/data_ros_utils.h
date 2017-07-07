#ifndef DATA_ROS_UTILS_H
#define DATA_ROS_UTILS_H

#include <motion_primitive_library/data_type.h>
#include <poly_utils/data_type.h>
#include <sensor_msgs/PointCloud.h>
#include <planning_ros_msgs/Polyhedra.h>
#include <planning_ros_msgs/Path.h>
#include <planning_ros_msgs/Mesh.h>
#include <planning_ros_msgs/Arrows.h>
#include <planning_ros_msgs/Ellipsoids.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

inline Vec3f pose_to_eigen(const geometry_msgs::Pose &pose) {
  return Vec3f(pose.position.x, pose.position.y, pose.position.z);
}

inline Vec3f twist_to_eigen(const geometry_msgs::Twist &twist) {
  return Vec3f(twist.linear.x, twist.linear.y, twist.linear.z);
}

inline Vec3f vec_to_eigen(const geometry_msgs::Vector3 &v) {
  return Vec3f(v.x, v.y, v.z);
}

inline geometry_msgs::Pose eigen_to_pose(const Vec3f& pose) {
  geometry_msgs::Pose p;
  p.position.x = pose(0);
  p.position.y = pose(1);
  p.position.z = pose(2);
  p.orientation.w = 1.0;
  return p;
}


inline geometry_msgs::Twist eigen_to_twist(const Vec3f& twist) {
  geometry_msgs::Twist t;
  t.linear.x = twist(0);
  t.linear.y = twist(1);
  t.linear.z = twist(2);
  return t;
}

inline vec_Vec3f path_to_eigen(const nav_msgs::Path &path) {
  vec_Vec3f vs;
  for (auto it : path.poses) {
    Vec3f v(it.pose.position.x, it.pose.position.y, it.pose.position.z);

    vs.push_back(v);
  }

  return vs;
}

inline nav_msgs::Path eigen_to_path(const vec_Vec3f &vs) {
  nav_msgs::Path path;
  for (auto it : vs) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = it(2);
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}

inline sensor_msgs::PointCloud
transform_cloud(const sensor_msgs::PointCloud &cloud,
                const Aff3f &TF) {
  sensor_msgs::PointCloud new_cloud = cloud;
  int i = 0;
  for (const auto& it : cloud.points) {
    Vec3f raw(it.x, it.y, it.z);
    raw = TF * raw;
    new_cloud.points[i].x = raw(0);
    new_cloud.points[i].y = raw(1);
    new_cloud.points[i].z = raw(2);
    i++;
  }
  return new_cloud;
}

inline sensor_msgs::PointCloud vec_to_cloud(const vec_Vec3f &pts) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());

  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }
  return cloud;
}

inline vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }

  return pts;
}

inline Aff3f toTF(const geometry_msgs::Pose &p)
{
  tf::Pose Ttf;
  tf::poseMsgToTF(p, Ttf);
  Eigen::Affine3d Td;
  tf::poseTFToEigen(Ttf, Td);
  return Td.cast<decimal_t>();
}

inline planning_ros_msgs::Path path_to_mav(const vec_Vec3f& path) {
  planning_ros_msgs::Path msg;
  for (const auto &itt : path) {
    geometry_msgs::Point pt;
    pt.x = itt(0);
    pt.y = itt(1);
    pt.z = itt(2);
    msg.waypoints.push_back(pt);
  }
  return msg;
}

inline vec_Vec3f mav_to_path(const planning_ros_msgs::Path& msg) {
  vec_Vec3f path;
  for (const auto &it : msg.waypoints)
    path.push_back(Vec3f(it.x, it.y, it.z));
  return path;
}

inline Polyhedra mav_to_polyhedra(const planning_ros_msgs::Polyhedra& msg){
  Polyhedra polys;
  for(const auto& polyhedron: msg.polyhedra){
    Polyhedron p;
    for(unsigned int i = 0; i < polyhedron.points.size(); i++){
      Vec3f pt(polyhedron.points[i].x,
               polyhedron.points[i].y,
               polyhedron.points[i].z);
      Vec3f n(polyhedron.normals[i].x,
              polyhedron.normals[i].y,
              polyhedron.normals[i].z);
      if(polyhedron.passes.empty())
        p.push_back(Face(pt, n));
      else
        p.push_back(Face(pt, n, polyhedron.passes[i]));
    }
    polys.push_back(p);
  }
  return polys;
}

inline planning_ros_msgs::Polyhedra polyhedra_to_mav(const Polyhedra& vs){
  planning_ros_msgs::Polyhedra poly;
  for (const auto &v : vs) {
    planning_ros_msgs::Polyhedron f;
    for (const auto &p : v) {
      geometry_msgs::Point pt, n;
      pt.x = p.p(0);
      pt.y = p.p(1);
      pt.z = p.p(2);
      n.x = p.n(0);
      n.y = p.n(1);
      n.z = p.n(2);
      f.points.push_back(pt);
      f.normals.push_back(n);
      f.passes.push_back(p.pass);
    }
    poly.polyhedra.push_back(f);
  }

  return poly;
}

inline planning_ros_msgs::Ellipsoids ellipsoids_to_mav(const vec_Ellipsoid& Es) {
  planning_ros_msgs::Ellipsoids ellipsoids;
  for (unsigned int i = 0; i < Es.size(); i++) {
    planning_ros_msgs::Ellipsoid ellipsoid;
    ellipsoid.d[0] = Es[i].second(0);
    ellipsoid.d[1] = Es[i].second(1);
    ellipsoid.d[2] = Es[i].second(2);

    for (int x = 0; x < 3; x++)
      for (int y = 0; y < 3; y++)
        ellipsoid.E[3 * x + y] = Es[i].first(x, y);
    ellipsoids.ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids;
}


inline Polyhedron vertices_to_poly(const BoundVec3f& vs) {
  Polyhedron pl;
  if(vs.size() < 4)
  {
    printf("cannot form a polyhedron as the num of faces %zu < 4\n", vs.size());
    return pl;
  }

  vec_Vec3f cs;
  for(const auto &v: vs) {
    Vec3f center = Vec3f::Zero();
    for(const auto &pt: v)
      center += pt;
    center /= v.size();
    cs.push_back(center);
  }

  for(int i = 0; i < (int)vs.size(); i++) {
    Vec3f p = cs[i];
    Vec3f n = (p - vs[i][0]).cross(p-vs[i][1]);
    n = n.normalized();
    int j = i + 1;
    if(j >= (int)vs.size())
      j = 0;
    if(n.dot(cs[j]-cs[i]) > 0)
      n = -n;

    pl.push_back(Face(p, n, true));
  }

  return pl;
}

inline planning_ros_msgs::Mesh bound_to_mesh(const BoundVec3f& bds) {
  planning_ros_msgs::Mesh mesh;

  for(const auto& face: bds) {
    planning_ros_msgs::Face f;
    for(const auto& v: face) {
      geometry_msgs::Point pt;
      pt.x = v(0);
      pt.y = v(1);
      pt.z = v(2);
      f.vertices.push_back(pt);
    }
    mesh.faces.push_back(f);
  }
  return mesh;
}

inline planning_ros_msgs::Arrows pairs_to_arrows(const vec_E<std::pair<Vec3f, Vec3f>>& vs) {
  planning_ros_msgs::Arrows msg;

  for(const auto& v: vs) {
    geometry_msgs::Point pt;
    pt.x = v.first(0);
    pt.y = v.first(1);
    pt.z = v.first(2);
    geometry_msgs::Point dir;
    dir.x = v.second(0);
    dir.y = v.second(1);
    dir.z = v.second(2);
    msg.points.push_back(pt);
    msg.directions.push_back(dir);
  }

  return msg;
}

#endif
