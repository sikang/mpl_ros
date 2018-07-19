#ifndef MPL_DATA_ROS_UTILS_H
#define MPL_DATA_ROS_UTILS_H

#include <mpl_basis/data_type.h>
#include <geometry_msgs/Twist.h>
#include <planning_ros_msgs/PathArray.h>
#include <sensor_msgs/PointCloud.h>
#include <tf_conversions/tf_eigen.h>

inline vec_Vec3f vec2_to_vec3(const vec_Vec2f& pts2d, decimal_t z = 0) {
  vec_Vec3f pts(pts2d.size());

  for(size_t i = 0; i < pts.size(); i++)
    pts[i] = Vec3f(pts2d[i](0), pts2d[i](1), z);

  return pts;
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

inline Aff3f toTF(const geometry_msgs::Pose &p) {
  tf::Pose Ttf;
  tf::poseMsgToTF(p, Ttf);
  Eigen::Affine3d Td;
  tf::poseTFToEigen(Ttf, Td);
  return Td.cast<decimal_t>();
}

inline vec_Vec3f ros_to_path(const planning_ros_msgs::Path &msg) {
  vec_Vec3f path;
  for (const auto &it : msg.waypoints)
    path.push_back(Vec3f(it.x, it.y, it.z));
  return path;
}

inline planning_ros_msgs::Path path_to_ros(const vec_Vec3f &path) {
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

inline planning_ros_msgs::PathArray
path_array_to_ros(const vec_E<vec_Vec3f> &paths) {
  planning_ros_msgs::PathArray msg;
  for (const auto &it : paths) {
    planning_ros_msgs::Path path_msg;
    for (const auto &itt : it) {
      geometry_msgs::Point pt;
      pt.x = itt(0);
      pt.y = itt(1);
      pt.z = itt(2);
      path_msg.waypoints.push_back(pt);
    }
    msg.paths.push_back(path_msg);
  }
  return msg;
}

inline planning_ros_msgs::PathArray
path_array_to_ros(const std::vector<std::pair<std::string, vec_Vec3f>> &paths) {
  planning_ros_msgs::PathArray msg;
  for (const auto &it : paths) {
    planning_ros_msgs::Path path_msg;
    path_msg.name = it.first;
    for (const auto &itt : it.second) {
      geometry_msgs::Point pt;
      pt.x = itt(0);
      pt.y = itt(1);
      pt.z = itt(2);
      path_msg.waypoints.push_back(pt);
    }
    msg.paths.push_back(path_msg);
  }
  return msg;
}

#endif
