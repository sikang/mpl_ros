/**
 * @file primitive_ros_utils.h
 * @brief Interface between primitive classes and ROS
 */
#pragma once
#include <planning_ros_msgs/Trajectory.h>
#include <planning_ros_msgs/PrimitiveArray.h>
#include <motion_primitive_library/primitive/trajectory.h>

///Primitive2 to primitive ROS message
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive2D& pr, double z = 0);
///Primitive3 to primitive ROS message
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive3D& pr);
///Multiple Primitive2 to Primitive ROS message
planning_ros_msgs::PrimitiveArray toPrimitiveArrayROSMsg(const vec_E<Primitive2D>& prs, double z = 0);
///Multiple Primitive3 to Primitive ROS message
planning_ros_msgs::PrimitiveArray toPrimitiveArrayROSMsg(const vec_E<Primitive3D>& prs);
///Trajectory2 class to trajectory ROS message
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory2D& traj, double z = 0);
///Trajectory3 class to trajectory ROS message
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory3D& traj);
///ROS message to Primitive2 class
Primitive2D toPrimitive2D(const planning_ros_msgs::Primitive& pr_msg);
///ROS message to Primitive3 class
Primitive3D toPrimitive3D(const planning_ros_msgs::Primitive& pr_msg);
///ROS message to Trajectory2 class
Trajectory2D toTrajectory2D(const planning_ros_msgs::Trajectory& traj_msg);
///ROS message to Trajectory3 class
Trajectory3D toTrajectory3D(const planning_ros_msgs::Trajectory& traj_msg);

