/**
 * @file primitive_ros_utils.h
 * @brief Interface between primitive classes and ROS
 */
#pragma once
#include <planning_ros_msgs/Trajectory.h>
#include <planning_ros_msgs/Primitives.h>
#include <motion_primitive_library/primitive/trajectory.h>

/**
 * @brief Primitive class to primitive ROS message
 */
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive& pr);
/**
 * @brief Multiple Primitive class to Primitive ROS message
 */
planning_ros_msgs::Primitives toPrimitivesROSMsg(const std::vector<Primitive>& prs);
/**
 * @brief Trajectory class to trajectory ROS message
 */
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory& traj);
/**
 * @brief ROS message to primitive class
 */
Primitive toPrimitive(const planning_ros_msgs::Primitive& pr_msg);

/**
 * @brief ROS message to trajectory class 
 */
Trajectory toTrajectory(const planning_ros_msgs::Trajectory & traj_msg);

