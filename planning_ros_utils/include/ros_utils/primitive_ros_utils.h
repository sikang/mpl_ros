/**
 * @file primitive_ros_utils.h
 * @brief Interface between primitive classes and ros
 */
#pragma once
#include <planning_ros_msgs/Trajectories.h>
#include <primitive/trajectory.h>

/**
 * @brief Primitive class to primitive ros message
 */
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive& p);

/**
 * @brief Trajectory class to trajectory ros message
 */
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory& traj);
/**
 * @brief Multiple trajectories class to trajectories ros message for visualization
 */
planning_ros_msgs::Trajectories toTrajectoriesROSMsg(const std::vector<Trajectory>& trajs);
/**
 * @brief Ros message to primitive class
 */
Primitive toPrimitive(const planning_ros_msgs::Primitive& p);

/**
 * @brief Ros message to trajectory class 
 */
Trajectory toTrajectory(const planning_ros_msgs::Trajectory & ps);

