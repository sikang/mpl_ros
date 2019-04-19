#include <mpl_traj_solver/traj_solver.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

#include "bag_reader.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher min_vel_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>(
      "min_vel_trajectory", 1, true);
  ros::Publisher min_acc_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>(
      "min_acc_trajectory", 1, true);
  ros::Publisher min_jrk_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>(
      "min_jrk_trajectory", 1, true);
  ros::Publisher min_snp_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>(
      "min_snp_trajectory", 1, true);

  ros::Publisher path_pub =
      nh.advertise<planning_ros_msgs::Path>("path", 1, true);

  // Standard header
  std_msgs::Header header;
  header.frame_id = std::string("map");

  vec_Vec3f path;
  path.push_back(Vec3f(0, 0, 0));
  path.push_back(Vec3f(1, 0, 0));
  path.push_back(Vec3f(2, 1, 0));
  path.push_back(Vec3f(5, 1, 0));

  auto path_msg = path_to_ros(path);
  path_msg.header = header;
  path_pub.publish(path_msg);

  /// Min Vel Traj
  {
    TrajSolver3D traj_solver(Control::VEL);
    traj_solver.setPath(path);
    const auto traj = traj_solver.solve();
    // Publish trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    min_vel_traj_pub.publish(traj_msg);
  }
  /// Min Acc Traj
  {
    TrajSolver3D traj_solver(Control::ACC);
    traj_solver.setPath(path);
    const auto traj = traj_solver.solve();
    // Publish trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    min_acc_traj_pub.publish(traj_msg);
  }
  /// Min Jrk Traj
  {
    TrajSolver3D traj_solver(Control::JRK);
    traj_solver.setPath(path);
    const auto traj = traj_solver.solve();
    // Publish trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    min_jrk_traj_pub.publish(traj_msg);
  }
  /// Min Snp Traj, does not work
  {
    TrajSolver3D traj_solver(Control::SNP);
    traj_solver.setPath(path);
    const auto traj = traj_solver.solve();
    // Publish trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    min_snp_traj_pub.publish(traj_msg);
  }

  ros::spin();

  return 0;
}
