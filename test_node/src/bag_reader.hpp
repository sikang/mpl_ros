#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <planning_ros_msgs/VoxelMap.h>

planning_ros_msgs::VoxelMap read_bag(std::string file_name, std::string topic) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool find = false;
  planning_ros_msgs::VoxelMap map;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    planning_ros_msgs::VoxelMap::ConstPtr map_ptr = m.instantiate<planning_ros_msgs::VoxelMap>();
    if (map_ptr != NULL) {
      map = *map_ptr;
      ROS_WARN("Get data!");
      find = true;
      break;
    }
  }
  bag.close();
  if (!find)
    ROS_WARN("Fail to find '%s' in '%s'", topic.c_str(), file_name.c_str());
  return map;
}
