#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

sensor_msgs::PointCloud2 read_point_cloud2(std::string file_name, std::string topic) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool find = false;
  sensor_msgs::PointCloud2 map;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    sensor_msgs::PointCloud2::ConstPtr map_ptr = m.instantiate<sensor_msgs::PointCloud2>();
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


visualization_msgs::MarkerArray read_marker_array(std::string file_name, std::string topic) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool find = false;
  visualization_msgs::MarkerArray map;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    visualization_msgs::MarkerArray::ConstPtr map_ptr = m.instantiate<visualization_msgs::MarkerArray>();
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
