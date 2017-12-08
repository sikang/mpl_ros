#include <tf2_ros/transform_listener.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>

class TFListener {
public:
  TFListener() {
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    tfBuffer.setUsingDedicatedThread(true);
  }

  bool getPose(const ros::Time &t, const std::string &frame,
               const std::string &ref_frame, geometry_msgs::Pose &pose) {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped =
          tfBuffer.lookupTransform(ref_frame, frame, t, ros::Duration(0.4));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1, "Fail to find transform from [%s] to [%s]",
                        ref_frame.c_str(), frame.c_str());
      return false;
    }

    pose.position.x = transformStamped.transform.translation.x;
    pose.position.y = transformStamped.transform.translation.y;
    pose.position.z = transformStamped.transform.translation.z;
    pose.orientation.w = transformStamped.transform.rotation.w;
    pose.orientation.x = transformStamped.transform.rotation.x;
    pose.orientation.y = transformStamped.transform.rotation.y;
    pose.orientation.z = transformStamped.transform.rotation.z;

    return true;
  }

private:
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener *tfListener;
};
