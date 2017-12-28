#include <deque>
#include <queue>

#include <sensor_msgs/PointCloud.h>
#include <planning_ros_msgs/VoxelMap.h>

#include <rviz/message_filter_display.h>

#include <tf/transform_listener.h>

#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>

#include <boost/circular_buffer.hpp>
#include <ros_utils/data_ros_utils.h>

#include <collision_checking/voxel_map_util.h>

#include "bound_visual.h"

namespace planning_rviz_plugins {
  class MapDisplay
    : public rviz::MessageFilterDisplay<planning_ros_msgs::VoxelMap> {
      Q_OBJECT
      public:
        MapDisplay();
        ~MapDisplay();

        virtual void reset();
        virtual void update(float wall_dt, float ros_dt);

        private Q_SLOTS:
          void updateQueueSize();
        void updateState();
        void updateScale();

      protected:
        void setMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, const planning_ros_msgs::VoxelMap& msg);
        void getMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, planning_ros_msgs::VoxelMap& map);


        virtual void onInitialize();

        virtual void processMessage(const planning_ros_msgs::VoxelMapConstPtr &map);
        virtual void visualizeMessage(int state);
        BoundVec3f getSpace();

        rviz::IntProperty *queue_size_property_;
        rviz::EnumProperty *state_property_;
        rviz::FloatProperty *scale_property_;

        rviz::PointCloudCommon *point_cloud_common_;
        boost::circular_buffer<boost::shared_ptr<BoundVisual>> visuals_;

        std::shared_ptr<MPL::VoxelMapUtil> map_util_;

        std_msgs::Header header_;
    };

} // namespace planning_rviz_plugins
