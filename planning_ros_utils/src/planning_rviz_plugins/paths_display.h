#include <boost/circular_buffer.hpp>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <planning_ros_msgs/Paths.h>
#include <rviz/message_filter_display.h>

#include "path_visual.h"

namespace planning_rviz_plugins {
  class PathsDisplay
    : public rviz::MessageFilterDisplay<planning_ros_msgs::Paths> {
      Q_OBJECT
      public:
        PathsDisplay();
        virtual ~PathsDisplay();

      protected:
        virtual void onInitialize();

        virtual void reset();

        private Q_SLOTS:
          void updateLineColorAndAlpha();
        void updateNodeColorAndAlpha();
        void updateLineScale();
        void updateNodeScale();
        void updateHistoryLength();
        void updateID();

      private:
        void processMessage(const planning_ros_msgs::Paths::ConstPtr &msg);
        void visualizeMessage(int state);

        boost::circular_buffer<boost::shared_ptr<PathVisual>> visuals_;

        rviz::ColorProperty *line_color_property_;
        rviz::ColorProperty *node_color_property_;
        rviz::FloatProperty *line_scale_property_;
        rviz::FloatProperty *node_scale_property_;
        rviz::IntProperty *history_length_property_;
        rviz::EnumProperty *id_property_;

        Ogre::Vector3 position_;
        Ogre::Quaternion orientation_;

        planning_ros_msgs::Paths paths_;
    };

}
